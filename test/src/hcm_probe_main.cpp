// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

//
// qrb_ros_adapter_stream_probe, the HCM side of a probed stream: a host that drives an
// HcmStreamProbeNode and an HcmIpcAdapterNode through QCNodeIfs only, on one
// frame they share.
//
// It initializes and starts both nodes, then reads commands from stdin; see
// `help`. On a terminal, command results and events scroll above a status
// panel redrawn every second (terminal_ui.hpp); otherwise stdout carries
// command results, events and the summary only. Both show the Adapter's
// calls and states, not the probe's. The --log file is plain text: a line
// per frame, a monitor line every 5 seconds, commands, events, the summary.
//
// Exit code: 0 ok, 1 a frame failed verification or belonged to another
// stream, 2 the automatic start failed with stdin closed or a Stop() or
// DeInitialize() at exit failed (2 wins over 1), 3 bad parameters.
//

#include <fcntl.h>
#include <linux/dma-heap.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "hcm_probe_node.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "terminal_ui.hpp"

namespace qrb::adapter::stream_probe
{

namespace
{

constexpr std::uint64_t kNsPerSecond = 1000000000ull;
/** Failed calls logged one by one per node; later ones are only counted. */
constexpr std::uint64_t kLoggedProblems = 10;
/** Seconds between the monitor lines of the --log file. */
constexpr std::uint64_t kLogMonitorSeconds = 5;
constexpr const char * kDmaHeap = "/dev/dma_heap/system";
constexpr const char * kProbeNodeName = "stream_probe";

constexpr int kExitOk = 0;
constexpr int kExitVerify = 1;
constexpr int kExitRuntime = 2;
constexpr int kExitParams = 3;

std::uint64_t SecondsToNs(double seconds)
{
  return static_cast<std::uint64_t>(seconds * static_cast<double>(kNsPerSecond));
}

double NsToSeconds(std::uint64_t ns)
{
  return static_cast<double>(ns) / static_cast<double>(kNsPerSecond);
}

std::string StatusName(QC::QCStatus_e status)
{
  switch (status) {
    case QC::QC_STATUS_OK:
      return "OK";
    case QC::QC_STATUS_BAD_ARGUMENTS:
      return "BAD_ARGUMENTS";
    case QC::QC_STATUS_BAD_STATE:
      return "BAD_STATE";
    case QC::QC_STATUS_NOMEM:
      return "NOMEM";
    case QC::QC_STATUS_INVALID_BUF:
      return "INVALID_BUF";
    case QC::QC_STATUS_UNSUPPORTED:
      return "UNSUPPORTED";
    case QC::QC_STATUS_ALREADY:
      return "ALREADY";
    case QC::QC_STATUS_TIMEOUT:
      return "TIMEOUT";
    case QC::QC_STATUS_OUT_OF_BOUND:
      return "OUT_OF_BOUND";
    case QC::QC_STATUS_FAIL:
      return "FAIL";
    case QC::QC_STATUS_NULL_PTR:
      return "NULL_PTR";
    case QC::QC_STATUS_NO_RESOURCE:
      return "NO_RESOURCE";
    default:
      return "STATUS_" + std::to_string(static_cast<int>(status));
  }
}

std::string StateName(QC::QCObjectState_e state)
{
  switch (state) {
    case QC::QC_OBJECT_STATE_INITIAL:
      return "INITIAL";
    case QC::QC_OBJECT_STATE_INITIALIZING:
      return "INITIALIZING";
    case QC::QC_OBJECT_STATE_READY:
      return "READY";
    case QC::QC_OBJECT_STATE_STARTING:
      return "STARTING";
    case QC::QC_OBJECT_STATE_RUNNING:
      return "RUNNING";
    case QC::QC_OBJECT_STATE_STOPING:
      return "STOPING";
    case QC::QC_OBJECT_STATE_ERROR:
      return "ERROR";
    case QC::QC_OBJECT_STATE_PAUSING:
      return "PAUSING";
    case QC::QC_OBJECT_STATE_PAUSE:
      return "PAUSE";
    case QC::QC_OBJECT_STATE_RESUMING:
      return "RESUMING";
    case QC::QC_OBJECT_STATE_DEINITIALIZING:
      return "DEINITIALIZING";
    default:
      return "STATE_" + std::to_string(static_cast<int>(state));
  }
}

// --- Contracts -------------------------------------------------------------------

struct HostContract
{
  const char * typeName;
  bool dmaBuf;
};

constexpr std::string_view kDmaBufPayload = "DmaBuf";

const HostContract kContracts[] = {
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ...) {TypeName, #Payload == kDmaBufPayload},
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
};

const HostContract * FindContract(const std::string & contract)
{
  for (const HostContract & entry : kContracts) {
    if (MatchesContract(contract, entry.typeName))
      return &entry;
  }
  return nullptr;
}

std::string SupportedContracts()
{
  std::string supported;
  for (const HostContract & entry : kContracts)
    supported += (supported.empty() ? "" : ", ") + ShortContractName(entry.typeName);
  return supported;
}

// --- Parameters --------------------------------------------------------------------

struct HostOptions
{
  ProbeParams params;
  std::string paramsFile;
  bool help{false};
};

/** @brief The static `direction` value of `direction`: 0, 1, or null when unconfigured. */
nlohmann::json ConfigDirection(adapter::Direction direction)
{
  const std::int64_t code = adapter::ToDirectionCode(direction);
  return code < 0 ? nlohmann::json(nullptr) : nlohmann::json(code);
}

constexpr const char * kCommandHelp =
    "commands, one per line on stdin, by name or by number (e.g. 7, or 1 FILE):\n"
    "  [1] init [FILE]  Initialize() both nodes; FILE: another Adapter static JSON, taken\n"
    "                   only while both nodes are INITIAL (default: the startup config)\n"
    "  [2] start        Start() both nodes; the frame loop runs\n"
    "  [3] stop         Stop() both nodes; the frame loop pauses\n"
    "  [4] deinit       DeInitialize() both nodes\n"
    "  [5] get_state    the Adapter's state\n"
    "  [6] get_config   the Adapter's configuration read back\n"
    "  [7] get_monitor  the Adapter's monitoring and the stream's statistics\n"
    "  [8] help         this list\n"
    "  [9] quit         Stop() and DeInitialize(), print the summary and exit, as SIGINT does";

/** @brief The commands in the order of their numbers, from 1. */
constexpr const char * kCommandNames[] = {
    "init", "start", "stop", "deinit", "get_state", "get_config", "get_monitor", "help", "quit"};

/** @brief The commands shown above the input line of the terminal screen. */
constexpr const char * kCommandBar =
    "[1]init [2]start [3]stop [4]deinit [5]get_state [6]get_config [7]get_monitor [8]help [9]quit";

/** @brief The installed probe_configs directory. */
std::string ConfigDir()
{
  char path[4096];
  const ssize_t length = ::readlink("/proc/self/exe", path, sizeof(path) - 1);
  if (length <= 0)
    throw std::runtime_error("cannot find the default configs");
  path[length] = '\0';
  std::string exe(path);
  // Installed as <prefix>/bin/qrb_ros_adapter_stream_probe.
  const std::string dir = exe.substr(0, exe.rfind('/'));
  return dir + "/../share/qrb_ros_adapter_stream_probe/probe_configs";
}

/** @brief `--key value` or `--key=value`; the type of a key is that of its default. */
HostOptions ParseCommandLine(int argc, char ** argv)
{
  HostOptions options;
  const std::string configDir = ConfigDir();
  options.params = DefaultParams(configDir);
  const nlohmann::json defaults = ToJson(options.params);
  nlohmann::json fromCli = nlohmann::json::object();
  std::string paramsFile;
  bool paramsGiven = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      options.help = true;
      return options;
    }
    if (arg.rfind("--", 0) != 0)
      throw std::invalid_argument("unexpected argument " + arg);
    std::string key = arg.substr(2);
    std::string value;
    const std::size_t equals = key.find('=');
    if (equals != std::string::npos) {
      value = key.substr(equals + 1);
      key.resize(equals);
    } else {
      if (i + 1 >= argc)
        throw std::invalid_argument("option " + arg + " needs a value");
      value = argv[++i];
    }
    const std::string option = "--" + key;
    for (char & c : key) {
      if (c == '-')
        c = '_';
    }

    if (key == "params") {
      paramsFile = value;
      paramsGiven = true;
      continue;
    }
    auto known = defaults.find(key);
    if (known == defaults.end())
      throw std::invalid_argument("unknown option " + option);

    const char * begin = value.c_str();
    char * end = nullptr;
    errno = 0;
    if (known->is_number_integer()) {
      if (!value.empty() && value[0] == '-') {
        const long long v = std::strtoll(begin, &end, 10);
        if (end == begin || *end != '\0' || errno != 0)
          throw std::invalid_argument(option + " must be an integer");
        fromCli[key] = static_cast<std::int64_t>(v);  // ApplyJson rejects it
      } else {
        const unsigned long long v = std::strtoull(begin, &end, 10);
        if (value.empty() || end == begin || *end != '\0' || errno != 0)
          throw std::invalid_argument(option + " must be an integer");
        fromCli[key] = static_cast<std::uint64_t>(v);
      }
    } else if (known->is_number()) {
      const double v = std::strtod(begin, &end);
      if (value.empty() || end == begin || *end != '\0' || errno != 0)
        throw std::invalid_argument(option + " must be a number");
      fromCli[key] = v;
    } else
      fromCli[key] = value;
  }

  options.paramsFile = paramsGiven ? paramsFile : configDir + "/stream_probe.json";
  ApplyJsonFile(options.params, options.paramsFile);
  ApplyJson(options.params, fromCli, "command line");
  Validate(options.params);
  return options;
}

/** @brief The DMA bytes of the host frame: what a sender sends, what a receiver can take. */
std::uint64_t FrameBytes(const ProbeParams & params, bool sender)
{
  return sender ? params.payloadBytes : params.bufferBytes;
}

/** @brief The probe node's static config for `flow`. */
std::string ProbeNodeConfig(const ProbeParams & params,
    const ProbeFlow & flow,
    const char * typeName,
    std::uint32_t slot)
{
  nlohmann::json node{{"name", kProbeNodeName}, {"id", 0},
      {"direction", static_cast<int>(flow.direction)}, {"contract", typeName},
      {"topic", flow.topic}, {"payload_bytes", params.payloadBytes}, {"warmup", params.warmup},
      {"globalBufferIdMap", nlohmann::json::array({{{"name", "Probe"}, {"id", slot}}})}};
  return nlohmann::json{{"static", node}}.dump();
}

void PrintHelp()
{
  std::printf(
      "usage: qrb_ros_adapter_stream_probe [--params FILE] [--OPTION VALUE ...]\n"
      "\n"
      "The HCM side of a probed stream through the HCM IPC Adapter. Parameters\n"
      "come from the built-in defaults, then the parameter file (default\n"
      "share/qrb_ros_adapter_stream_probe/probe_configs/stream_probe.json), then the command "
      "line.\n"
      "\n"
      "  --adapter-config FILE  the Adapter's static JSON (probe_configs/adapter_h2r.json);\n"
      "                         the probe follows its direction (0: HCM to ROS, this side\n"
      "                         sends; 1: ROS to HCM, this side receives), contract_type\n"
      "                         (one of: %s), ros_topic and globalBufferIdMap\n"
      "  --rate FPS             frames per second of a sender, 0: unlimited (30)\n"
      "  --payload-bytes N      payload of each frame this side sends (1048576)\n"
      "  --buffer-bytes N       DMA buffer this side receives into; a larger sample\n"
      "                         is dropped as a frame error (33554432)\n"
      "  --duration S           seconds to run, 0: until quit or a signal (0)\n"
      "  --warmup S             seconds a receiver does not measure (0)\n"
      "  --log FILE             text log: a line per frame sent or received, a monitor line\n"
      "                         every 5 s, commands, events and the summary (none)\n"
      "\n"
      "%s\n",
      SupportedContracts().c_str(), kCommandHelp);
}

/**
 * @brief How this run is set up and how to drive it, shown before anything
 * starts; the command list only when `commands`.
 */
std::string StartupHelp(const HostOptions & options,
    const ProbeFlow & flow,
    const HostContract & contract,
    bool commands)
{
  const ProbeParams & params = options.params;
  const bool sender = flow.direction == ProbeDirection::kHcmToRos;
  return FormatText(
      "qrb_ros_adapter_stream_probe: the HCM side of a probed stream through the HCM IPC Adapter\n"
      "  params file     %s\n"
      "  adapter config  %s\n"
      "  flow            direction %d (%s, this side %s), %s on %s\n"
      "                  QoS %s, %s, depth %u; descriptor at buffer %u\n"
      "  run             rate %g fps%s, %s %llu bytes, duration %g s%s, warmup %g s, log %s\n"
      "Start ros_ipc_bridge and the ROS-side probe first. Parameters: --OPTION VALUE or\n"
      "--params FILE, see --help; the Flow comes from the adapter config only.\n"
      "The Adapter and the probe are initialized and started now; the results shown are the "
      "Adapter's.\n"
      "%s",
      options.paramsFile.c_str(), params.adapterConfig.c_str(), static_cast<int>(flow.direction),
      sender ? "HCM to ROS" : "ROS to HCM", sender ? "sends" : "receives",
      ShortContractName(contract.typeName).c_str(), flow.topic.c_str(), flow.reliability.c_str(),
      flow.durability.c_str(), flow.depth, flow.globalBufferId, params.rate,
      params.rate == 0.0 ? " (unlimited)" : "", sender ? "payload" : "buffer",
      static_cast<unsigned long long>(FrameBytes(params, sender)), params.duration,
      params.duration == 0.0 ? " (until quit or a signal)" : "", params.warmup,
      params.log.empty() ? "none" : params.log.c_str(),
      commands ? kCommandHelp :
                 "Type a command below, help for what each does; the panel refreshes every "
                 "second.");
}

// --- Frame -------------------------------------------------------------------------

/**
 * @brief The host frame both nodes process: one Contract descriptor, at the
 * slot both are configured with, backed by a system-heap DMA-BUF for a
 * DMA-BUF Contract.
 */
class HostFrame final : public QC::QCFrameDescriptorNodeIfs
{
public:
  HostFrame(const HostContract & contract, std::size_t dmaBytes, std::string & error)
  {
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> created;
    if (adapter::CreateQrbBufferDescriptor(
            adapter::GetQrbBufferDescriptorType(contract.typeName), created) != QC::QC_STATUS_OK ||
        created == nullptr) {
      error = std::string("cannot create the descriptor of ") + contract.typeName;
      return;
    }
    if (contract.dmaBuf) {
      auto * buffer = dynamic_cast<QC::Memory::BufferDescriptor_t *>(created.get());
      if (buffer == nullptr) {
        error = std::string("the descriptor of ") + contract.typeName + " has no buffer";
        return;
      }
      fd_ = AllocateDmaBuf(dmaBytes, error);
      if (fd_ < 0)
        return;
      mapping_ = std::make_unique<DmaMapping>(fd_, dmaBytes, true);
      if (!mapping_->Valid()) {
        error = std::string("cannot map the DMA-BUF: ") + std::strerror(errno);
        return;
      }
      buffer->pBuf = mapping_->Data();
      buffer->size = dmaBytes;
      buffer->offset = 0;
      buffer->validSize = 0;
      buffer->allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;
      buffer->dmaHandle = static_cast<std::uint64_t>(fd_);
      buffer->pid = static_cast<std::uint64_t>(::getpid());
    }
    descriptor_ = std::move(created);
  }

  // QCFrameDescriptorNodeIfs has no virtual destructor on every QCNode SDK.
  ~HostFrame()
  {
    descriptor_.reset();
    mapping_.reset();
    if (fd_ >= 0)
      ::close(fd_);
  }

  HostFrame(const HostFrame &) = delete;
  HostFrame & operator=(const HostFrame &) = delete;

  bool Valid() const { return descriptor_ != nullptr; }
  void SetSlot(std::uint32_t slot) { slot_ = slot; }

  QC::Memory::QCBufferDescriptorBase_t & GetBuffer(std::uint32_t globalBufferId) override
  {
    return globalBufferId == slot_ && descriptor_ != nullptr ? *descriptor_ : empty_;
  }
  QC::QCStatus_e SetBuffer(std::uint32_t, QC::Memory::QCBufferDescriptorBase_t &) override
  {
    return QC::QC_STATUS_UNSUPPORTED;
  }
  void Clear() override {}
  QC::QCFrameDescriptorNodeIfs & operator=(QC::QCFrameDescriptorNodeIfs &) override
  {
    return *this;
  }

private:
  static int AllocateDmaBuf(std::size_t bytes, std::string & error)
  {
    const int heap = ::open(kDmaHeap, O_RDONLY | O_CLOEXEC);
    if (heap < 0) {
      error = std::string("cannot open ") + kDmaHeap + ": " + std::strerror(errno);
      return -1;
    }
    dma_heap_allocation_data allocation{};
    allocation.len = bytes;
    allocation.fd_flags = O_RDWR | O_CLOEXEC;
    const int result = ::ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &allocation);
    const int savedErrno = errno;
    ::close(heap);
    if (result != 0) {
      error = "cannot allocate " + std::to_string(bytes) + " bytes from " + kDmaHeap + ": " +
              std::strerror(savedErrno);
      return -1;
    }
    return static_cast<int>(allocation.fd);
  }

  int fd_{-1};
  std::unique_ptr<DmaMapping> mapping_;
  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor_;
  QC::Memory::QCBufferDescriptorBase_t empty_;
  std::uint32_t slot_{0};
};

// --- Monitoring helpers ------------------------------------------------------------

template <typename DataT>
bool PlaceMonitoring(QC::QCNodeIfs & node, DataT & out)
{
  std::uint32_t size = sizeof(DataT);
  return node.GetMonitoringIfs().Place(&out, size) == QC::QC_STATUS_OK;
}

/** @brief All the Adapter's monitoring fields, under their own names and in their order. */
std::string AdapterText(const adapter::HcmIpcAdapterMonitoringData & data)
{
  return FormatText(
      "sessionConnected=%u | frameCount=%llu | errorCount=%llu | lastLatencyNs=%llu | "
      "averageLatencyNs=%llu | minLatencyNs=%llu | maxLatencyNs=%llu | noDataCount=%llu",
      data.sessionConnected, U64(data.frameCount), U64(data.errorCount), U64(data.lastLatencyNs),
      U64(data.averageLatencyNs), U64(data.minLatencyNs), U64(data.maxLatencyNs),
      U64(data.noDataCount));
}

std::uint64_t Delta(std::uint64_t now, std::uint64_t earlier)
{
  return now >= earlier ? now - earlier : now;
}

/** @brief The stream counters of the probe. */
std::string ProbeText(const HcmStreamProbeMonitoringData & data)
{
  std::string text = data.sender != 0 ? SenderText(data.sent) : ReceiverText(data.received);
  if (data.processErrors != 0)
    text += FormatText("  process_errors %llu", U64(data.processErrors));
  return text;
}

std::uint64_t Problems(const HcmStreamProbeMonitoringData & data)
{
  return data.received.verifyErrors + data.received.foreignStream;
}

// --- Host --------------------------------------------------------------------------

std::atomic<bool> gSignalled{false};

extern "C" void OnSignal(int)
{
  gSignalled.store(true);
}

class StreamProbeHost
{
public:
  /** @param ui the terminal screen output goes to while it is active, or null */
  StreamProbeHost(const ProbeParams & params,
      const ProbeFlow & flow,
      const HostContract & contract,
      std::string adapterConfig,
      TerminalUi * ui)
    : params_(params)
    , startupAdapterConfig_(std::move(adapterConfig))
    , ui_(ui)
    , flow_(flow)
    , contract_(&contract)
    , sender_(flow.direction == ProbeDirection::kHcmToRos)
    , lastTickNs_(MonotonicNowNs())
    , lastLogNs_(lastTickNs_)
  {
  }

  ~StreamProbeHost() { StopLoop(); }

  /** @brief Create the nodes and the frame. @return empty, or why not */
  std::string Setup()
  {
    if (!params_.log.empty()) {
      log_.open(params_.log, std::ios::out | std::ios::trunc);
      if (!log_)
        return "cannot open log file " + params_.log;
      Write(
          "# qrb_ros_adapter_stream_probe, HCM side. Columns: CLOCK_MONOTONIC seconds (the clock "
          "of the\n"
          "# ROS side's log too), line type, details. Latencies in ms; e2e: probe send to "
          "receive.\n"
          "# A frame line per frame; on this side also the Adapter's call: status and latency.\n"
          "# A monitor line every 5 s: rates over those 5 s, totals since the probe's "
          "Initialize(),\n"
          "# then the Adapter's monitoring fields as they are (latencies in ns).\n" +
          FormatText("# run: rate %g fps, payload %llu bytes, buffer %llu bytes, duration %g s, "
                     "warmup %g s",
              params_.rate, U64(params_.payloadBytes), U64(params_.bufferBytes), params_.duration,
              params_.warmup));
    }
    if (QC::Node::QCNodeFactory::CreateNode(QC_NODE_TYPE_IPC_ADAPTER, adapter_) !=
            QC::QC_STATUS_OK ||
        CreateHcmStreamProbeNode(kHcmStreamProbeNodeType, probe_) != QC::QC_STATUS_OK)
      return "cannot create the nodes";
    std::string error;
    frameBytes_ = FrameBytes(params_, sender_.load());
    frame_ = std::make_unique<HostFrame>(*contract_, static_cast<std::size_t>(frameBytes_), error);
    if (!frame_->Valid())
      return error;
    frame_->SetSlot(flow_.globalBufferId);
    loop_ = std::thread([this]() { FrameLoop(); });
    return {};
  }

  /** @brief Initialize() and Start() both nodes. @return whether all four calls succeeded */
  bool AutoStart()
  {
    const auto lock = LockLifecycle();
    const bool initialized = InitLocked(startupAdapterConfig_, "init");
    const bool started = StartLocked("start");
    return initialized && started;
  }

  /** @brief Run one command line. @return false for quit */
  bool Command(std::string line)
  {
    std::istringstream words(line);
    std::string verb;
    words >> verb;
    std::string argument;
    std::getline(words >> std::ws, argument);
    if (verb.empty())
      return true;
    // A number stands for its command; the log has the name.
    if (verb.size() == 1 && verb[0] >= '1' &&
        verb[0] < '1' + static_cast<int>(std::size(kCommandNames))) {
      verb = kCommandNames[verb[0] - '1'];
      line = argument.empty() ? verb : verb + " " + argument;
    }

    if (verb == "quit" || verb == "exit")
      return false;
    if (verb == "help") {
      Print(kCommandHelp);
      return true;
    }
    const auto lock = LockLifecycle();
    if (verb == "init") {
      std::string config = startupAdapterConfig_;
      if (!argument.empty() && !ReadTextFile(argument, config)) {
        Print("init: cannot read " + argument);
        return true;
      }
      InitLocked(config, line);
    } else if (verb == "start")
      StartLocked(line);
    else if (verb == "stop")
      StopLocked(line);
    else if (verb == "deinit")
      DeinitLocked(line);
    else if (verb == "get_state")
      PrintNode("adapter", StateName(adapter_->GetState()));
    else if (verb == "get_config")
      PrintConfigLocked();
    else if (verb == "get_monitor")
      PrintMonitorLocked();
    else
      Print("unknown command '" + verb + "'; try help");
    return true;
  }

  /** @brief The frame loop reached `duration`. */
  bool Expired() const { return expired_.load(); }

  /**
   * @brief Once a second: report new verification problems and redraw the
   * panel; every kLogMonitorSeconds, write a monitor line.
   */
  void Tick()
  {
    HcmStreamProbeMonitoringData probe{};
    adapter::HcmIpcAdapterMonitoringData adapter{};
    std::string adapterState;
    std::uint64_t probeFailures = 0;
    {
      const auto lock = LockLifecycle();
      PlaceMonitoring(*probe_, probe);
      PlaceMonitoring(*adapter_, adapter);
      adapterState = StateName(adapter_->GetState());
      probeFailures = probeFailures_;
    }
    ReportProblems(probe);

    const std::uint64_t now = MonotonicNowNs();
    const auto perSecond = [](std::uint64_t count, std::uint64_t sinceNs, std::uint64_t nowNs) {
      const double interval = NsToSeconds(nowNs - sinceNs);
      return interval > 0.0 ? static_cast<double>(count) / interval : 0.0;
    };
    const auto frames = [](const HcmStreamProbeMonitoringData & data) {
      return data.sender != 0 ? data.sent.sent : data.received.received;
    };
    const auto bytes = [](const HcmStreamProbeMonitoringData & data) {
      return data.sender != 0 ? data.sent.bytes : data.received.bytes;
    };
    panelFps_ = perSecond(Delta(frames(probe), frames(lastProbe_)), lastTickNs_, now);
    panelBytesPerSecond_ = perSecond(Delta(bytes(probe), bytes(lastProbe_)), lastTickNs_, now);
    if (probe.sender == 0)
      panelLatency_ = probe.latency.Since(lastProbe_.latency);
    lastTickNs_ = now;
    lastProbe_ = probe;

    if (log_.is_open() &&
        now - lastLogNs_ >= kLogMonitorSeconds * kNsPerSecond - kNsPerSecond / 2) {
      // The frames are in the frame lines; this line has the rates and the totals.
      std::string text =
          FormatText("fps %.1f  ",
              perSecond(Delta(frames(probe), frames(lastLogProbe_)), lastLogNs_, now)) +
          ProbeText(probe);
      if (probe.sender == 0)
        text += "  " + LatencyText(probe.latency.Since(lastLogProbe_.latency));
      if (probeFailures != 0)
        text += FormatText("  probe_failed_calls %llu", U64(probeFailures));
      text += " | adapter " + adapterState + " | " + AdapterText(adapter);
      WriteLine("monitor", text, now);
      lastLogNs_ = now;
      lastLogProbe_ = probe;
    }
    ShowPanel();
  }

  /** @brief Redraw the status panel of the terminal screen: totals now, rates of the last Tick().
   */
  void ShowPanel()
  {
    if (ui_ == nullptr || !ui_->Active())
      return;
    std::vector<std::string> lines;
    {
      const auto lock = LockLifecycle();
      lines = PanelLocked();
    }
    ui_->SetPanel(std::move(lines));
  }

  /**
   * @brief Stop the frame loop, Stop() and DeInitialize() what is still up,
   * and print the summary. @return the exit code
   */
  int Finish(const std::string & reason, bool startFailed)
  {
    StopLoop();
    bool teardownFailed = false;
    {
      const auto lock = LockLifecycle();
      if (adapter_->GetState() == QC::QC_OBJECT_STATE_RUNNING ||
          probe_->GetState() == QC::QC_OBJECT_STATE_RUNNING)
        teardownFailed |= !StopLocked("exit stop");
      if (adapter_->GetState() != QC::QC_OBJECT_STATE_INITIAL ||
          probe_->GetState() != QC::QC_OBJECT_STATE_INITIAL)
        teardownFailed |= !DeinitLocked("exit deinit");
    }

    HcmStreamProbeMonitoringData probe{};
    adapter::HcmIpcAdapterMonitoringData adapter{};
    PlaceMonitoring(*probe_, probe);
    PlaceMonitoring(*adapter_, adapter);
    ReportProblems(probe);

    int exitCode = kExitOk;
    if (earlierProblems_ + Problems(probe) > 0)
      exitCode = kExitVerify;
    if (startFailed || teardownFailed)
      exitCode = kExitRuntime;

    const std::uint64_t now = MonotonicNowNs();
    std::vector<std::string> lines;
    lines.push_back(FormatText("end: %s, %s, exit code %d", reason.c_str(),
        startNs_ != 0 ?
            FormatText("%.1f s since the first frame", NsToSeconds(now - startNs_)).c_str() :
            "no frame",
        exitCode));
    std::string stream = ProbeText(probe);
    if (earlierProblems_ != 0)
      stream += FormatText("  problems before the last init %llu", U64(earlierProblems_));
    lines.push_back("stream: " + stream);
    if (probe.sender == 0)
      lines.push_back(LatencySummaryText(probe.latency));
    lines.push_back("adapter: " + AdapterText(adapter));
    if (adapterFailures_ != 0 || probeFailures_ != 0)
      lines.push_back(FormatText("failed ProcessFrameDescriptor calls: adapter %llu, probe %llu",
          U64(adapterFailures_), U64(probeFailures_)));
    std::string logged;
    for (const std::string & line : lines) {
      logged += (logged.empty() ? "" : "\n") + LogPrefix(now, "summary") + line;
      Print("summary: " + line);
    }
    Write(logged);
    return exitCode;
  }

private:
  // --- lifecycle, with the lifecycle mutex held ---------------------------------

  std::unique_lock<std::mutex> LockLifecycle()
  {
    // The frame loop takes the mutex once a frame; let the main thread in first.
    commandPending_.store(true);
    std::unique_lock<std::mutex> lock(lifecycleMutex_);
    commandPending_.store(false);
    return lock;
  }

  struct CallRecord
  {
    const char * node;
    const char * call;
    QC::QCStatus_e status;
    QC::QCObjectState_e before;
    QC::QCObjectState_e after;
  };

  static std::string CallText(const CallRecord & record)
  {
    return FormatText("%s %s %s %s->%s", record.node, record.call,
        StatusName(record.status).c_str(), StateName(record.before).c_str(),
        StateName(record.after).c_str());
  }

  /** @brief Call `invoke` on `node`; only the Adapter's calls are printed. */
  template <typename CallT>
  CallRecord Call(const char * nodeName, QC::QCNodeIfs & node, const char * call, CallT && invoke)
  {
    CallRecord record{nodeName, call, QC::QC_STATUS_OK, node.GetState(), node.GetState()};
    record.status = invoke(node);
    record.after = node.GetState();
    if (&node == adapter_.get())
      Print(CallText(record));
    return record;
  }

  /**
   * @brief Log the Adapter's call of `command`, and the probe's if it failed,
   * and take the Adapter's state as known. @return whether all calls succeeded
   */
  bool Record(const std::string & command, const std::vector<CallRecord> & records)
  {
    bool ok = true;
    std::string text = command + ":";
    for (const CallRecord & record : records) {
      ok &= record.status == QC::QC_STATUS_OK;
      if (record.status == QC::QC_STATUS_OK && std::strcmp(record.node, "adapter") != 0)
        continue;
      text += " " + CallText(record) + ";";
    }
    text.pop_back();
    WriteLine("command", text);
    adapterState_ = adapter_->GetState();
    return ok;
  }

  bool InitLocked(const std::string & adapterConfig, const std::string & command)
  {
    // The probe follows the Flow of the Adapter's config. The Adapter reports
    // what is wrong with the config, so a bad field keeps its current value.
    // A node that is not INITIAL refuses Initialize(); the Flow stays as is.
    if (adapter_->GetState() == QC::QC_OBJECT_STATE_INITIAL &&
        probe_->GetState() == QC::QC_OBJECT_STATE_INITIAL) {
      // What the config leaves out takes the Adapter's defaults again.
      const ProbeFlow defaults;
      ProbeFlow base = flow_;
      base.reliability = defaults.reliability;
      base.durability = defaults.durability;
      base.depth = defaults.depth;
      base.globalBufferId = adapter::kHcmDefaultGlobalBufferId;
      std::vector<std::string> problems;
      ProbeFlow flow = ReadProbeFlow(adapterConfig, base, problems);
      const HostContract * contract = FindContract(flow.contract);
      if (contract == nullptr) {
        contract = contract_;
        flow.contract = flow_.contract;
      }
      // A new Contract, or a new direction of a DMA-BUF one, needs a new frame.
      const std::uint64_t frameBytes =
          FrameBytes(params_, flow.direction == ProbeDirection::kHcmToRos);
      if (contract != contract_ || (contract->dmaBuf && frameBytes != frameBytes_)) {
        std::string error;
        auto frame =
            std::make_unique<HostFrame>(*contract, static_cast<std::size_t>(frameBytes), error);
        if (!frame->Valid()) {
          Print("init: " + error);
          return false;
        }
        frame_ = std::move(frame);
        frameBytes_ = frameBytes;
        contract_ = contract;
      }
      flow_ = flow;
      sender_.store(flow_.direction == ProbeDirection::kHcmToRos);
      frame_->SetSlot(flow_.globalBufferId);
      WriteLine("flow", FlowText(flow_, true));
    }

    HcmStreamProbeMonitoringData before{};
    PlaceMonitoring(*probe_, before);

    std::vector<CallRecord> records;
    records.push_back(Call("adapter", *adapter_, "Initialize", [&](QC::QCNodeIfs & node) {
      QC::QCNodeInit_t init;
      init.config = adapterConfig;
      return node.Initialize(init);
    }));
    records.push_back(Call("probe", *probe_, "Initialize", [&](QC::QCNodeIfs & node) {
      QC::QCNodeInit_t init;
      init.config = ProbeConfig();
      return node.Initialize(init);
    }));
    // Initialize() resets the monitoring of a probe that accepts it.
    if (records[1].status == QC::QC_STATUS_OK) {
      earlierProblems_ += Problems(before);
      PlaceMonitoring(*probe_, lastProbe_);
      lastLogProbe_ = lastProbe_;
      reportedProblems_ = 0;
    }
    return Record(command, records);
  }

  bool StartLocked(const std::string & command)
  {
    std::vector<CallRecord> records;
    records.push_back(
        Call("adapter", *adapter_, "Start", [](QC::QCNodeIfs & node) { return node.Start(); }));
    records.push_back(
        Call("probe", *probe_, "Start", [](QC::QCNodeIfs & node) { return node.Start(); }));
    return Record(command, records);
  }

  bool StopLocked(const std::string & command)
  {
    std::vector<CallRecord> records;
    records.push_back(
        Call("adapter", *adapter_, "Stop", [](QC::QCNodeIfs & node) { return node.Stop(); }));
    records.push_back(
        Call("probe", *probe_, "Stop", [](QC::QCNodeIfs & node) { return node.Stop(); }));
    return Record(command, records);
  }

  bool DeinitLocked(const std::string & command)
  {
    std::vector<CallRecord> records;
    records.push_back(Call("adapter", *adapter_, "DeInitialize",
        [](QC::QCNodeIfs & node) { return node.DeInitialize(); }));
    records.push_back(Call("probe", *probe_, "DeInitialize",
        [](QC::QCNodeIfs & node) { return node.DeInitialize(); }));
    return Record(command, records);
  }

  std::string ProbeConfig() const
  {
    return ProbeNodeConfig(params_, flow_, contract_->typeName, flow_.globalBufferId);
  }

  void PrintConfigLocked()
  {
    QC::QCNodeConfigIfs & adapterIfs = adapter_->GetConfigurationIfs();
    const QC::QCNodeConfigBase_t & adapterBase = adapterIfs.Get();
    nlohmann::json out{{"node_id",
        {{"name", adapterBase.nodeId.name}, {"type", static_cast<int>(adapterBase.nodeId.type)},
            {"id", adapterBase.nodeId.id}}}};
    if (auto * config = dynamic_cast<adapter::HcmIpcAdapterNodeConfig *>(&adapterIfs)) {
      const adapter::HcmIpcAdapterConfig & a = config->Adapter();
      // The static fields; name and id are in node_id, the timeouts are fixed.
      out["adapter"] = nlohmann::json{{"contract_type", a.flowConfig.contractTypeName},
          {"direction", ConfigDirection(a.flowConfig.direction)},
          {"ros_topic", a.flowConfig.rosTopic},
          {"qos_profile", {{"reliability", adapter::ToString(a.flowConfig.qosProfile.reliability)},
                              {"durability", adapter::ToString(a.flowConfig.qosProfile.durability)},
                              {"depth", a.flowConfig.qosProfile.depth}}},
          {"global_buffer_id", a.globalBufferId}};
    }
    PrintNode("adapter", out.dump());
  }

  void PrintMonitorLocked()
  {
    adapter::HcmIpcAdapterMonitoringData adapter{};
    HcmStreamProbeMonitoringData probe{};
    const bool adapterOk = PlaceMonitoring(*adapter_, adapter);
    const bool probeOk = PlaceMonitoring(*probe_, probe);
    PrintNode("adapter", adapterOk ? AdapterText(adapter) : "Place() failed");
    std::string stream = "Place() failed";
    if (probeOk) {
      stream = std::string(probe.sender != 0 ? "this side sends: " : "this side receives: ") +
               ProbeText(probe);
      if (probe.sender == 0)
        stream += "  " + LatencyText(probe.latency);
    }
    PrintNode("stream", stream);
  }

  // --- frame loop ------------------------------------------------------------------

  void FrameLoop()
  {
    using Clock = std::chrono::steady_clock;
    const auto period = params_.rate > 0.0 ?
                            std::chrono::nanoseconds(static_cast<std::int64_t>(
                                static_cast<double>(kNsPerSecond) / params_.rate)) :
                            std::chrono::nanoseconds(0);
    Clock::time_point next{};
    bool pacing = false;

    while (!stopLoop_.load()) {
      if (commandPending_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      std::chrono::microseconds pause{0};
      bool paced = false;
      {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        const bool sender = sender_.load();
        paced = sender && params_.rate > 0.0;
        const bool running = adapter_->GetState() == QC::QC_OBJECT_STATE_RUNNING &&
                             probe_->GetState() == QC::QC_OBJECT_STATE_RUNNING;
        if (!running || expired_.load()) {
          pause = std::chrono::milliseconds(10);
          pacing = false;
        } else {
          if (sender && startNs_ == 0)
            startNs_ = MonotonicNowNs();
          if (startNs_ != 0 && params_.duration > 0.0 &&
              MonotonicNowNs() - startNs_ >= SecondsToNs(params_.duration)) {
            expired_.store(true);
            continue;
          }
          const bool ok = sender ? SendFrameLocked() : ReceiveFrameLocked(pause);
          if (!ok && pause.count() == 0)
            pause = std::chrono::microseconds(paced ? 0 : 100);
        }
        ReportStateChangesLocked();
      }
      if (paced && pause == std::chrono::microseconds(0)) {
        const Clock::time_point now = Clock::now();
        if (!pacing || now > next + period) {
          next = now;
          pacing = true;
        }
        next += period;
        std::this_thread::sleep_until(next);
      } else if (pause.count() > 0)
        std::this_thread::sleep_for(pause);
    }
  }

  bool SendFrameLocked()
  {
    const QC::QCStatus_e probeStatus = probe_->ProcessFrameDescriptor(*frame_);
    if (probeStatus != QC::QC_STATUS_OK) {
      Failure("probe", probeFailures_, probeStatus);
      return false;
    }
    const QC::QCStatus_e adapterStatus = adapter_->ProcessFrameDescriptor(*frame_);
    // Logged whatever the Adapter did, so the log shows where a frame stopped.
    if (log_.is_open())
      LogFrameLocked(adapterStatus);
    if (adapterStatus != QC::QC_STATUS_OK) {
      Failure("adapter", adapterFailures_, adapterStatus);
      return false;
    }
    return true;
  }

  /** @brief The frame line of the frame the probe just sent or received. */
  void LogFrameLocked(QC::QCStatus_e adapterStatus,
      const adapter::HcmIpcAdapterMonitoringData * adapter = nullptr)
  {
    // ~30 KB a frame; only with --log.
    HcmStreamProbeMonitoringData probe{};
    if (!PlaceMonitoring(*probe_, probe))
      return;
    std::string extra = "  adapter " + StatusName(adapterStatus);
    adapter::HcmIpcAdapterMonitoringData placed{};
    if (adapter == nullptr && adapterStatus == QC::QC_STATUS_OK &&
        PlaceMonitoring(*adapter_, placed))
      adapter = &placed;
    if (adapter != nullptr)
      extra += FormatText(" %.3f ms", NsToMs(adapter->lastLatencyNs));
    WriteFrame(FrameLogLine(probe.lastFrame, extra));
  }

  bool ReceiveFrameLocked(std::chrono::microseconds & pause)
  {
    adapter::HcmIpcAdapterMonitoringData before{};
    PlaceMonitoring(*adapter_, before);
    const QC::QCStatus_e adapterStatus = adapter_->ProcessFrameDescriptor(*frame_);
    if (adapterStatus != QC::QC_STATUS_OK) {
      Failure("adapter", adapterFailures_, adapterStatus);
      pause = std::chrono::milliseconds(1);
      return false;
    }
    adapter::HcmIpcAdapterMonitoringData after{};
    PlaceMonitoring(*adapter_, after);
    if (after.noDataCount != before.noDataCount) {
      pause = std::chrono::microseconds(200);  // no fresh sample: the frame has no new data
      return true;
    }
    if (startNs_ == 0)
      startNs_ = MonotonicNowNs();
    const QC::QCStatus_e probeStatus = probe_->ProcessFrameDescriptor(*frame_);
    if (probeStatus != QC::QC_STATUS_OK) {
      Failure("probe", probeFailures_, probeStatus);
      return false;
    }
    if (log_.is_open())
      LogFrameLocked(adapterStatus, &after);
    return true;
  }

  void Failure(const char * node, std::uint64_t & count, QC::QCStatus_e status)
  {
    ++count;
    if (count > kLoggedProblems)
      return;
    const std::string detail = FormatText("%s ProcessFrameDescriptor %s (failure %llu)%s", node,
        StatusName(status).c_str(), static_cast<unsigned long long>(count),
        count == kLoggedProblems ? "; further failures are only counted" : "");
    Event(detail);
  }

  /** @brief A change of the Adapter's state outside a command. */
  void ReportStateChangesLocked()
  {
    const QC::QCObjectState_e adapterNow = adapter_->GetState();
    if (adapterNow != adapterState_)
      Event("adapter " + StateName(adapterState_) + "->" + StateName(adapterNow));
    adapterState_ = adapterNow;
  }

  void ReportProblems(const HcmStreamProbeMonitoringData & probe)
  {
    const std::uint64_t problems = Problems(probe);
    if (problems <= reportedProblems_)
      return;
    const std::uint64_t first = reportedProblems_;
    reportedProblems_ = problems;
    if (earlierProblems_ + first >= kLoggedProblems)
      return;
    const FrameReading & reading = probe.lastProblem;
    Event(FormatText("frame %llu: %s at offset %llu (stream tag 0x%08x); %llu problem(s) so far",
        static_cast<unsigned long long>(reading.header.sequence), FrameCheckName(reading.check),
        static_cast<unsigned long long>(reading.badOffset), reading.header.streamTag,
        static_cast<unsigned long long>(earlierProblems_ + problems)));
  }

  void StopLoop()
  {
    stopLoop_.store(true);
    if (loop_.joinable())
      loop_.join();
  }

  // --- output ----------------------------------------------------------------------

  /** @brief The status panel: a title, then TerminalUi::kPanelRows - 1 lines. */
  std::vector<std::string> PanelLocked()
  {
    HcmStreamProbeMonitoringData probe{};
    adapter::HcmIpcAdapterMonitoringData adapter{};
    PlaceMonitoring(*probe_, probe);
    PlaceMonitoring(*adapter_, adapter);
    const auto ms = [](std::uint64_t ns) { return static_cast<double>(ns) / 1e6; };
    const auto count = [](std::uint64_t value) { return static_cast<unsigned long long>(value); };
    const double mibPerSecond = panelBytesPerSecond_ / static_cast<double>(1u << 20);
    const bool sender = sender_.load();

    std::string elapsed = startNs_ != 0 ?
                              FormatText("%.1f s", NsToSeconds(MonotonicNowNs() - startNs_)) :
                              std::string("not started");
    if (params_.duration > 0.0)
      elapsed += FormatText(" of %g s", params_.duration);
    std::vector<std::string> lines;
    lines.push_back(FormatText("%s  %s %s %s", elapsed.c_str(), sender ? "H2R" : "R2H",
        ShortContractName(contract_->typeName).c_str(), flow_.topic.c_str()));
    lines.push_back(FormatText("adapter  %-8s session %u  failed calls %llu",
        StateName(adapter_->GetState()).c_str(), adapter.sessionConnected,
        count(adapterFailures_)));
    lines.push_back(FormatText("         frames %llu  errors %llu  no_data %llu",
        count(adapter.frameCount), count(adapter.errorCount), count(adapter.noDataCount)));
    // The panel keeps its height, so a placeholder until the first frame.
    lines.push_back(adapter.maxLatencyNs == 0 ?
                        std::string("         latency -") :
                        FormatText("         latency last %.3f  avg %.3f  min %.3f  max %.3f ms",
                            ms(adapter.lastLatencyNs), ms(adapter.averageLatencyNs),
                            ms(adapter.minLatencyNs), ms(adapter.maxLatencyNs)));
    const std::string probeFailed = probeFailures_ != 0 ?
                                        FormatText("  failed calls %llu", count(probeFailures_)) :
                                        std::string();
    if (sender) {
      lines.push_back(FormatText("probe    sent %.1f fps  %.1f MiB/s  total %llu", panelFps_,
          mibPerSecond, count(probe.sent.sent)));
      lines.push_back(FormatText(
          "         send_errors %llu%s", count(probe.sent.sendErrors), probeFailed.c_str()));
    } else {
      const ReceiverCounters & received = probe.received;
      lines.push_back(
          FormatText("probe    recv %.1f fps  %.1f MiB/s  total %llu  lost %llu  dup %llu  reorder "
                     "%llu",
              panelFps_, mibPerSecond, count(received.received), count(received.lost),
              count(received.duplicates), count(received.reordered)));
      std::string latency = "e2e(1s) -";
      if (panelLatency_.Count() > 0)
        latency = FormatText("e2e(1s) p50 %.3f  p99 %.3f  max %.3f ms",
            ms(panelLatency_.Percentile(0.50)), ms(panelLatency_.Percentile(0.99)),
            ms(panelLatency_.Percentile(1.0)));
      lines.push_back(
          FormatText("         foreign %llu  verify %llu  %s%s", count(received.foreignStream),
              count(received.verifyErrors), latency.c_str(), probeFailed.c_str()));
    }
    return lines;
  }

  void Print(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (ui_ != nullptr && ui_->Print(text))
      return;
    std::fputs(text.c_str(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
  }

  void PrintNode(const char * node, const std::string & text)
  {
    Print(FormatText("%-7s ", node) + text);
  }

  void Event(const std::string & detail)
  {
    Print("event: " + detail);
    WriteLine("event", detail);
  }

  /** @brief A --log line of `type`; the time is CLOCK_MONOTONIC, as in the frame lines of both
   * sides. */
  void WriteLine(const char * type,
      const std::string & text,
      std::uint64_t nowNs = MonotonicNowNs())
  {
    Write(LogPrefix(nowNs, type) + text);
  }

  /** @brief `text`, one or more lines, to the --log file if there is one. */
  void Write(const std::string & text)
  {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (!log_.is_open())
      return;
    log_ << text << '\n';
    log_.flush();
  }

  /** Frame lines are flushed with the next other line, at the latest the monitor line. */
  void WriteFrame(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(outputMutex_);
    if (log_.is_open())
      log_ << line << '\n';
  }

  const ProbeParams params_;
  const std::string startupAdapterConfig_;
  TerminalUi * const ui_;
  // Changed by Initialize() only, with the lifecycle mutex held.
  ProbeFlow flow_;
  const HostContract * contract_;
  std::atomic<bool> sender_;

  std::unique_ptr<QC::QCNodeIfs> adapter_;
  std::unique_ptr<QC::QCNodeIfs> probe_;
  std::unique_ptr<HostFrame> frame_;
  std::uint64_t frameBytes_{0};

  std::mutex lifecycleMutex_;  // node calls
  std::atomic<bool> commandPending_{false};
  std::atomic<bool> stopLoop_{false};
  std::atomic<bool> expired_{false};
  std::thread loop_;
  // With the lifecycle mutex held.
  std::uint64_t startNs_{0};
  QC::QCObjectState_e adapterState_{QC::QC_OBJECT_STATE_INITIAL};
  std::uint64_t adapterFailures_{0};
  std::uint64_t probeFailures_{0};
  std::uint64_t earlierProblems_{0};  // of the probe before its latest Initialize()
  // Main thread only.
  std::uint64_t reportedProblems_{0};
  std::uint64_t lastTickNs_;
  std::uint64_t lastLogNs_;  // of the last monitor line
  HcmStreamProbeMonitoringData lastProbe_{};
  HcmStreamProbeMonitoringData lastLogProbe_{};
  // Main thread only: rates of the last Tick() for the panel.
  double panelFps_{0.0};
  double panelBytesPerSecond_{0.0};
  LatencyHistogram panelLatency_;

  std::mutex outputMutex_;
  std::ofstream log_;
};

/** @brief Lines read from stdin without blocking for longer than `timeoutMs`. */
class LineReader
{
public:
  /** @return false once stdin is closed */
  bool Poll(int timeoutMs, std::vector<std::string> & lines)
  {
    if (closed_)
      return false;
    pollfd fd{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&fd, 1, timeoutMs);
    if (ready <= 0)
      return true;  // timeout or a signal
    char buffer[4096];
    const ssize_t got = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (got < 0 && errno == EINTR)
      return true;
    if (got <= 0) {
      if (!pending_.empty())
        lines.push_back(pending_);
      pending_.clear();
      closed_ = true;
      return false;
    }
    pending_.append(buffer, static_cast<std::size_t>(got));
    std::size_t newline = 0;
    while ((newline = pending_.find('\n')) != std::string::npos) {
      lines.push_back(pending_.substr(0, newline));
      pending_.erase(0, newline + 1);
    }
    return true;
  }

private:
  std::string pending_;
  bool closed_{false};
};

int Run(int argc, char ** argv)
{
  HostOptions options;
  try {
    options = ParseCommandLine(argc, argv);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "qrb_ros_adapter_stream_probe: %s\n", error.what());
    return kExitParams;
  }
  if (options.help) {
    PrintHelp();
    return kExitOk;
  }
  const ProbeParams & params = options.params;

  std::string adapterConfig;
  if (!ReadTextFile(params.adapterConfig, adapterConfig)) {
    std::fprintf(
        stderr, "qrb_ros_adapter_stream_probe: cannot read %s\n", params.adapterConfig.c_str());
    return kExitParams;
  }
  // A bad field is left to the Adapter to report; the probe keeps its default.
  std::vector<std::string> problems;
  ProbeFlow flow = ReadProbeFlow(adapterConfig, ProbeFlow{}, problems);
  const HostContract * contract = FindContract(flow.contract);
  if (contract == nullptr) {
    std::fprintf(stderr,
        "qrb_ros_adapter_stream_probe: %s: unsupported contract_type %s; supported: %s\n",
        params.adapterConfig.c_str(), flow.contract.c_str(), SupportedContracts().c_str());
    return kExitParams;
  }
  const bool sender = flow.direction == ProbeDirection::kHcmToRos;
  if (contract->dmaBuf && FrameBytes(params, sender) == 0) {
    std::fprintf(stderr, "qrb_ros_adapter_stream_probe: '%s' must be > 0 for %s\n",
        sender ? "payload_bytes" : "buffer_bytes", contract->typeName);
    return kExitParams;
  }

  // What the probe would refuse is a parameter error, before anything runs.
  {
    std::unique_ptr<QC::QCNodeIfs> check;
    CreateHcmStreamProbeNode(kHcmStreamProbeNodeType, check);
    std::string errors;
    if (check == nullptr ||
        check->GetConfigurationIfs().VerifyAndSet(
            ProbeNodeConfig(params, flow, contract->typeName, flow.globalBufferId), errors) !=
            QC::QC_STATUS_OK) {
      std::fprintf(stderr, "qrb_ros_adapter_stream_probe: %s\n", errors.c_str());
      return kExitParams;
    }
  }

  struct sigaction action{};
  action.sa_handler = OnSignal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  std::signal(SIGPIPE, SIG_IGN);

  // A terminal of enough rows gets the screen; anything else, plain lines.
  TerminalUi ui;
  const bool screen = ui.Open(kCommandBar);
  const std::string startupHelp = StartupHelp(options, flow, *contract, !screen);
  if (!ui.Print(startupHelp))
    std::fprintf(stderr, "%s\n", startupHelp.c_str());
  StreamProbeHost host(params, flow, *contract, adapterConfig, screen ? &ui : nullptr);
  const std::string error = host.Setup();
  if (!error.empty()) {
    ui.Close();
    std::fprintf(stderr, "qrb_ros_adapter_stream_probe: %s\n", error.c_str());
    return kExitRuntime;
  }

  const bool interactive = ::isatty(STDIN_FILENO) != 0;
  const bool started = host.AutoStart();
  host.ShowPanel();
  LineReader reader;
  bool stdinOpen = true;
  std::string reason;
  auto lastTick = std::chrono::steady_clock::now();
  const auto prompt = [interactive, screen]() {
    if (!interactive || screen)
      return;
    std::fputs("> ", stdout);
    std::fflush(stdout);
  };
  prompt();

  while (reason.empty()) {
    std::vector<std::string> lines;
    if (stdinOpen)
      stdinOpen = screen ? ui.Poll(100, lines) : reader.Poll(100, lines);
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (const std::string & line : lines) {
      if (!host.Command(line)) {
        reason = "quit";
        break;
      }
      prompt();
    }
    if (!reason.empty())
      break;
    if (!lines.empty())
      host.ShowPanel();
    if (gSignalled.load())
      reason = "signal";
    else if (host.Expired())
      reason = "duration";
    else if (!stdinOpen && !started)
      reason = "start_failed";

    const auto now = std::chrono::steady_clock::now();
    if (now - lastTick >= std::chrono::seconds(1)) {
      lastTick = now;
      host.Tick();
    }
  }
  if (screen) {
    // The last figures stay on the screen; the teardown and the summary follow below them.
    host.ShowPanel();
    ui.Close();
  } else if (interactive)
    std::fputc('\n', stdout);
  return host.Finish(reason, reason == "start_failed");
}

}  // namespace

}  // namespace qrb::adapter::stream_probe

int main(int argc, char ** argv)
{
  return qrb::adapter::stream_probe::Run(argc, argv);
}
