<div align="center">
  <h1>QRB ROS Adapter</h1>
  <p>HCM and ROS 2 Bridge for Qualcomm Robotics Platforms</p>

  <img src="https://img.shields.io/badge/Qualcomm%20Linux-3253DC?style=for-the-badge&logo=qualcomm&logoColor=white" alt="Qualcomm Linux">
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
</div>

---

## Overview

QRB ROS Adapter bridges Contract messages between HCM and ROS 2 over Unix
Domain Sockets. The HCM IPC Adapter is a standard synchronous QCNode; the ROS
IPC Bridge creates one ROS Flow Bridge for each Flow an Adapter registers.

- `ipc_runtime`: IPC definitions, transport and Contract validation over the
  `qrb_contracts_proto` messages.
- `hcm_ipc_adapter`: the IPC Adapter Node. Needs QCNode, not ROS.
- `ros_ipc_bridge`: the Bridge Manager and ROS Flow Bridge. Needs ROS, not
  QCNode.
- `test`: the end-to-end test program.

## Table of contents

- [APIs](#apis)
- [Supported targets](#supported-targets)
- [Installation](#installation)
- [Usage](#usage)
- [Build from source](#build-from-source)
- [Test](#test)
- [License](#license)

## APIs

The IPC Adapter Node is built into `libQCNode.so` by the qcnode build. An HCM
application, or a graph framework hosting QCNodes, creates one node per Flow
with `QCNodeFactory::CreateNode(QC_NODE_TYPE_IPC_ADAPTER, ...)` and drives it
through `Initialize()`, `Start()`, `ProcessFrameDescriptor()`, `Stop()` and
`DeInitialize()`. The host owns the frame descriptors and their buffers.

The Flow is set in the QCNode static config:

| Field | Required | Description |
| --- | --- | --- |
| `contract_type` | yes | Contract full name, e.g. `qrb.contract.sensor.v1.PointCloud2`. |
| `direction` | yes | `0`: HCM to ROS (the node publishes the descriptor); `1`: ROS to HCM (the node fills it in). |
| `ros_topic` | yes | ROS topic. |
| `qos_profile` | no | `reliability` (`reliable`/`best_effort`), `durability` (`volatile`/`transient_local`), `depth` (1–32); defaults to `rclcpp::QoS(10)`. |

Monitoring reports `sessionConnected`, `frameCount`, `errorCount`,
`last/average/min/maxLatencyNs` and `noDataCount`.

The ROS IPC Bridge takes no parameters. It runs the node
`/qrb_ros_adapter_bridge_manager` in the component container
`/qrb_ros_adapter_container`, and adds `/qrb_ros_adapter_flow_<name>` for each
registered Flow. Opposite-direction Flows on the same Contract and Topic are
rejected.

## Supported targets

| Development hardware | Operating system | ROS |
| --- | --- | --- |
| Qualcomm IQ-10 Ride | Qualcomm Linux (IQ10.LE.2.0) | Jazzy |

## Installation

Installation packages and setup instructions are not ready yet.

## Usage

Start the ROS IPC Bridge:

```bash
/bin/bash
source /usr/ros/jazzy/setup.bash
export ROS_HOME=/data/ros
ros2 launch qrb_ros_adapter ros_ipc_bridge.launch.py
```

The ROS setup script needs bash, and the root filesystem of the image is
read-only, so `ROS_HOME` points ROS logs to a writable directory.

On the HCM side, the host creates one IPC Adapter Node per Flow through
`QCNodeFactory` and drives it like any other QCNode, following the
lifecycle and frame processing in
[QCNode](https://github.com/qualcomm/QcNode/blob/main/docs/QCNode.md).
`Initialize()` takes the static config above and registers the Flow with the
Bridge. The Contract descriptor, created with `CreateQrbBufferDescriptor()`, is
buffer 0 of the frame passed to `ProcessFrameDescriptor()`: direction 0
publishes it to the ROS topic, and direction 1 fills it with the newest ROS
message, or only counts `noDataCount` if there is none.

ROS nodes publish or subscribe the topic as usual.

`SampleIpcAdapter` in the QCNode sample application is such a host. With the
Bridge running, it publishes a 1 MiB PointCloud2 to
`/qrb_ros_adapter/sample/points` at 30 FPS:

```bash
qcrun QCNodeSampleApp -n adapter -t IpcAdapter
```

Its attributes, given as `-k <name> -v <value>`, such as the direction, the
Contract and the topic, are listed in
[SampleIpcAdapter.cpp](hcm_ipc_adapter/sample/SampleIpcAdapter.cpp). For
example, to receive the poses published on `/qrb_ros_adapter/sample/pose` for
8 seconds:

```bash
qcrun QCNodeSampleApp -n adapter -t IpcAdapter -T 8 \
  -k contract_type -v qrb.contract.geometry.v1.PoseWithCovarianceStamped -k direction -v 1
```

```text
Init adapter OK
Start adapter OK
adapter received 30 frames, 0 errors
...
adapter received 239 frames, 0 errors; adapter: frameCount=240 errorCount=0 noDataCount=1 averageLatencyNs=67594 maxLatencyNs=98229
Stop adapter OK
Deinit adapter OK
```

## Build from source

Source build instructions are not ready yet.

## Test

The steps below run on a Qualcomm Linux image that includes the packages and
ros2cli.

`qrb_ros_adapter_stream_probe` runs one Flow end to end, the two sides in
separate processes: the HCM side hosts the Adapter and a probe QCNode on one
frame, and the ROS side is the probe component
`qrb::adapter::stream_probe::RosStreamProbeNode`. Each frame carries a sequence
number and its send time, so the receiver counts lost, duplicate and reordered
frames and measures the end-to-end latency. `qrb_ros_adapter_stream_test.sh`
starts one side per terminal and writes the same Adapter config for both.

1. In one terminal, start the ROS IPC Bridge with the ROS probe in its
   container:

   ```bash
   /bin/bash
   source /usr/ros/jazzy/setup.bash
   qrb_ros_adapter_stream_test.sh ros
   ```

2. In another terminal, start the HCM side:

   ```bash
   /bin/bash
   source /usr/ros/jazzy/setup.bash
   qrb_ros_adapter_stream_test.sh hcm
   ```

   It initializes and starts both nodes and sends 1 MiB PointCloud2 frames from
   HCM to ROS at 30 FPS. Type `help` for the lifecycle commands, and `quit` to
   stop and print the summary. Ctrl-C in the first terminal stops the Bridge,
   and the ROS probe prints its summary there.

Give both sides the same options, such as the direction, the Contract, the
payload size, the rate and the QoS; `--help` lists them:

```bash
qrb_ros_adapter_stream_test.sh --help
```

A run, trimmed (the Bridge terminal, then the HCM side):

```text
$ qrb_ros_adapter_stream_test.sh ros
[INFO] [qrb_ros_adapter_bridge_manager]: listening for Adapters on '/run/qrb_ros_adapter/bootstrap.sock'
[INFO] [qrb_ros_adapter_bridge_manager]: component container: /qrb_ros_adapter_container
...
Loaded component 1 into '/qrb_ros_adapter_container' container node as '/qrb_ros_adapter_stream_probe'
Start the HCM side in another terminal: qrb_ros_adapter_stream_test.sh hcm, with the same options.
Ctrl-C stops the Bridge.
[INFO] [qrb_ros_adapter_bridge_manager]: registered flow 'stream_probe_adapter' at '/run/qrb_ros_adapter/flows/stream_probe_adapter-1.sock'
[INFO] [qrb_ros_adapter_stream_probe]: first frame: seq 0, check ok
[INFO] [qrb_ros_adapter_flow_stream_probe_adapter]: flow 'stream_probe_adapter': closed, frames in 180, out 180, no-data 0, conversion errors 0
[INFO] [qrb_ros_adapter_bridge_manager]: unregistered flow 'stream_probe_adapter'
^C
[INFO] [qrb_ros_adapter_stream_probe]: summary: stream: received 180  lost 0
[INFO] [qrb_ros_adapter_stream_probe]: summary: e2e latency of 180 frames: avg 0.187 min 0.134 p50 0.190 p95 0.203 p99 0.219 max 0.239 ms
```

```text
$ qrb_ros_adapter_stream_test.sh hcm
qrb_ros_adapter_stream_probe: the HCM side of a probed stream through the HCM IPC Adapter
  flow            direction 0 (HCM to ROS, this side sends), PointCloud2 on /qrb_ros_adapter/stream_probe/points
                  QoS best_effort, volatile, depth 5; descriptor at buffer 0
  run             rate 30 fps, payload 1048576 bytes, duration 0 s (until quit or a signal), warmup 0 s, log none
...
adapter Initialize OK INITIAL->READY
adapter Start OK READY->RUNNING
> quit
adapter Stop OK RUNNING->READY
adapter DeInitialize OK READY->INITIAL
summary: end: quit, 6.0 s since the first frame, exit code 0
summary: stream: sent 180
summary: adapter: sessionConnected=0 | frameCount=180 | errorCount=0 | lastLatencyNs=158385 | averageLatencyNs=184076 | minLatencyNs=116823 | maxLatencyNs=290416 | noDataCount=0
```

Each package also has unit tests, run with `ctest` in its build directory.

## License

This project is licensed under the BSD-3-Clause-Clear license.
