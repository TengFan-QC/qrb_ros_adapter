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

An HCM application, or a graph framework hosting QCNodes, links
`libqrb_ros_adapter_hcm_ipc_adapter.so`, creates one node per Flow with
`CreateHcmIpcAdapterNode()` and drives it through `Initialize()`, `Start()`,
`ProcessFrameDescriptor()`, `Stop()` and `DeInitialize()`. The host owns the
frame descriptors and their buffers.

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
source /usr/ros/jazzy/setup.bash
ros2 launch qrb_ros_adapter ros_ipc_bridge.launch.py
```

On the HCM side, the host creates one IPC Adapter Node per Flow with
`CreateHcmIpcAdapterNode()` and drives it like any other QCNode, following the
lifecycle and frame processing in
[QCNode](https://github.com/qualcomm/QcNode/blob/main/docs/QCNode.md).
`Initialize()` takes the static config above and registers the Flow with the
Bridge. The Contract descriptor, created with `CreateQrbBufferDescriptor()`, is
buffer 0 of the frame passed to `ProcessFrameDescriptor()`: direction 0
publishes it to the ROS topic, and direction 1 fills it with the newest ROS
message, or only counts `noDataCount` if there is none.

ROS nodes publish or subscribe the topic as usual.

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
frames and measures the end-to-end latency. Both sides take the same Adapter
config: `probe_configs/adapter_h2r.json` (default) or `adapter_r2h.json`, under
`/usr/ros/jazzy/share/qrb_ros_adapter_stream_probe/`.

1. Start the ROS IPC Bridge as in [Usage](#usage).

2. In another terminal, load the ROS probe into the Bridge container:

   ```bash
   source /usr/ros/jazzy/setup.bash
   ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe \
     qrb::adapter::stream_probe::RosStreamProbeNode
   ```

3. In a third terminal, start the HCM side:

   ```bash
   source /usr/ros/jazzy/setup.bash
   qrb_ros_adapter_stream_probe
   ```

   It initializes and starts both nodes and sends 1 MiB PointCloud2 frames from
   HCM to ROS at 30 FPS. Type `help` for the lifecycle commands, and `quit` to
   stop and print the summary. The ROS probe prints its summary in the Bridge
   terminal when the Bridge stops.

Both sides take the same parameters, such as the Adapter config
(`adapter_r2h.json` for ROS to HCM), the rate, the payload size and the
duration; `--help` lists them:

```bash
qrb_ros_adapter_stream_probe --help
ros2 run qrb_ros_adapter_stream_probe ros_stream_probe_node --help
```

A run, trimmed (HCM side, then the Bridge terminal):

```text
$ qrb_ros_adapter_stream_probe
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
summary: adapter: sessionConnected=0 | frameCount=180 | errorCount=0 | lastLatencyNs=1531302 | averageLatencyNs=2288826 | minLatencyNs=1052588 | maxLatencyNs=4258687 | noDataCount=0
```

```text
[INFO] [qrb_ros_adapter_bridge_manager]: registered flow 'stream_probe_adapter' at '/run/qrb_ros_adapter/flows/stream_probe_adapter-1.sock'
[INFO] [qrb_ros_adapter_stream_probe]: first frame: seq 0, check ok
[INFO] [qrb_ros_adapter_flow_stream_probe_adapter]: flow 'stream_probe_adapter': closed, frames in 180, out 180, no-data 0, conversion errors 0
[INFO] [qrb_ros_adapter_bridge_manager]: unregistered flow 'stream_probe_adapter'
[INFO] [qrb_ros_adapter_stream_probe]: summary: stream: received 180  lost 0
[INFO] [qrb_ros_adapter_stream_probe]: summary: e2e latency of 180 frames: avg 2.538 min 1.201 p50 2.556 p95 3.244 p99 3.506 max 3.547 ms
```

Each package also has unit tests, run with `ctest` in its build directory.

## License

This project is licensed under the BSD-3-Clause-Clear license.
