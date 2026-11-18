#!/usr/bin/env bash
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# One terminal of a stream_probe run: `ros` starts the ROS IPC Bridge with the ROS probe, `hcm`
# starts the HCM side. Both write the same Adapter static config from the options.

set -uo pipefail

usage() {
  cat <<'EOF'
usage: qrb_ros_adapter_stream_test.sh ros|hcm [OPTION ...] [-- PROBE_ARG ...]

Runs one side of a stream through the HCM IPC Adapter and the ROS IPC Bridge.
Start `ros` in one terminal and then `hcm` in another, with the same options.
  ros   the ROS IPC Bridge, with the ROS probe in its container; Ctrl-C stops it
  hcm   the HCM side, the Adapter and the HCM probe; type `help` for its commands

options:
  -d, --direction h2r|r2h    h2r: HCM to ROS, r2h: ROS to HCM (h2r)
  -c, --contract points|pose PointCloud2 or PoseWithCovarianceStamped (points)
  -s, --size N[K|M]          payload of a PointCloud2 frame (1M)
  -r, --rate FPS             frames per second of the sender, 0: unlimited (30)
  -t, --duration S           seconds to run, 0: until `quit` or Ctrl-C (0)
  -q, --qos best_effort|reliable
                             QoS reliability of the Flow (best_effort)
  -n, --depth N              QoS depth of the Flow, 1..32 (5)
      --dds                  ros: run the ROS probe in its own process, over DDS
  -l, --log-dir DIR          write the probe's text log to DIR/hcm.log or DIR/ros.log
  -h, --help
PROBE_ARG goes to the probe as is: an option of qrb_ros_adapter_stream_probe for
hcm, `-p NAME:=VALUE` for ros.
EOF
}

die() {
  echo "qrb_ros_adapter_stream_test.sh: $*" >&2
  exit 2
}

[ $# -ge 1 ] || { usage >&2; exit 2; }
case $1 in
  -h | --help) usage; exit 0 ;;
  ros | hcm) side=$1 ;;
  *) die "the first argument must be ros or hcm, not '$1'" ;;
esac
shift

direction=h2r contract=points size=1M rate=30 duration=0 qos=best_effort depth=5 dds=0 logdir=
while [ $# -gt 0 ]; do
  case $1 in
    -h | --help) usage; exit 0 ;;
    --dds) dds=1; shift; continue ;;
    --) shift; break ;;
    -d | --direction | -c | --contract | -s | --size | -r | --rate | -t | --duration | -q | --qos | \
      -n | --depth | -l | --log-dir)
      [ $# -ge 2 ] || die "$1 needs a value" ;;
    *) die "unknown option '$1', see --help" ;;
  esac
  case $1 in
    -d | --direction) direction=$2 ;;
    -c | --contract) contract=$2 ;;
    -s | --size) size=$2 ;;
    -r | --rate) rate=$2 ;;
    -t | --duration) duration=$2 ;;
    -q | --qos) qos=$2 ;;
    -n | --depth) depth=$2 ;;
    -l | --log-dir) logdir=$2 ;;
  esac
  shift 2
done

case $direction in
  h2r) code=0 ;;
  r2h) code=1 ;;
  *) die "--direction must be h2r or r2h" ;;
esac
case $contract in
  points) type=qrb.contract.sensor.v1.PointCloud2 ;;
  pose) type=qrb.contract.geometry.v1.PoseWithCovarianceStamped ;;
  *) die "--contract must be points or pose" ;;
esac
case $qos in
  best_effort | reliable) ;;
  *) die "--qos must be best_effort or reliable" ;;
esac
[[ $depth =~ ^[0-9]+$ ]] && depth=$((10#$depth)) && [ "$depth" -ge 1 ] && [ "$depth" -le 32 ] ||
  die "--depth must be in 1..32"
[[ $size =~ ^([0-9]+)([KkMm]?)$ ]] || die "--size must be N, NK or NM"
bytes=$((10#${BASH_REMATCH[1]}))
case ${BASH_REMATCH[2]} in
  K | k) bytes=$((bytes * 1024)) ;;
  M | m) bytes=$((bytes * 1048576)) ;;
esac
[ "$bytes" -gt 0 ] || die "--size must be greater than 0"
# The HCM side receives into a buffer it allocates up front: at least the largest frame.
buffer=$((bytes > 33554432 ? bytes : 33554432))

work=${TMPDIR:-/tmp}/qrb_ros_adapter_test
mkdir -p "$work" || die "cannot create $work"
[ -z "$logdir" ] || mkdir -p "$logdir" || die "cannot create $logdir"

# Both sides must agree on the Flow, so each writes the same file.
cfg=$work/adapter_${direction}_${contract}_${qos}_${depth}.json
printf '{"static":{"name":"stream_probe_adapter","id":0,"contract_type":"%s","direction":%d,"ros_topic":"/qrb_ros_adapter/stream_probe/%s","qos_profile":{"reliability":"%s","durability":"volatile","depth":%d}}}\n' \
  "$type" "$code" "$contract" "$qos" "$depth" > "$cfg.$$" && mv -f "$cfg.$$" "$cfg" || die "cannot write $cfg"

if [ "$side" = hcm ]; then
  command -v qrb_ros_adapter_stream_probe > /dev/null ||
    die "qrb_ros_adapter_stream_probe not found: source /usr/ros/jazzy/setup.bash first"
  args=(--adapter-config "$cfg" --rate "$rate" --payload-bytes "$bytes" --buffer-bytes "$buffer"
    --duration "$duration")
  [ -z "$logdir" ] || args+=(--log "$logdir/hcm.log")
  exec qrb_ros_adapter_stream_probe "${args[@]}" "$@"
fi

command -v ros2 > /dev/null || die "ros2 not found: source /usr/ros/jazzy/setup.bash first"
pid=$(pgrep -x ros_ipc_bridge | head -1)
[ -z "$pid" ] || die "a ROS IPC Bridge is already running (pid $pid); stop it first"
# ROS writes its logs under ~/.ros; on a read-only root filesystem it cannot.
if [ -z "${ROS_HOME:-}${ROS_LOG_DIR:-}" ] && ! mkdir -p "${HOME:-/nonexistent}/.ros" 2> /dev/null; then
  export ROS_HOME=$work/ros
fi

params=(-p adapter_config:="$cfg" -p rate:="$rate" -p payload_bytes:="$bytes"
  -p buffer_bytes:="$buffer" -p duration:="$duration")
[ -z "$logdir" ] || params+=(-p log:="$logdir/ros.log")

# The binary itself rather than `ros2 run`: a signal to that wrapper leaves the Bridge behind, and
# background jobs of a script ignore SIGINT, so the Bridge is stopped with SIGTERM below.
"$(ros2 pkg prefix qrb_ros_adapter)/lib/qrb_ros_adapter/ros_ipc_bridge" &
bridge=$!
probe=

stop() {
  trap - INT TERM
  if [ -n "$probe" ]; then
    kill -TERM "$probe" 2> /dev/null
    wait "$probe" 2> /dev/null
  fi
  kill -TERM "$bridge" 2> /dev/null
  wait "$bridge"
  exit $?
}
trap stop INT TERM

for ((i = 0; i < 120; i++)); do
  kill -0 "$bridge" 2> /dev/null || { wait "$bridge"; exit $?; }
  ros2 component list 2> /dev/null | grep -qx /qrb_ros_adapter_container && break
  sleep 0.5
done
[ $i -lt 120 ] || { echo "the Bridge container did not come up" >&2; stop; }

if [ "$dds" = 1 ]; then
  "$(ros2 pkg prefix qrb_ros_adapter_stream_probe)/lib/qrb_ros_adapter_stream_probe/ros_stream_probe_node" \
    --ros-args "${params[@]}" "$@" &
  probe=$!
  # Over DDS, a probe that comes up after the HCM side has started sending can miss frames while
  # it is being matched: wait until it is in the ROS graph.
  for ((i = 0; i < 60; i++)); do
    kill -0 "$probe" 2> /dev/null || { echo "the ROS probe exited" >&2; stop; }
    ros2 node list 2> /dev/null | grep -qx /qrb_ros_adapter_stream_probe && break
    sleep 0.5
  done
  [ $i -lt 60 ] || { echo "the ROS probe did not come up" >&2; stop; }
else
  # The first load after the ros2 daemon starts can miss the container: retry.
  for i in 1 2 3; do
    out=$(ros2 component load /qrb_ros_adapter_container qrb_ros_adapter_stream_probe \
      qrb::adapter::stream_probe::RosStreamProbeNode "${params[@]}" "$@" 2>&1)
    [[ $out == *"Loaded component"* ]] && break
    [ $i -lt 3 ] && sleep 2
  done
  echo "$out" | tail -1
  [[ $out == *"Loaded component"* ]] || stop
fi
echo "Start the HCM side in another terminal: qrb_ros_adapter_stream_test.sh hcm, with the same options."
echo "Ctrl-C stops the Bridge."

wait "$bridge"
rc=$?
[ -z "$probe" ] || { kill -TERM "$probe" 2> /dev/null; wait "$probe" 2> /dev/null; }
exit $rc
