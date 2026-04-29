#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

PARAMS_FILE="$REPO_ROOT/config/params_1.yaml"
MODE="optimized"
REPEAT=1
TIMEOUT_SEC=25
SUBSCRIBER_WAIT_SEC=2
MESSAGE_TYPE="sensor_msgs/msg/Image"
VIDEO_DEVICE=""
ROS_DOMAIN_ID_OVERRIDE=""
LABEL_PREFIX="usb_cam_latency"
SAVE_ROOT="${TMPDIR:-/tmp}"

usage() {
  cat <<'EOF'
用法:
  measure_startup_latency.sh [选项]

说明:
  运行 usb_cam 节点并订阅第一帧图像，计算启动延迟。
  默认使用优化配置；传入 --mode baseline 可切换为对照配置。

选项:
  --mode optimized|baseline    测试模式，默认 optimized
  --repeat N                   重复测试次数，默认 1
  --params-file PATH           参数文件，默认 config/params_1.yaml
  --video-device PATH          覆盖 video_device，例如 /dev/video0
  --message-type TYPE          订阅消息类型，默认 sensor_msgs/msg/Image
  --timeout-sec N              单次等待首帧的超时时间，默认 25 秒
  --subscriber-wait-sec N      启动节点前等待订阅端就绪的时间，默认 2 秒
  --domain-id N                覆盖 ROS_DOMAIN_ID
  --label-prefix PREFIX        测试节点和话题名前缀
  --save-root DIR              日志目录根路径，默认 /tmp
  -h, --help                   显示帮助

指标说明:
  startup_ms:
    从启动 usb_cam 进程到测试端收到第一帧消息的时间
  first_frame_after_start_ms:
    第一帧自己的 header.stamp 相对启动时刻的偏移
  frame_age_ms:
    收到第一帧时，这一帧已经“老了”多久

示例:
  ./scripts/measure_startup_latency.sh --mode optimized --repeat 3
  ./scripts/measure_startup_latency.sh --mode baseline --repeat 3
  ./scripts/measure_startup_latency.sh --video-device /dev/video0 --domain-id 71
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少命令: $1" >&2
    exit 1
  fi
}

cleanup_pid() {
  local pid="$1"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    sleep 0.5
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    sleep 0.5
  fi
  wait "$pid" 2>/dev/null || true
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --mode)
        MODE="$2"
        shift 2
        ;;
      --repeat)
        REPEAT="$2"
        shift 2
        ;;
      --params-file)
        PARAMS_FILE="$2"
        shift 2
        ;;
      --video-device)
        VIDEO_DEVICE="$2"
        shift 2
        ;;
      --message-type)
        MESSAGE_TYPE="$2"
        shift 2
        ;;
      --timeout-sec)
        TIMEOUT_SEC="$2"
        shift 2
        ;;
      --subscriber-wait-sec)
        SUBSCRIBER_WAIT_SEC="$2"
        shift 2
        ;;
      --domain-id)
        ROS_DOMAIN_ID_OVERRIDE="$2"
        shift 2
        ;;
      --label-prefix)
        LABEL_PREFIX="$2"
        shift 2
        ;;
      --save-root)
        SAVE_ROOT="$2"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "未知参数: $1" >&2
        usage >&2
        exit 1
        ;;
    esac
  done
}

validate_args() {
  if [[ "$MODE" != "optimized" && "$MODE" != "baseline" ]]; then
    echo "--mode 只支持 optimized 或 baseline" >&2
    exit 1
  fi

  if ! [[ "$REPEAT" =~ ^[0-9]+$ ]] || [[ "$REPEAT" -lt 1 ]]; then
    echo "--repeat 必须是大于等于 1 的整数" >&2
    exit 1
  fi

  if ! [[ "$TIMEOUT_SEC" =~ ^[0-9]+$ ]] || [[ "$TIMEOUT_SEC" -lt 1 ]]; then
    echo "--timeout-sec 必须是大于等于 1 的整数" >&2
    exit 1
  fi

  if ! [[ "$SUBSCRIBER_WAIT_SEC" =~ ^[0-9]+$ ]] || [[ "$SUBSCRIBER_WAIT_SEC" -lt 1 ]]; then
    echo "--subscriber-wait-sec 必须是大于等于 1 的整数" >&2
    exit 1
  fi

  if [[ ! -f "$PARAMS_FILE" ]]; then
    echo "参数文件不存在: $PARAMS_FILE" >&2
    exit 1
  fi

  if [[ -n "$ROS_DOMAIN_ID_OVERRIDE" ]] && ! [[ "$ROS_DOMAIN_ID_OVERRIDE" =~ ^[0-9]+$ ]]; then
    echo "--domain-id 必须是整数" >&2
    exit 1
  fi
}

run_one() {
  local run_index="$1"
  local run_label="${LABEL_PREFIX}_${MODE}_$(date +%s)_${run_index}"
  local topic="/${run_label}/image_raw"
  local outdir
  local echo_log
  local node_log
  local start_ms
  local receive_ms
  local sec
  local nsec
  local stamp_ms
  local startup_ms
  local first_frame_after_start_ms
  local frame_age_ms
  local loop_count
  local echo_pid
  local node_pid
  local -a ros_cmd

  outdir=$(mktemp -d "${SAVE_ROOT%/}/usb_cam_latency_${run_label}_XXXX")
  echo_log="$outdir/echo.log"
  node_log="$outdir/node.log"

  ros2 topic echo "$topic" "$MESSAGE_TYPE" --field header.stamp --once --no-daemon >"$echo_log" 2>&1 &
  echo_pid=$!
  sleep "$SUBSCRIBER_WAIT_SEC"

  start_ms=$(date +%s%3N)
  ros_cmd=(
    stdbuf -oL
    ros2 run usb_cam usb_cam_node_exe
    --ros-args
    -r "__node:=${run_label}"
    -r "image_raw:=${topic}"
    --params-file "$PARAMS_FILE"
  )

  if [[ -n "$VIDEO_DEVICE" ]]; then
    ros_cmd+=(-p "video_device:=${VIDEO_DEVICE}")
  fi

  if [[ "$MODE" == "baseline" ]]; then
    ros_cmd+=(
      -p "low_latency_mode:=false"
      -p "buffer_count:=4"
      -p "startup_frame_drop_count:=0"
    )
  fi

  "${ros_cmd[@]}" >"$node_log" 2>&1 &
  node_pid=$!

  loop_count=$((TIMEOUT_SEC * 10))
  for ((i = 0; i < loop_count; ++i)); do
    if ! kill -0 "$echo_pid" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  receive_ms=$(date +%s%3N)

  cleanup_pid "$node_pid"
  cleanup_pid "$echo_pid"

  sec=$(awk '/^sec:/{print $2; exit}' "$echo_log" || true)
  nsec=$(awk '/^nanosec:/{print $2; exit}' "$echo_log" || true)

  if [[ -z "$sec" || -z "$nsec" ]]; then
    echo "RESULT run=${run_index} status=FAILED logs=${outdir}"
    echo "NODE_LOG_TAIL_BEGIN"
    tail -n 40 "$node_log" || true
    echo "NODE_LOG_TAIL_END"
    echo "ECHO_LOG_BEGIN"
    sed -n '1,80p' "$echo_log" || true
    echo "ECHO_LOG_END"
    return 1
  fi

  stamp_ms=$((sec * 1000 + nsec / 1000000))
  startup_ms=$((receive_ms - start_ms))
  first_frame_after_start_ms=$((stamp_ms - start_ms))
  frame_age_ms=$((receive_ms - stamp_ms))

  echo "RESULT run=${run_index} status=OK mode=${MODE} topic=${topic} startup_ms=${startup_ms} first_frame_after_start_ms=${first_frame_after_start_ms} frame_age_ms=${frame_age_ms} logs=${outdir}"
}

main() {
  local success_count=0
  local startup_sum=0
  local first_frame_sum=0
  local frame_age_sum=0
  local result
  local startup_ms
  local first_frame_after_start_ms
  local frame_age_ms

  parse_args "$@"
  validate_args

  require_cmd ros2
  require_cmd awk
  require_cmd sed
  require_cmd date
  require_cmd stdbuf
  require_cmd mktemp
  require_cmd tail

  if [[ -n "$ROS_DOMAIN_ID_OVERRIDE" ]]; then
    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_OVERRIDE"
  fi

  echo "测试模式: $MODE"
  echo "重复次数: $REPEAT"
  echo "参数文件: $PARAMS_FILE"
  if [[ -n "$VIDEO_DEVICE" ]]; then
    echo "视频设备: $VIDEO_DEVICE"
  fi
  if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
  fi
  echo

  for ((run = 1; run <= REPEAT; ++run)); do
    result=$(run_one "$run")
    echo "$result"

    if [[ "$result" == *"status=OK"* ]]; then
      startup_ms=$(awk '{for (i = 1; i <= NF; ++i) if ($i ~ /^startup_ms=/) {split($i, a, "="); print a[2]}}' <<<"$result")
      first_frame_after_start_ms=$(awk '{for (i = 1; i <= NF; ++i) if ($i ~ /^first_frame_after_start_ms=/) {split($i, a, "="); print a[2]}}' <<<"$result")
      frame_age_ms=$(awk '{for (i = 1; i <= NF; ++i) if ($i ~ /^frame_age_ms=/) {split($i, a, "="); print a[2]}}' <<<"$result")

      startup_sum=$((startup_sum + startup_ms))
      first_frame_sum=$((first_frame_sum + first_frame_after_start_ms))
      frame_age_sum=$((frame_age_sum + frame_age_ms))
      success_count=$((success_count + 1))
    fi

    if [[ "$run" -lt "$REPEAT" ]]; then
      sleep 2
    fi
  done

  echo
  if [[ "$success_count" -eq 0 ]]; then
    echo "SUMMARY status=FAILED success_runs=0 total_runs=${REPEAT}"
    exit 1
  fi

  echo "SUMMARY status=OK success_runs=${success_count} total_runs=${REPEAT} avg_startup_ms=$((startup_sum / success_count)) avg_first_frame_after_start_ms=$((first_frame_sum / success_count)) avg_frame_age_ms=$((frame_age_sum / success_count))"
}

main "$@"
