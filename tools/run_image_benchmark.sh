#!/usr/bin/env bash
set -euo pipefail

MODE="${1:?usage: $0 <mode> [seconds] [frequency] [launch_arg:=value ...]}"
DURATION="${2:-12}"
FREQUENCY="${3:-200}"
ARG_COUNT=$#
if (( ARG_COUNT >= 3 )); then
  shift 3
else
  shift "${ARG_COUNT}"
fi
EXTRA_LAUNCH_ARGS=("$@")

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ID="mode${MODE}_$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROOT_DIR}/benchmark_results/${RUN_ID}"
LOG_DIR="${OUT_DIR}/ros_log"
LAUNCH_LOG="${OUT_DIR}/launch.log"
CPU_LOG="${OUT_DIR}/cpu_samples.csv"
SUMMARY="${OUT_DIR}/summary.txt"

mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

export ROS_LOG_DIR="${LOG_DIR}"
if [[ -n "${BENCH_RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION="${BENCH_RMW_IMPLEMENTATION}"
else
  unset RMW_IMPLEMENTATION
fi

if [[ -n "${BENCH_ZENOH_CONFIG_OVERRIDE:-}" ]]; then
  export ZENOH_CONFIG_OVERRIDE="${BENCH_ZENOH_CONFIG_OVERRIDE}"
else
  unset ZENOH_CONFIG_OVERRIDE
fi

if [[ -n "${BENCH_FASTDDS_PROFILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${BENCH_FASTDDS_PROFILE}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${BENCH_RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
elif [[ "${BENCH_USE_FASTDDS_SHM:-0}" == "1" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${ROOT_DIR}/src/ros2_shm_msgs/config/shm_fastdds.xml"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${BENCH_RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset RMW_FASTRTPS_USE_QOS_FROM_XML
fi

ZENOH_PID=""
if [[ "${BENCH_START_ZENOHD:-0}" == "1" ]]; then
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 run rmw_zenoh_cpp rmw_zenohd > "${OUT_DIR}/zenohd.log" 2>&1 &
  ZENOH_PID="$!"
  sleep 2
fi

ROUDI_PID=""
if [[ "${MODE}" == "5" || "${BENCH_START_ROUDI:-0}" == "1" ]]; then
  /opt/ros/humble/bin/iox-roudi -c "${ROOT_DIR}/roudi_config.toml" > "${OUT_DIR}/roudi.log" 2>&1 &
  ROUDI_PID="$!"
  sleep 1
fi

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "-${LAUNCH_PID}" 2>/dev/null || kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "-${LAUNCH_PID}" 2>/dev/null || kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ROUDI_PID}" ]] && kill -0 "${ROUDI_PID}" 2>/dev/null; then
    kill -INT "${ROUDI_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${ROUDI_PID}" 2>/dev/null || true
  fi
  if [[ -n "${ZENOH_PID}" ]] && kill -0 "${ZENOH_PID}" 2>/dev/null; then
    kill -INT "${ZENOH_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${ZENOH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

setsid ros2 launch image_test image_test.launch.py \
  mode:="${MODE}" \
  image_pub_frequency:="${FREQUENCY}" \
  "${EXTRA_LAUNCH_ARGS[@]}" \
  > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

CONTAINER_PID=""
for _ in $(seq 1 80); do
  CONTAINER_PID="$(pgrep -n -f 'rclcpp_components.*component_container' || true)"
  if [[ -n "${CONTAINER_PID}" ]]; then
    break
  fi
  sleep 0.1
done

echo "timestamp,pid,cpu_percent_one_core,cpu_percent_all_cores,rss_kb" > "${CPU_LOG}"
if [[ -n "${CONTAINER_PID}" ]]; then
  CLK_TCK="$(getconf CLK_TCK)"
  NCPU="$(nproc)"
  PREV_TOTAL=""
  PREV_TIME=""
  END_TIME=$((SECONDS + DURATION))
  while (( SECONDS < END_TIME )); do
    if [[ ! -r "/proc/${CONTAINER_PID}/stat" ]]; then
      break
    fi
    STAT_CONTENT="$(cat "/proc/${CONTAINER_PID}/stat")"
    REST="${STAT_CONTENT##*) }"
    UTIME="$(awk '{print $12}' <<< "${REST}")"
    STIME="$(awk '{print $13}' <<< "${REST}")"
    TOTAL_TICKS=$((UTIME + STIME))
    NOW_NS="$(date +%s%N)"
    RSS_KB="$(awk '/VmRSS:/ {print $2}' "/proc/${CONTAINER_PID}/status" 2>/dev/null || echo 0)"
    CPU_ONE_CORE="0"
    CPU_ALL_CORES="0"
    if [[ -n "${PREV_TOTAL}" ]]; then
      DELTA_TICKS=$((TOTAL_TICKS - PREV_TOTAL))
      DELTA_NS=$((NOW_NS - PREV_TIME))
      if (( DELTA_NS > 0 )); then
        CPU_ONE_CORE="$(awk -v dt="${DELTA_TICKS}" -v hz="${CLK_TCK}" -v dns="${DELTA_NS}" \
          'BEGIN { printf "%.2f", (dt / hz) / (dns / 1000000000.0) * 100.0 }')"
        CPU_ALL_CORES="$(awk -v cpu="${CPU_ONE_CORE}" -v ncpu="${NCPU}" \
          'BEGIN { printf "%.2f", cpu / ncpu }')"
      fi
    fi
    echo "$(date +%s.%N),${CONTAINER_PID},${CPU_ONE_CORE},${CPU_ALL_CORES},${RSS_KB}" >> "${CPU_LOG}"
    PREV_TOTAL="${TOTAL_TICKS}"
    PREV_TIME="${NOW_NS}"
    sleep 0.5
  done
fi

cleanup
trap - EXIT

python3 - "$MODE" "$DURATION" "$FREQUENCY" "$LAUNCH_LOG" "$CPU_LOG" "$SUMMARY" "${RMW_IMPLEMENTATION:-default}" "${ZENOH_CONFIG_OVERRIDE:-}" "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" "${RMW_FASTRTPS_USE_QOS_FROM_XML:-}" "${EXTRA_LAUNCH_ARGS[*]}" <<'PY'
import csv
import re
import statistics
import sys

(
    mode,
    duration,
    frequency,
    launch_log,
    cpu_log,
    summary_path,
    rmw_impl,
    zenoh_override,
    fastdds_profile,
    rmw_fastrtps_use_qos,
    launch_args,
) = sys.argv[1:]

with open(launch_log, "r", errors="replace") as f:
    text = f.read()
text = re.sub(r"\x1b\[[0-9;]*m", "", text)

latencies = [float(m.group(1)) for m in re.finditer(r"([-+]?\d+(?:\.\d+)?)ms", text)]
if len(latencies) > 20:
    latencies_for_stats = latencies[20:]
else:
    latencies_for_stats = latencies

cpu_one_core_values = []
cpu_all_core_values = []
with open(cpu_log, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            one_core = float(row["cpu_percent_one_core"])
            all_cores = float(row["cpu_percent_all_cores"])
        except (ValueError, KeyError):
            continue
        if one_core > 0:
            cpu_one_core_values.append(one_core)
        if all_cores > 0:
            cpu_all_core_values.append(all_cores)

def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * pct / 100)
    return ordered[index]

def fmt(value, unit=""):
    if value is None:
        return "n/a"
    return f"{value:.3f}{unit}"

lines = [
    f"mode={mode}",
    f"duration_s={duration}",
    f"frequency_hz={frequency}",
    f"rmw={rmw_impl}",
    f"zenoh_config_override={zenoh_override or 'unset'}",
    f"fastrtps_profiles_file={fastdds_profile or 'unset'}",
    f"rmw_fastrtps_use_qos_from_xml={rmw_fastrtps_use_qos or 'unset'}",
    f"extra_launch_args={launch_args or 'none'}",
    f"latency_samples={len(latencies)}",
    f"latency_used_after_warmup={len(latencies_for_stats)}",
]

loan_status = re.search(r"mode 4 loaned image messages: ([^\r\n]+)", text)
if loan_status:
    lines.append(f"mode4_loan_status={loan_status.group(1).strip()}")

if latencies_for_stats:
    lines.extend([
        f"latency_min_ms={fmt(min(latencies_for_stats))}",
        f"latency_p50_ms={fmt(percentile(latencies_for_stats, 50))}",
        f"latency_mean_ms={fmt(statistics.mean(latencies_for_stats))}",
        f"latency_p95_ms={fmt(percentile(latencies_for_stats, 95))}",
        f"latency_max_ms={fmt(max(latencies_for_stats))}",
    ])
else:
    lines.append("latency_error=no latency samples found")

if cpu_one_core_values:
    lines.extend([
        f"cpu_samples={len(cpu_one_core_values)}",
        f"cpu_min_percent_one_core_basis={fmt(min(cpu_one_core_values))}",
        f"cpu_mean_percent_one_core_basis={fmt(statistics.mean(cpu_one_core_values))}",
        f"cpu_p95_percent_one_core_basis={fmt(percentile(cpu_one_core_values, 95))}",
        f"cpu_max_percent_one_core_basis={fmt(max(cpu_one_core_values))}",
        f"cpu_min_percent_all_cores_basis={fmt(min(cpu_all_core_values))}",
        f"cpu_mean_percent_all_cores_basis={fmt(statistics.mean(cpu_all_core_values))}",
        f"cpu_p95_percent_all_cores_basis={fmt(percentile(cpu_all_core_values, 95))}",
        f"cpu_max_percent_all_cores_basis={fmt(max(cpu_all_core_values))}",
    ])
else:
    lines.append("cpu_error=no cpu samples found")

with open(summary_path, "w") as f:
    f.write("\n".join(lines) + "\n")

print("\n".join(lines))
PY

echo "output_dir=${OUT_DIR}"
