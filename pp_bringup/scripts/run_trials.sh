#!/bin/bash
# Pick-and-place benchmark: one fresh simulation per (seed, arm controller), headless.
# Each trial appends one row per object to <output_dir>/pick_place.csv; completed trials
# (logs present with a result) are skipped, so the campaign can be resumed.
#
#   ros2 run pp_bringup run_trials.sh [output_dir]
#
# Each trial also records the arm controller state (joint reference, feedback and error) in
# <output_dir>/bags/<run_id>, for the tracking analysis (tools/analyze_trials.py).
#
# Environment overrides: SEEDS (default "1 .. 10"), CONTROLLERS, NUM_OBJECTS, TRIAL_TIMEOUT.
set -u

OUT="${1:-${HOME}/pick_place_results}"
SEEDS="${SEEDS:-$(seq -s ' ' 1 10)}"
CONTROLLERS="${CONTROLLERS:-arm_controller ctc_controller}"
NUM_OBJECTS="${NUM_OBJECTS:-3}"
TRIAL_TIMEOUT="${TRIAL_TIMEOUT:-600}"
mkdir -p "${OUT}/logs" "${OUT}/bags"

cleanup() {
  # ROS nodes, Gazebo (gz-sim-main lives under /opt/ros/<distro>/opt) and our own nodes.
  local pattern='^(/opt/ros/[a-z]+/(lib|opt)/|[^ ]*/install/([^ ]+/)?lib/|ruby .*gz sim)'
  for sig in INT TERM KILL; do
    pgrep -f -- "${pattern}" >/dev/null || break
    pkill -"${sig}" -f -- "${pattern}"
    sleep 2
  done
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
}

for seed in ${SEEDS}; do
  for controller in ${CONTROLLERS}; do
    run_id="seed${seed}_${controller}"
    log="${OUT}/logs/${run_id}.log"
    if [ -f "${log}" ] && grep -q "Task finished" "${log}"; then
      echo "[skip] ${run_id}"
      continue
    fi
    echo "[run ] ${run_id}"
    rm -rf "${OUT:?}/bags/${run_id:?}"  # a rerun replaces the bag of a failed attempt
    start=$(date +%s)
    timeout --signal=INT --kill-after=30 "${TRIAL_TIMEOUT}" \
      ros2 launch pp_bringup pick_place.launch.py headless:=true use_rviz:=false \
        seed:="${seed}" num_objects:="${NUM_OBJECTS}" arm_controller:="${controller}" \
        results_file:="${OUT}/pick_place.csv" run_id:="${run_id}" \
        state_bag:="${OUT}/bags/${run_id}" \
      > "${log}" 2>&1
    cleanup
    if grep -q "Task finished" "${log}"; then
      echo "[done] ${run_id} in $(( $(date +%s) - start )) s: $(grep -o 'Task finished: .*' "${log}" | tail -1)"
    else
      echo "[fail] ${run_id}, see ${log}"
    fi
  done
done
