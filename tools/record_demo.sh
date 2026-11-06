#!/bin/bash
# Records the README GIF: one complete pick-and-place task, headless, filmed by an extra
# Gazebo camera with an oblique view of the cell (pp_bringup record_dir argument). Gazebo
# saves every frame; ffmpeg turns them into an optimized GIF.
#
#   tools/record_demo.sh [output.gif] [arm_controller] [seed]
#
# Needs ffmpeg; run from a sourced workspace.
set -euo pipefail

OUT="${1:-docs/media/demo.gif}"
CONTROLLER="${2:-arm_controller}"
SEED="${3:-1}"
SPEEDUP="${SPEEDUP:-3}"
WORK="$(mktemp -d)"
mkdir -p "$(dirname "${OUT}")" "${WORK}/frames"

cleanup() {
  pkill -KILL -f '^(/opt/ros/[a-z]+/(lib|opt)/|[^ ]*/install/([^ ]+/)?lib/|ruby .*gz sim)' 2>/dev/null || true
}
trap cleanup EXIT

# With results_file set the launch shuts down by itself when the task is over.
ros2 launch pp_bringup pick_place.launch.py headless:=true use_rviz:=false seed:="${SEED}" \
  arm_controller:="${CONTROLLER}" results_file:="${WORK}/result.csv" \
  record_dir:="${WORK}/frames" > "${WORK}/launch.log" 2>&1 || true
grep -o 'Task finished: .*' "${WORK}/launch.log" || echo "warning: the task did not finish"

# Gazebo numbers the frames without padding: renumber them in order for ffmpeg.
i=0
while IFS= read -r frame; do
  mv "${frame}" "$(printf '%s/f_%06d.png' "${WORK}/frames" "${i}")"
  i=$((i + 1))
done < <(find "${WORK}/frames" -name '*.png' ! -name 'f_*' | sort -V)
echo "${i} frames"

# Frames are 15 Hz of simulated time: played at 15 * SPEEDUP fps, resampled to 10 fps.
RATE=$((15 * SPEEDUP))
ffmpeg -loglevel error -y -framerate "${RATE}" -i "${WORK}/frames/f_%06d.png" \
  -vf "fps=10,scale=800:-1:flags=lanczos,palettegen=stats_mode=diff" "${WORK}/palette.png"
ffmpeg -loglevel error -y -framerate "${RATE}" -i "${WORK}/frames/f_%06d.png" \
  -i "${WORK}/palette.png" \
  -lavfi "fps=10,scale=800:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=5" \
  "${OUT}"
echo "Wrote ${OUT} ($(du -h "${OUT}" | cut -f1))"
