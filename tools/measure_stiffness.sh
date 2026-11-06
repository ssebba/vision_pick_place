#!/bin/bash
# Measures the translational stiffness of the Cartesian impedance controller in simulation:
# a constant force is applied to the wrist through Gazebo and the displacement of the tool
# center point (impedance_controller/current_pose) gives k = F / dx along each axis.
#
#   tools/measure_stiffness.sh [force_N]
#
# Run from a sourced workspace, with no other simulation running.
set -euo pipefail
set -m  # job control: the background launch keeps SIGINT (otherwise ignored) for a clean stop

FORCE="${1:-10.0}"
WORLD=pick_place
LINK="pp_robot::wrist_3_link"
WORK="$(mktemp -d)"

ros2 launch pp_bringup sim.launch.py headless:=true arm_controller:=impedance_controller \
  num_objects:=1 > "${WORK}/sim.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  timeout 20 tail --pid="${LAUNCH_PID}" -f /dev/null || true
  pkill -KILL -f '^(/opt/ros/[a-z]+/(lib|opt)/|[^ ]*/install/([^ ]+/)?lib/|ruby .*gz sim)' 2>/dev/null || true
}
trap cleanup EXIT

tcp_position() {
  ros2 topic echo --once --field pose.position /impedance_controller/current_pose \
    | awk '/^x:/ {x=$2} /^y:/ {y=$2} /^z:/ {z=$2} END {printf "%s %s %s", x, y, z}'
}

push() {  # push <fx> <fy> <fz>: constant force [N] in the world frame, until cleared
  gz topic -t "/world/${WORLD}/wrench/persistent" -m gz.msgs.EntityWrench \
    -p "entity: {name: \"${LINK}\", type: LINK}, wrench: {force: {x: $1, y: $2, z: $3}}"
}

release() {
  gz topic -t "/world/${WORLD}/wrench/clear" -m gz.msgs.Entity \
    -p "name: \"${LINK}\", type: LINK"
}

echo "Waiting for the impedance controller..."
until timeout 5 ros2 topic echo --once /impedance_controller/current_pose >/dev/null 2>&1; do
  sleep 2
done
sleep 5  # settle at the initial pose

printf "axis  force[N]  displacement[mm]  stiffness[N/m]\n"
for axis in x y z; do
  read -r x0 y0 z0 <<< "$(tcp_position)"
  case "${axis}" in
    x) push "${FORCE}" 0 0 ;;
    y) push 0 "${FORCE}" 0 ;;
    z) push 0 0 "${FORCE}" ;;
  esac
  sleep 4
  read -r x1 y1 z1 <<< "$(tcp_position)"
  release
  sleep 4
  python3 - "${axis}" "${FORCE}" "${x0}" "${y0}" "${z0}" "${x1}" "${y1}" "${z1}" <<'EOF'
import sys
axis, force = sys.argv[1], float(sys.argv[2])
p0 = [float(v) for v in sys.argv[3:6]]
p1 = [float(v) for v in sys.argv[6:9]]
d = p1['xyz'.index(axis)] - p0['xyz'.index(axis)]
print(f'{axis:>4}  {force:8.1f}  {1000 * d:16.2f}  {force / d if d else float("inf"):14.0f}')
EOF
done
