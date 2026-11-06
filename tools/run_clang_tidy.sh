#!/bin/bash
# Runs clang-tidy (config: .clang-tidy) on the C++ sources of this repository, using the
# compile_commands.json of a colcon build made with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON.
#
#   tools/run_clang_tidy.sh [workspace_dir]      (default: ../.. from the repository)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$(cd "${REPO}/../.." && pwd)}"
JOBS="${JOBS:-$(nproc)}"
# Use the same GCC installation (libstdc++ headers) as the compiler: clang picks the highest
# version under /usr/lib/gcc, which may be a runtime-only directory without headers.
GCC_DIR="$(dirname "$(g++ -print-libgcc-file-name)")"
status=0
for pkg in arm_control_core arm_controllers perception_core pp_perception pp_task; do
  db="${WS}/build/${pkg}"
  if [ ! -f "${db}/compile_commands.json" ]; then
    echo "missing ${db}/compile_commands.json (build with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)" >&2
    exit 2
  fi
  echo "== clang-tidy ${pkg}"
  # Library sources and tools only; tests are checked by the compiler warnings.
  run-clang-tidy -quiet -j "${JOBS}" -p "${db}" -extra-arg="--gcc-install-dir=${GCC_DIR}" \
    "${REPO}/${pkg}/(src|tools)/.*\.cpp$" || status=1
done
exit ${status}
