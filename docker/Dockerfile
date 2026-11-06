# syntax=docker/dockerfile:1
#
# Multi-stage image:
#   base     ROS 2 + all the dependencies resolved by rosdep (cached as long as no
#            package.xml changes)
#   dev      base + developer tools (used by the devcontainer)
#   build    compiles and tests the workspace
#   runtime  base + installed workspace, default command = pick-and-place demo
#
# Build from the repository root:
#   docker build -f docker/Dockerfile -t vision_pick_place .

ARG ROS_DISTRO=lyrical

FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}
WORKDIR /ws

# Only the manifests: the dependency layer is rebuilt only when dependencies change.
COPY arm_control_core/package.xml src/vision_pick_place/arm_control_core/
COPY arm_controllers/package.xml src/vision_pick_place/arm_controllers/
COPY perception_core/package.xml src/vision_pick_place/perception_core/
COPY pp_perception/package.xml src/vision_pick_place/pp_perception/
COPY pp_task/package.xml src/vision_pick_place/pp_task/
COPY pp_description/package.xml src/vision_pick_place/pp_description/
COPY pp_moveit_config/package.xml src/vision_pick_place/pp_moveit_config/
COPY pp_bringup/package.xml src/vision_pick_place/pp_bringup/
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
 && apt-get install -y --no-install-recommends python3-colcon-common-extensions ccache \
 && rm -rf /var/lib/apt/lists/*

FROM base AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
      clang-format clang-tidy gdb git less nano pre-commit python3-matplotlib python3-pandas \
 && rm -rf /var/lib/apt/lists/*

FROM base AS build
COPY . src/vision_pick_place
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPP_WARNINGS_AS_ERRORS=ON \
 && rm -rf build log

FROM base AS runtime
COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "pp_bringup", "pick_place.launch.py"]
