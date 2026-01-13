# ─── Component Builder ─────────────────────────────────────────────────────
FROM docker.io/dragomirxyz/llmy_sim_cuda_base:jazzy-latest

ENV PYTHONDONTWRITEBYTECODE=1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash","-lc"]
#ENV ROS_WS=/ros_ws

# 1) Install any component-specific system packages
RUN apt-get update && apt-get install -y ros-jazzy-rviz2 \
  && rm -rf /var/lib/apt/lists/*

# 1b) Install any component-specific pip packages

# 2) VCS repos → build into /vcs_ws/install


# 3) Copy & prepare any local source
WORKDIR /ros_ws
RUN mkdir -p src

# Copy source packages directly from their original locations
COPY ros/src/llmy_description /ros_ws/src/llmy_description
COPY ros/src/llmy_rviz /ros_ws/src/llmy_rviz

# Verify we have actual files
RUN echo '=== Checking source directory structure ===' \
 && ls -la src/ \
 && echo '=== Found package.xml files ===' \
 && find src -type f -name package.xml -exec echo '{}' \;

# Install dependencies
RUN set -exo pipefail \
 && apt-get update \
 && source /opt/ros/$ROS_DISTRO/setup.bash \
 && if [ -f /ros_ws_common/install/setup.bash ]; then source /ros_ws_common/install/setup.bash; fi \
 && (rosdep init 2>/dev/null || true) \
 && rosdep update \
 && echo '=== Checking what rosdep will install ===' \
 && (rosdep keys --rosdistro $ROS_DISTRO --from-paths src --ignore-src | sort | uniq || echo 'No rosdep keys found') \
 && echo '=== Installing dependencies ===' \
 && rosdep install --rosdistro $ROS_DISTRO --from-paths src --ignore-src -r -y

# Sanity check
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && echo '=== Verifying dependencies ===' \
 && rosdep check --rosdistro $ROS_DISTRO --from-paths src --ignore-src || true


# RMW Configuration
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER=tcp/127.0.0.1:7447
