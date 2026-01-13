# ─── Component Builder ─────────────────────────────────────────────────────
FROM registry.core:5000/llmy_base:jazzy-latest

ENV PYTHONDONTWRITEBYTECODE=1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash","-lc"]
#ENV ROS_WS=/ros_ws

# 1) Install any component-specific system packages

# 1b) Install any component-specific pip packages
RUN pip3 install --no-cache-dir --break-system-packages  icm20948

# 2) VCS repos → build into /vcs_ws/install


# 3) Copy & prepare any local source
WORKDIR /ros_ws
RUN mkdir -p src

# Copy source packages from context src/ folder
COPY src/ /ros_ws/src/

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
# FastDDS Discovery Server config
COPY superclient.xml /superclient.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/superclient.xml
ENV ROS_DISCOVERY_SERVER=192.168.99.183:11811
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
