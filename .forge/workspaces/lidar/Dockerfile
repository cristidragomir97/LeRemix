# ─── Component Builder ─────────────────────────────────────────────────────
FROM qwiki.lan:5000/llmy/llmy_base:jazzy-latest

ENV PYTHONDONTWRITEBYTECODE=1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash","-lc"]
#ENV ROS_WS=/ros_ws

# 1) Install any component-specific system packages

# 1b) Install any component-specific pip packages

# 2) VCS repos → build into /vcs_ws/install

# ─── Ensure workspace exists ──────────────────────────────────────────────
RUN mkdir -p /vcs_ws/src

# ─── Import repos.yaml ────────────────────────────────────────────────────
COPY repos.yaml /vcs_ws/repos.yaml

# Install vcstool if needed
RUN apt-get update && apt-get install -y python3-vcstool && rm -rf /var/lib/apt/lists/*

# Import VCS repositories
RUN vcs import /vcs_ws/src < /vcs_ws/repos.yaml

# ─── Build workspace ──────────────────────────────────────────────────────
WORKDIR /vcs_ws
RUN apt-get update \
 && source /opt/ros/$ROS_DISTRO/setup.bash \
 && rosdep init || true \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install --event-handlers console_direct+

# ─── Optional: Validate build success ─────────────────────────────────────




# 3) Copy & prepare any local source


# RMW Configuration
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER=tcp/192.168.99.184:7447
