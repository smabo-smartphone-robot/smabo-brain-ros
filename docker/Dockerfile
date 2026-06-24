# smabo-brain-ros ROS 2 environment: rosbridge + Nav2 + MoveIt2 + the smabo
# workspace, with smabo-brain importable so the ROS nodes can reuse brain's
# pure logic (odometry, topic prefixes).
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

# --- ROS dependencies (Nav2, MoveIt2, rosbridge, SLAM, model tooling) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      ros-humble-rosbridge-suite \
      ros-humble-rosapi \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-moveit \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
      ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# tf2_web_republisher feeds TF to the ros3djs viewer in smabo-web. It may not be
# packaged for every Humble mirror, so install it best-effort (the 3D view still
# renders grid/map/robot at origin without it — see README).
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-tf2-web-republisher \
    ; rm -rf /var/lib/apt/lists/* || true

# --- workspace layout ---
# Build context is this repo (see docker-compose.yml: context: .). smabo-brain is
# only needed at *runtime* (odom_node / relays import `brain`), so it is mounted
# via compose and put on PYTHONPATH by the entrypoint — not copied at build time.
ENV SMABO_WS=/opt/smabo_ws
ENV SMABO_BRAIN_DIR=/opt/smabo-brain
WORKDIR ${SMABO_WS}

# smabo-brain-ros sources -> colcon src
COPY . ${SMABO_WS}/src/smabo-brain-ros
RUN source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y || true \
    && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "smabo_bringup", "bringup.launch.py"]
