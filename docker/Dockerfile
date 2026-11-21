# smabo-brain-ros ROS 2 environment: rosbridge + Nav2 + MoveIt2 + the smabo
# workspace, with smabo-brain importable so the ROS nodes can reuse brain's
# pure logic (odometry, topic prefixes).
#
# ros:humble (ros-base) is published for amd64 and arm64, so on an Apple Silicon
# Mac / Raspberry Pi the stack runs natively. (osrf/ros:humble-desktop is amd64
# only: under emulation Nav2 + rosbridge could not keep up and the whole graph
# stalled for seconds at a time.) Nothing here needs the desktop GUI tools.
FROM ros:humble

SHELL ["/bin/bash", "-c"]

# --- ROS dependencies (Nav2, MoveIt2, rosbridge, SLAM, model tooling) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-opencv \
      python3-numpy \
      ros-humble-rosbridge-suite \
      ros-humble-rosapi \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-cartographer-ros \
      ros-humble-moveit \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
      ros-humble-topic-tools \
      ros-humble-audio-common-msgs \
      ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# aiortc: webrtc_camera_node terminates smabo-app's WebRTC video as a peer and
# republishes frames as native sensor_msgs/Image (camera bypasses rosbridge for
# real-time — see README / design.md). Pulls PyAV (ffmpeg) + pylibsrtp. The apt
# libs help the build on architectures without prebuilt wheels (e.g. ARM/RPi).
RUN apt-get update && apt-get install -y --no-install-recommends \
      libsrtp2-dev libopus-dev libvpx-dev pkg-config \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir aiortc

# vosk: default speech-to-text engine for speech_recognizer_node (brain.stt).
# Offline; faster-whisper is the optional, heavier alternative (install only
# when stt_engine:=whisper). Imported lazily, so the node runs without them.
RUN python3 -m pip install --no-cache-dir vosk

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
