# File: ros2_ws/src/audio_alert_cpp/Dockerfile
FROM ros:foxy

SHELL ["/bin/bash", "-lc"]

# Remove any ROS “latest” entries
RUN rm -f /etc/apt/sources.list.d/ros-latest.list \
    && rm -f /etc/apt/sources.list.d/ros2-latest.list \
    && rm -f /etc/apt/sources.list.d/ros2.list

# Remove any snapshot.ros.org lines from all APT lists
RUN sed -i '/snapshots\.ros\.org/d' /etc/apt/sources.list \
    && sed -i '/snapshots\.ros\.org/d' /etc/apt/sources.list.d/*.list || true

RUN apt-get update && \
    apt-get install -y \
      python3-colcon-common-extensions \
      pkg-config \
      libasound2-dev \
      libsndfile1-dev \
      nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN mkdir -p src/audioalert_msg src/audio_alert_cpp
COPY src/audioalert_msg   src/audioalert_msg
COPY src/audio_alert_cpp  src/audio_alert_cpp

COPY src/audio_alert_cpp/asound.conf /etc/asound.conf

RUN rosdep fix-permissions && \
    rosdep update  || echo "rosdep update failed, continuing" && \
    rosdep install --from-paths \
      src/audioalert_msg src/audio_alert_cpp \
      --ignore-src -r -y

RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --packages-select \
      audioalert_msg audio_alert_cpp

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash","-lc",\
 "source /opt/ros/foxy/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  exec \"$0\" \"$@\""]
CMD ["ros2","run","audio_alert_cpp","fleet_playback"]

