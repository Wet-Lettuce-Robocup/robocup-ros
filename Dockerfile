FROM ros:kilted

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install -y python3-pip git python3-jinja2 \
  libboost-dev \
  libgnutls28-dev openssl libtiff-dev pybind11-dev \
  meson cmake \
  python3-yaml python3-ply \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev \
  python3-colcon-meson \
  ros-$ROS_DISTRO-robot-localization

# Clone and build raspberrypi's libcamera fork
RUN git clone https://github.com/raspberrypi/libcamera.git

WORKDIR /app/libcamera

RUN meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
  && ninja -C build install

# Clone and build the camera_ros node
WORKDIR /app/src

RUN git clone https://github.com/christianrauch/camera_ros.git

WORKDIR /app

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" \
  && rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys=libcamera \
  && colcon build --event-handlers=console_direct+

RUN apt-get update && apt-get install -y python3-serial python3-smbus2 \
  python3-lgpio python3-gpiozero \
  python3-opencv

ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN pip3 install --no-cache-dir --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org rpi5-ws2812 vl53l5cx-ctypes

COPY ./ros /app/src
COPY docker_entrypoint.sh /app/

WORKDIR /app

RUN source /app/docker_entrypoint.sh \
  && rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys=libcamera \
  && colcon build --symlink-install

ENV ROS_DOMAIN_ID=1
ENV ROS_LOCALHOST_ONLY=0

ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD ["ros2", "launch", "robot_core", "state_manager.launch.py"]
