# ==================== BASE / COMMON ====================
FROM ros:kilted AS base

LABEL type = "base"

RUN apt-get update && apt-get install -y \
  python3-pip git python3-jinja2 \
  libboost-dev \
  libgnutls28-dev openssl libtiff-dev pybind11-dev \
  meson cmake \
  python3-yaml python3-ply \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev \
  python3-colcon-meson \
  build-essential \
  && rm -rf /var/lib/apt/lists/*

RUN git config --global http.sslVerify false

# ========== HAILORT STAGE ==========
FROM base AS hailo-base

LABEL type = "hailo-base"

ENV PIP_BREAK_SYSTEM_PACKAGES=1

COPY hailo_dependencies/hailort_5.3.0_arm64.deb /tmp/
COPY hailo_dependencies/hailort-5.3.0-cp312-cp312-linux_aarch64.whl /tmp/

RUN apt-get update && \
  apt-get install -y \
  libusb-1.0-0 \
  /tmp/hailort_5.3.0_arm64.deb && \
  pip3 install --no-cache-dir /tmp/hailort-*.whl && \
  python3 -c "import hailo_platform" && \
  rm -rf /var/lib/apt/lists/* && \
  rm -f /tmp/*

# ==================== LIBCAMERA STAGE ====================
FROM base AS libcamera-builder

LABEL type = "libcamera-builder"

WORKDIR /build/libcamera

RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git . \
  && meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
  && ninja -C build install

# ==================== OPENCV STAGE ====================
FROM base AS opencv-builder

LABEL type = "opencv-builder"

WORKDIR /build/opencv

RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential libgtk-3-dev

RUN git clone --depth 1 --branch 4.x https://github.com/opencv/opencv.git \
  && git clone --depth 1 --branch 4.x https://github.com/opencv/opencv_contrib.git

WORKDIR /build/opencv/opencv/build
RUN cmake .. \
  -DCMAKE_BUILD_TYPE=RELEASE \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -DWITH_LIBCAMERA=ON \
  && make -j$(nproc) \
  && make install

# ==================== ROS2 EXTERNAL PACKAGES ====================
FROM hailo-base AS external-ros-builder

LABEL type = "external-ros-builder"

COPY --from=opencv-builder /usr/local /usr/local
COPY --from=libcamera-builder /usr/local /usr/local
RUN ldconfig

RUN apt-get update && apt-get install -y libboost-python-dev ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4

WORKDIR /underlay_ws
RUN --mount=type=cache,target=/var/lib/apt/lists \
  --mount=type=cache,target=/var/cache/apt/archives \
  --mount=type=cache,target=/underlay_ws/build \
  --mount=type=cache,target=/underlay_ws/ccache \
  export CCACHE_DIR=/underlay_ws/ccache && \
  mkdir -p src \
  && git clone --depth 1 https://github.com/christianrauch/camera_ros.git src/camera_ros \
  && git clone --depth 1 --branch rolling https://github.com/ros-perception/vision_opencv.git \
  && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys='libcamera opencv opencv4 libopencv-dev python3-opencv libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libopencv-highgui-dev libopencv-features2d-dev libopencv-calib3d-dev cv_bridge' \
  && colcon build --packages-select camera_ros cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=${OpenCV_DIR} --event-handlers=console_direct+"

# ==================== ROS2 ROBOT PACKAGES ====================
FROM external-ros-builder AS robot-ros-builder

LABEL type = "robot-ros-builder"

WORKDIR /overlay_ws
RUN mkdir -p src

COPY ros/ ./src/

# Install dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt-get update \
  && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y --skip-keys='libcamera opencv opencv4 libopencv-dev python3-opencv libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libopencv-highgui-dev libopencv-features2d-dev libopencv-calib3d-dev' \
  && rm -rf /var/lib/apt/lists/*"

# Build overlay on top of underlay
RUN --mount=type=cache,target=/overlay_ws/build \
  --mount=type=cache,target=/overlay_ws/ccache \
  export CCACHE_DIR=/overlay_ws/ccache && \
  export PATH="/usr/lib/ccache:$PATH" && \
  /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ==================== RUNTIME STAGE ====================
FROM hailo-base AS runtime

LABEL type = "runtime"

# Python dependencies
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install --no-cache-dir --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org rpi5-ws2812 adafruit-circuitpython-vl53l1x
COPY docker_entrypoint.sh /

RUN apt-get update && apt-get -y install ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-vision-msgs \
  python3-serial python3-smbus2 \
  python3-lgpio python3-gpiozero \
  python3-opencv python3-luma.oled python3-pil \
  ffmpeg \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libboost-python1.83.0 \
  python3-dev && \
  ldconfig

COPY --from=libcamera-builder /usr/local /usr/local
COPY --from=opencv-builder /usr/local /usr/local
COPY --from=external-ros-builder /underlay_ws/install /underlay_ws/install
COPY --from=robot-ros-builder /overlay_ws/install /overlay_ws/install

RUN ldconfig

ENV ROS_WS=/overlay_ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
  echo "source /underlay_ws/install/setup.bash" >> /etc/bash.bashrc && \
  echo "source ${ROS_WS}/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=1
ENV ROS_LOCALHOST_ONLY=0

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["ros2", "launch", "robot_core", "state_manager.launch.py"]