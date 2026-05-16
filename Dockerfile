# ==================== BASE / COMMON ====================
FROM ros:kilted AS base

RUN apt-get update && apt-get install -y python3-pip git python3-jinja2 \
  libboost-dev \
  libgnutls28-dev openssl libtiff-dev pybind11-dev \
  meson cmake \
  python3-yaml python3-ply \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev \
  python3-colcon-meson

RUN git config --global http.sslVerify false

# ==================== LIBCAMERA STAGE ====================
FROM base AS libcamera-builder
WORKDIR /build/libcamera

RUN git clone --depth 1 https://github.com/raspberrypi/libcamera.git . \
  && meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
  && ninja -C build install

# ==================== OPENCV STAGE ====================
FROM base AS opencv-builder
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
FROM base AS external-ros-builder

COPY --from=opencv-builder /usr/local /usr/local
COPY --from=libcamera-builder /usr/local /usr/local
RUN ldconfig

RUN apt-get update && apt-get install -y libboost-python-dev
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4

WORKDIR /underlay_ws
RUN mkdir -p src \
  && git clone --depth 1 https://github.com/christianrauch/camera_ros.git src/camera_ros \
  && git clone --depth 1 --branch rolling https://github.com/ros-perception/vision_opencv.git \
  && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys='libcamera opencv opencv4 libopencv-dev python3-opencv libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libopencv-highgui-dev libopencv-features2d-dev libopencv-calib3d-dev cv_bridge' \
  && colcon build --packages-select camera_ros cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=${OpenCV_DIR} --event-handlers=console_direct+"

# ==================== ROS2 ROBOT PACKAGES ====================
FROM external-ros-builder AS robot-ros-builder

WORKDIR /overlay_ws
RUN mkdir -p src

COPY ros/ ./src/

# Install dependencies
RUN apt-get update \
  && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y --skip-keys='libcamera opencv opencv4 libopencv-dev python3-opencv libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libopencv-highgui-dev libopencv-features2d-dev libopencv-calib3d-dev' \
  && rm -rf /var/lib/apt/lists/*"

# Build overlay on top of underlay
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ==================== RUNTIME STAGE ====================
FROM base AS runtime

# Python dependencies
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install --no-cache-dir --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org rpi5-ws2812 vl53l5cx-ctypes
COPY docker_entrypoint.sh /

RUN apt-get update && apt-get -y install ros-$ROS_DISTRO-robot-localization \
  python3-serial python3-smbus2 \
  python3-lgpio python3-gpiozero \
  python3-opencv python3-luma.oled python3-pil

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
#CMD ["ros2", "launch", "robot_core", "state_manager.launch.py"]
CMD ["/bin/bash"]
