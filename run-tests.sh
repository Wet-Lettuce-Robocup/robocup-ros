docker build --target robot-ros-builder -t robot-ros-tests .

docker run -it \
  --privileged \
  --net=host \
  -v /dev:/dev/ \
  -v /run/udev/:/run/udev/ \
  --group-add video \
  -e HOME=/tpm \
  robot-ros-tests \
  colcon test --event-handlers console_direct+
