docker run -it \
  --privileged \
  --net=host \
  -v /dev:/dev/ \
  -v /run/udev/:/run/udev/ \
  --group-add video \
  -e HOME=/tpm \
  wetlettuce/robot \
  colcon test --event-handlers console_direct+
