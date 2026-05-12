#!/bin/bash
dtc -I dts -O dtb -o pwm-pi5.dtbo pwm-pi5-overlay.dts
sudo cp pwm-pi5.dtbo /boot/firmware/overlays/

if ! grep -qxF "dtoverlay=pwm-pi5" /boot/firmware/config.txt; then
  echo "dtoverlay=pwm-pi5" | sudo tee -a /boot/firmware/config.txt
fi
