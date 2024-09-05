#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  ds2024 and imu and sllidar"

echo "sudo rm   /etc/udev/rules.d/ds2024.rules"
sudo rm   /etc/udev/rules.d/ds2024.rules

echo "sudo rm   /etc/udev/rules.d/imu.rules"
sudo rm   /etc/udev/rules.d/imu.rules

echo "sudo rm   /etc/udev/rules.d/sllidar.rules"
sudo rm   /etc/udev/rules.d/sllidar.rules

echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish  delete"
