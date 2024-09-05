#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ds2024 and imu and rplidar"
echo "ds2024 ttl connection as /dev/ds2024_base , check it using the command : ls -l /dev|grep ttyUSB"
echo "rplidar usb connection as /dev/sllidar_base , check it using the command : ls -l /dev|grep ttyUSB"
echo "imu usb connection as /dev/imu_base , check it using the command : ls -l /dev|grep ttyUSB"

echo "start copy ds2024.rules to  /etc/udev/rules.d/"
sudo cp ds2024.rules  /etc/udev/rules.d

echo "start copy imu.rules to  /etc/udev/rules.d/"
sudo cp imu.rules  /etc/udev/rules.d

echo "start copy sllidar.rules to  /etc/udev/rules.d/"
sudo cp sllidar.rules  /etc/udev/rules.d

echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish "
