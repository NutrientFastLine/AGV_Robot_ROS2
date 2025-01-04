#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import math
import platform
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')
        self.declare_parameter('port', '/dev/imu_base')
        self.declare_parameter('baudrate', 921600)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.imu_pub = self.create_publisher(Imu, '/base/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/base/mag', 10)
        self.timer = self.create_timer(0.005, self.read_serial_data)  # 200 Hz

        self.buff = {}
        self.key = 0
        self.acceleration = [0, 0, 0]
        self.angular_velocity = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]
        self.pub_flag = [True, True, True, True]

        self.python_version = platform.python_version()[0]
        self.hf_imu = None  # 串口对象初始化
        self.init_serial()

    def init_serial(self):
        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("串口打开成功")
            else:
                self.hf_imu.open()
                self.get_logger().info("成功打开串口")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            exit(1)

    def read_serial_data(self):
        try:
            if self.hf_imu is None or not self.hf_imu.isOpen():
                self.get_logger().warning("串口未打开，无法读取数据")
                return

            buff_count = self.hf_imu.in_waiting
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(buff_count):
                    self.handle_serial_data(buff_data[i])
        except Exception as e:
            self.get_logger().error(f"读取串口数据时发生错误: {e}")
            exit(1)

    def handle_serial_data(self, raw_data):
        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        else:
            self.buff[self.key] = raw_data

        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return
        if self.key < 11:
            return
        else:
            data_buff = list(self.buff.values())
            if self.buff[1] == 0x51 and self.pub_flag[0]:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)]
                else:
                    self.get_logger().warning('0x51 校验失败')
                self.pub_flag[0] = False
            elif self.buff[1] == 0x52 and self.pub_flag[1]:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angular_velocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
                else:
                    self.get_logger().warning('0x52 校验失败')
                self.pub_flag[1] = False
            elif self.buff[1] == 0x53 and self.pub_flag[2]:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)]
                else:
                    self.get_logger().warning('0x53 校验失败')
                self.pub_flag[2] = False
            elif self.buff[1] == 0x54 and self.pub_flag[3]:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    # self.magnetometer = self.hex_to_short(data_buff[2:10])
                    self.magnetometer = [float(value) for value in self.hex_to_short(data_buff[2:10])]
                else:
                    self.get_logger().warning('0x54 校验失败')
                self.pub_flag[3] = False
            else:
                self.get_logger().warning(f"未知数据类型: {self.buff[1]} 或数据错误")
                self.buff = {}
                self.key = 0
                return

            self.buff = {}
            self.key = 0
            if any(self.pub_flag):
                return
            self.pub_flag = [True, True, True, True]
            self.publish_imu_data()

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        mag_msg = MagneticField()
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = "imu_link"

        angle_radian = [angle * math.pi / 180 for angle in self.angle_degree]
        qua = quaternion_from_euler(*angle_radian)

        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]

        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]

        imu_msg.linear_acceleration.x = self.acceleration[0]
        imu_msg.linear_acceleration.y = self.acceleration[1]
        imu_msg.linear_acceleration.z = self.acceleration[2]

        mag_msg.magnetic_field.x = self.magnetometer[0]
        mag_msg.magnetic_field.y = self.magnetometer[1]
        mag_msg.magnetic_field.z = self.magnetometer[2]

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

    def close_serial(self):
        if self.hf_imu is not None and self.hf_imu.isOpen():
            self.hf_imu.close()
            self.get_logger().info("串口已关闭")

    @staticmethod
    def check_sum(list_data, check_data):
        return sum(list_data) & 0xff == check_data

    @staticmethod
    def hex_to_short(raw_data):
        return list(struct.unpack("hhhh", bytearray(raw_data)))


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU 节点退出")
    finally:
        node.close_serial()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
