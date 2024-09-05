#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import math
import platform
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))


# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# 主节点类
class ImuNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.imu_msg = Imu()
        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        self.timer = self.create_timer(0.05, self.read_serial_data)

        self.key = 0
        self.buff = {}
        self.angularVelocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.angle_degree = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.pub_flag = [True, True, True, True]

        self.python_version = platform.python_version()[0]

        try:
            self.hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("\033[32m串口打开成功...\033[0m")
            else:
                self.hf_imu.open()
                self.get_logger().info("\033[32m打开串口成功...\033[0m")
        except Exception as e:
            self.get_logger().error("\033[31m串口打开失败\033[0m")
            raise e

    def read_serial_data(self):
        try:
            buff_count = self.hf_imu.in_waiting
        except Exception as e:
            self.get_logger().error(f"exception: {e}")
            self.get_logger().error("imu 失去连接，接触不良，或断线")
            rclpy.shutdown()
        else:
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(0, buff_count):
                    self.handle_serial_data(buff_data[i])

    def handle_serial_data(self, raw_data):
        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        if self.python_version == '3':
            self.buff[self.key] = raw_data

        # self.get_logger().info("Buffer V: {}".format(self.buff[self.key]))
        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return
        if self.key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
            return
        else:
            # self.get_logger().info("Buffer V: {}".format(self.buff[0]))
            data_buff = list(self.buff.values())  # 获取字典所有 value

            if self.buff[1] == 0x51 and self.pub_flag[0]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                else:
                    self.get_logger().warn('0x51 校验失败')
                self.pub_flag[0] = False

            elif self.buff[1] == 0x52 and self.pub_flag[1]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
                else:
                    self.get_logger().warn('0x52 校验失败')
                self.pub_flag[1] = False

            elif self.buff[1] == 0x53 and self.pub_flag[2]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                else:
                    self.get_logger().warn('0x53 校验失败')
                self.pub_flag[2] = False

            elif self.buff[1] == 0x54 and self.pub_flag[3]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = hex_to_short(data_buff[2:10])
                else:
                    print('0x54 校验失败')
                self.pub_flag[3] = False

            else:
                self.get_logger().warn(f"该数据处理类没有提供该 {str(self.buff[1])} 的解析或数据错误")
                self.buff = {}
                self.key = 0

            self.buff = {}
            self.key = 0
            if any(self.pub_flag):
                return
            self.pub_flag[0] = self.pub_flag[1] = self.pub_flag[2] = self.pub_flag[3] = True
            stamp = self.get_clock().now().to_msg()

            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = "imu_link"

            angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            self.imu_msg.orientation.x = qua[0]
            self.imu_msg.orientation.y = qua[1]
            self.imu_msg.orientation.z = qua[2]
            self.imu_msg.orientation.w = qua[3]

            self.imu_msg.angular_velocity.x = self.angularVelocity[0]
            self.imu_msg.angular_velocity.y = self.angularVelocity[1]
            self.imu_msg.angular_velocity.z = self.angularVelocity[2]

            self.imu_msg.linear_acceleration.x = self.acceleration[0]
            self.imu_msg.linear_acceleration.y = self.acceleration[1]
            self.imu_msg.linear_acceleration.z = self.acceleration[2]

            self.imu_pub.publish(self.imu_msg)


def main(args=None):
    rclpy.init(args=args)
    find_ttyUSB()
    imu_node = ImuNode("imu_driver_node")
    rclpy.spin(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()


# if __name__ == "__main__":
#     main()
