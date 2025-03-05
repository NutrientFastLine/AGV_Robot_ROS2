import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time

class TfFrequencyChecker(Node):
    def __init__(self):
        super().__init__('tf_frequency_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_time = None
        self.timer = self.create_timer(1.0, self.check_tf_frequency)  # 每秒检查一次

    def check_tf_frequency(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'laser_link', rclpy.time.Time())
            if self.last_time:
                dt = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9 - self.last_time
                self.get_logger().info(f'Transform Frequency: {1/dt:.2f} Hz')
            self.last_time = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().warn(f'No transform available yet: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TfFrequencyChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
