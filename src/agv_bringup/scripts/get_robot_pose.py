import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion


class TFListener(Node):

    def __init__(self):
        super().__init__('tf2_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1, self.get_transform)
        self.has_map_frame = False  # 默认假设没有map坐标系

    def get_transform(self):
        try:
            # 获取 'odom' 到 'base_link' 的变换
            tf_odom_base = self.buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform_odom_base = tf_odom_base.transform
            rotation_euler_odom_base = euler_from_quaternion([
                transform_odom_base.rotation.x,
                transform_odom_base.rotation.y,
                transform_odom_base.rotation.z,
                transform_odom_base.rotation.w
            ])
            self.get_logger().info('**************************************')
            self.get_logger().info(f'odom到base_link的平移:{transform_odom_base.translation}, '
                                   f'旋转四元数:{transform_odom_base.rotation}, '
                                   f'旋转欧拉角:{rotation_euler_odom_base}')

            # 如果有 map 坐标系，则获取 'map' 到 'odom' 的变换
            if self.has_map_frame:
                tf_map_base = self.buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform_map_base = tf_map_base.transform
                rotation_euler_map_base = euler_from_quaternion([
                    transform_map_base.rotation.x,
                    transform_map_base.rotation.y,
                    transform_map_base.rotation.z,
                    transform_map_base.rotation.w
                ])
                self.get_logger().info(f'map到base_link的平移:{transform_map_base.translation}, '
                                       f'旋转四元数:{transform_map_base.rotation}, '
                                       f'旋转欧拉角:{rotation_euler_map_base}')

                # 对比坐标变换之间的差异
                translation_diff = (
                    transform_map_base.translation.x - transform_odom_base.translation.x,
                    transform_map_base.translation.y - transform_odom_base.translation.y,
                    transform_map_base.translation.z - transform_odom_base.translation.z
                )

                rotation_diff = (
                    rotation_euler_map_base[0] - rotation_euler_odom_base[0],
                    rotation_euler_map_base[1] - rotation_euler_odom_base[1],
                    rotation_euler_map_base[2] - rotation_euler_odom_base[2]
                )
                self.get_logger().info(f'平移差异 (map - odom): {translation_diff}')
                self.get_logger().info(f'旋转差异 (map - odom): {rotation_diff}')
        except Exception as e:
            self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')

    def set_map_frame_status(self, status: bool):
        """ 设置是否存在 map 坐标系 """
        self.has_map_frame = status


def main():
    rclpy.init()
    node = TFListener()

    # 假设某个地方判断是否存在 'map' 坐标系并设置状态
    # 这里模拟，如果 'map' 坐标系存在，则调用 set_map_frame_status(True)
    # 你可以根据实际情况通过回调或其他逻辑来判断是否有 'map' 坐标系
    node.set_map_frame_status(True)  # 设置为 True 表示存在 map 坐标系

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
