#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class LaserTfPublisher : public rclcpp::Node {
public:
    LaserTfPublisher() : Node("laser_tf_sync") {
        // 订阅 /scan 话题
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LaserTfPublisher::scanCallback, this, std::placeholders::_1)
        );

        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        geometry_msgs::msg::TransformStamped transformStamped;

        // **🔹 关键：使用 /scan 的时间戳，确保同步**
        transformStamped.header.stamp = scan_msg->header.stamp;
        transformStamped.header.frame_id = "base_link";  // 机器人主体
        transformStamped.child_frame_id = "scan";        // 激光雷达坐标系

        // **🔹 设置激光雷达相对 base_link 的安装位置**
        transformStamped.transform.translation.x = 0.0;  // 例如：前方 0.2m
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.15;  // 例如：高 0.1m

        // **🔹 设置激光雷达的姿态（假设无旋转）**
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 1.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 0.0;

        // **🔹 发布 TF**
        tf_broadcaster_->sendTransform(transformStamped);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
