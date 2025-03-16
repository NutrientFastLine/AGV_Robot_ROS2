#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class LaserTfPublisher : public rclcpp::Node {
public:
    LaserTfPublisher() : Node("laser_tf_sync") {
        // **🔹 确保 QoS 与 /scan 话题匹配**
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // **订阅 /scan 话题**
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile,
            std::bind(&LaserTfPublisher::scanCallback, this, std::placeholders::_1)
        );

        // **发布 /scan_filter 话题**
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile
        );
        // **订阅 /base/odom 话题**
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/base/odom", qos_profile,
        //     std::bind(&LaserTfPublisher::odomCallback, this, std::placeholders::_1)
        // );

        // 初始化 TF 广播器
        // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // // **设置定时器，每 50ms（20Hz）调用一次 publishTransform**
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(10),  // 50ms = 20Hz
        //     std::bind(&LaserTfPublisher::publishTransform, this)
        // );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::TimerBase::SharedPtr timer_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // **🔹 获取当前系统时间**
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Time scan_time = scan_msg->header.stamp;

        scan_msg->header.stamp = current_time;
        scan_msg->header.frame_id = "laser_link";
        // **🔹 计算时间差（纳秒级）**
        int64_t time_diff_ns = (current_time - scan_time).nanoseconds();
        double time_diff_s = time_diff_ns * 1e-9;  // 转换为秒（双精度）

        // **🔹 输出时间差（精确到纳秒）**
        RCLCPP_INFO(this->get_logger(), "LaserScan time difference: %ld ns (%.9f s)", time_diff_ns, time_diff_s);

        // 发布修改后的消息
        // scan_pub_->publish(*scan_msg);

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // **🔹 获取当前系统时间**
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Time odom_time = odom_msg->header.stamp;

        // // **🔹 计算时间差（纳秒级）**
        // int64_t time_diff_ns = (current_time - odom_time).nanoseconds();
        // double time_diff_s = time_diff_ns * 1e-9;  // 转换为秒（双精度）

        // // **🔹 输出时间差（精确到纳秒）**
        // RCLCPP_INFO(this->get_logger(), "Odometry time difference: %ld ns (%.9f s)", time_diff_ns, time_diff_s);
    }

    // void publishTransform() {
    //     // **🔹 获取当前系统时间**
    //     rclcpp::Time current_time = this->get_clock()->now();

    //     geometry_msgs::msg::TransformStamped transformStamped;

    //     // **🔹 使用当前时间戳**
    //     transformStamped.header.stamp = current_time;
    //     transformStamped.header.frame_id = "base_link";  // 机器人主体
    //     transformStamped.child_frame_id = "laser_link";  // 激光雷达坐标系

    //     // **🔹 设置激光雷达相对 base_link 的安装位置**
    //     transformStamped.transform.translation.x = 0.0;  
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = 0.15;  

    //     // **🔹 设置激光雷达的姿态（假设无旋转）**
    //     transformStamped.transform.rotation.x = 0.0;
    //     transformStamped.transform.rotation.y = 1.0;
    //     transformStamped.transform.rotation.z = 0.0;
    //     transformStamped.transform.rotation.w = 0.0;

    //     // **🔹 发布 TF**
    //     tf_broadcaster_->sendTransform(transformStamped);

    //     // RCLCPP_INFO(this->get_logger(), "Published TF at: %.6f", current_time.seconds());
    // }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
