
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <modbus.h>
#include <cmath>

using namespace std;

class DS2024_motor_driver : public rclcpp::Node
{
public:

    DS2024_motor_driver(string name);
    void enableMotor();                                                   // 轴使能函数
    void setRpm(int16_t rpm_left, int16_t rpm_right);
    void poseUpdate();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg); // 速度订阅函数

private:

    // string  serial_name = "/dev/tiewoniu_base";
    const int32_t L_ENABLE = 0x3100;      // 左轮使能地址；
    const int32_t R_ENABLE = 0x2100;      // 右轮使能地址；
    const int32_t L_CMD_RPM = 0x3318;     // 左轮速度地址
    const int32_t R_CMD_RPM = 0x2318;     // 右轮速度地址
    const int32_t L_ENCODER_RPM = 0x5104; // 左轮编码器地址
    const int32_t R_ENCODER_RPM = 0x5004; // 右轮编码器地址
    const int16_t ENABLE = 1;

    float base = 0.358;                // 两车轮之间的距离
    float rps_2_rpm = 9.54929658551;  //(60/(2*pi))
    float wheel = 0.0845;             // 车轮半径
    double param = 0.000094808778296; // 每走1个脉冲信号车辆行走0.000094808778295m
    uint16_t Last_L = 0;              // 机器人初始位姿定义,xy坐标，angle角度
    uint16_t Last_R = 0;
    double x_odom = 0.0, y_odom = 0.0, th_odom = 0.0; // x方向的里程计、y方向的里程计，旋转角度
    double vx = 0.0, vy = 0.0, vth = 0.0, dt = 0.0;   // 记录车辆的实时速度(vx为线速度，vth为角速度)

    modbus_t *modbus_ctx_;
    rclcpp::Time current_time; // 记录时间
    rclcpp::Time last_time;
    geometry_msgs::msg::Twist vel;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped transformStamped;

    tf2_ros::TransformBroadcaster br; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdVel;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    int16_t int16DecToHex(int16_t data) // 将10进制数转换为16进制数，并分为高低位存储；
    {
        uint8_t low_byte = (data & 0x00FF);
        uint8_t high_byte = (data & 0xFF00) >> 8;
        int16_t all_bytes = (high_byte << 8) | low_byte;
        return all_bytes;
    }

    void limitCheck(int16_t &mach)
    {
        if (mach < 0 && abs(mach) > 5000)
        {
            mach = 5600 + mach;
        }
        else if (mach > 0 && abs(mach) > 5000)
        {
            mach = mach - 5600;
        }
    }

};

DS2024_motor_driver::DS2024_motor_driver(string name) : Node(name), last_time(this->get_clock()->now()),br(this)
{
    RCLCPP_INFO(this->get_logger(), "电机驱动正在连接...");
    int device_ID = 1;
    modbus_ctx_ = modbus_new_rtu("/dev/ttyTHS1", 115200, 'N', 8, 1);
    if (modbus_ctx_ == NULL)
    {
        RCLCPP_FATAL(this->get_logger(), "无法创建libmodbus上下文 ...");
        return;
    }
    modbus_set_slave(modbus_ctx_, device_ID);
    if (modbus_connect(modbus_ctx_) == -1)
    {
        RCLCPP_FATAL(this->get_logger(), "连接失败: %s\n", modbus_strerror(errno));
        if (errno == EACCES) {
        RCLCPP_FATAL(this->get_logger(), "权限问题，无法访问设备。");
        }
        else if (errno == ENOENT) {
        RCLCPP_FATAL(this->get_logger(), "设备节点不存在。");
        }   
        modbus_free(modbus_ctx_);
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "电机驱动连接成功！");
    }

    sub_cmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&DS2024_motor_driver::cmdVelCallback, this, std::placeholders::_1));

    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);


}

void DS2024_motor_driver::enableMotor()
{
    if (modbus_write_register(modbus_ctx_, L_ENABLE, ENABLE) != 1)
    {
        RCLCPP_FATAL(this->get_logger(), "左轴使能失败！");
    }
    if (modbus_write_register(modbus_ctx_, R_ENABLE, ENABLE) != 1)
    {
        RCLCPP_FATAL(this->get_logger(), "右轴使能失败！");
    }
}

void DS2024_motor_driver::poseUpdate()
{
    uint16_t tab_reg_L[1];
    modbus_read_registers(modbus_ctx_, L_ENCODER_RPM, 1, tab_reg_L);
    uint16_t tab_reg_R[1];
    modbus_read_registers(modbus_ctx_, R_ENCODER_RPM, 1, tab_reg_R);

    int16_t R_M = (tab_reg_R[0] - Last_R);
    int16_t L_M = -(tab_reg_L[0] - Last_L);
    limitCheck(R_M);
    limitCheck(L_M);

    double d_left = L_M * param;
    double d_right = R_M * param;
    double dia_distance = (d_left + d_right) / 2.0; // 轮距中心行使距离
    double angle = -((d_left - d_right) / base);
    double delta_x = dia_distance * cos(angle);
    double delta_y = -dia_distance * sin(angle);
    x_odom += cos(th_odom) * delta_x - sin(th_odom) * delta_y;
    y_odom += sin(th_odom) * delta_x + cos(th_odom) * delta_y;
    th_odom += angle;

    Last_L = tab_reg_L[0];
    Last_R = tab_reg_R[0];
    current_time = this->get_clock()->now();
    dt = (current_time - last_time).seconds();
    last_time = current_time;

    vx = dia_distance / dt;
    vth = angle / dt;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th_odom); // 航向角转化为四元数；
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x_odom;
    odom.pose.pose.position.y = y_odom;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    odom.child_frame_id = "base_link"; // this->robot_frame_id;
    odom.twist.twist.linear.x = vx;     // this->vx;线速度
    odom.twist.twist.linear.y = 0.0;    // this->vy;
    odom.twist.twist.angular.z = vth;   // this->vth;//角速度

    if (vx == 0 && vth == 0)
    {
        odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9};
        odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
                                 0, 1e-3, 1e-9, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e-9};
    }
    else
    {
        odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3};
        odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
                                 0, 1e-3, 0, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e3};
    }
    pub_odom->publish(odom);

    // 发布在rviz下显示的坐标
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = x_odom;
    transformStamped.transform.translation.y = y_odom;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = odom_quat.x();
    transformStamped.transform.rotation.y = odom_quat.y();
    transformStamped.transform.rotation.z = odom_quat.z();
    transformStamped.transform.rotation.w = odom_quat.w();
    br.sendTransform(transformStamped);
}

void DS2024_motor_driver::setRpm(int16_t rpm_left, int16_t rpm_right)
{
    int16_t rpm[2] = {rpm_left, rpm_right};
    if (rpm[0] > 6000)
    {
        rpm[0] = 6000;
    }
    else if (rpm[0] < -6000)
    {
        rpm[0] = -6000;
    }

    if (rpm[1] > 6000)
    {
        rpm[1] = 6000;
    }
    else if (rpm[1] < -6000)
    {
        rpm[1] = -6000;
    }
    uint16_t rpm_command[2];
    rpm_command[0] = int16DecToHex(rpm[0]);
    rpm_command[1] = int16DecToHex(rpm[1]);
    modbus_write_register(modbus_ctx_, R_CMD_RPM, rpm_command[1]);
    modbus_write_register(modbus_ctx_, L_CMD_RPM, rpm_command[0]);
}

void DS2024_motor_driver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vel.linear.x = -msg->linear.x;
    vel.angular.z = -msg->angular.z;
    int command_left = (vel.linear.x - (vel.angular.z * base / 2)) / wheel * rps_2_rpm;
    int command_right = (vel.linear.x + (vel.angular.z * base / 2)) / wheel * rps_2_rpm; // RPS-> RPM (1/(2*pi))*60
    setRpm(-command_left, command_right);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DS2024_motor_driver>("DS2024_motor_driver_node");

    node->enableMotor();
    rclcpp::Rate loop_rate(50ms);
    while (rclcpp::ok())
    {
        node->poseUpdate();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
