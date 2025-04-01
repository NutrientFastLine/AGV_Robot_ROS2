#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"

#include "tf2_ros/buffer.h"
#include <tf2/convert.h>
#include <nav_msgs/msg/path.hpp>
#include <fstream>
#include "tf2/utils.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//路径保存中，相邻两点的间隔
double RECORD_PATH_LEN_DENS = 0.05;
double RECORD_PATH_AGU_DENS = 10 * M_PI / 180;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("save_path");

  std::string pathfilename;
  node->declare_parameter<std::string>("pathfilename", "/home/wheeltec/AGV_Robot_ROS2/src/wheeltec_path_follow/path/wheeltec_path");
  node->get_parameter("pathfilename", pathfilename);
  pathfilename = "/home/wheeltec/AGV_Robot_ROS2/src/wheeltec_path_follow/path/wheeltec_path";
  auto pub_ = node->create_publisher<nav_msgs::msg::Path>("followpath", 10);

  geometry_msgs::msg::TransformStamped base_footprint_transform;
  tf2_ros::Buffer tf_buffer_(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  nav_msgs::msg::Path paths;
  paths.header.frame_id = "map";
  paths.header.stamp = rclcpp::Time();

  while(rclcpp::ok())
  {
    try
    {
        base_footprint_transform = tf_buffer_.lookupTransform(
            "map", "base_link",
            tf2::TimePoint());
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_INFO(node->get_logger(), "%s to %stransform lookup waiting", "map", "base_link");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
    }
    geometry_msgs::msg::PoseStamped carpose;
    carpose.pose.position.x = base_footprint_transform.transform.translation.x;
    carpose.pose.position.y = base_footprint_transform.transform.translation.y;
    carpose.pose.orientation = base_footprint_transform.transform.rotation;
    if (paths.poses.empty()) 
    {
    	paths.poses.push_back(carpose);
    	continue;
    }
    auto len = std::hypot(paths.poses.back().pose.position.x - carpose.pose.position.x,
                        paths.poses.back().pose.position.y - carpose.pose.position.y);
    auto agu = std::abs(tf2::getYaw(paths.poses.back().pose.orientation) - tf2::getYaw(carpose.pose.orientation));
    if (len < RECORD_PATH_LEN_DENS && agu < RECORD_PATH_AGU_DENS) continue;
    paths.poses.push_back(carpose);
    pub_->publish(paths);
  }

  RCLCPP_INFO(node->get_logger(),"Save path file %s now , waiting!", pathfilename.c_str());
  std::ofstream out(pathfilename,std::ios_base::trunc);
  if (!out.is_open()) {
    RCLCPP_ERROR(node->get_logger(),"Open file %s failed!", pathfilename.c_str());
    return false;
  }
  for (auto const& p : paths.poses) {
    out << std::to_string(p.pose.position.x) << " "<< std::to_string(p.pose.position.y) << " "<< std::to_string(tf2::getYaw(p.pose.orientation)) << "\n";
  }
  out << "EOP" << "\n";  
  out.close();
  RCLCPP_INFO(node->get_logger(),"Save path file %s succeed!", pathfilename.c_str());

}
