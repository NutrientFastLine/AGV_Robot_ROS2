#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile
import math
from rclpy.duration import Duration

"""
Basic navigation to follow a given path
"""


def read_file(file_path):
    with open(file_path, 'r') as in_file:
        route = Path()
        route.header.frame_id = "map"

        while True:
            line = in_file.readline()
            if not line:
                break

            if line.strip() == "EOP" and len(route.poses) > 0:
                print("Route got %d poses" % len(route.poses))
                break

            tokens = line.split()
            if len(tokens) != 3:
                continue

            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = float(tokens[0])
            p.pose.position.y = float(tokens[1])
            angle = float(tokens[2])
            p.pose.orientation.z = math.sin(angle / 2.0)
            p.pose.orientation.w = math.cos(angle / 2.0)
            route.poses.append(p)
    return route


def main():
    rclpy.init()
    node = rclpy.create_node('follow_path')
    record_path_pub_ = node.create_publisher(Path, 'followpath', QoSProfile(depth=10))
    node.declare_parameter('pathfilename', "/home/wheeltec/AGV_Robot_ROS2/src/wheeltec_path_follow/path/wheeltec_path")
    pathfilename = node.get_parameter('pathfilename').get_parameter_value().string_value
    pathfilename = "/home/wheeltec/AGV_Robot_ROS2/src/wheeltec_path_follow/path/wheeltec_path"
    node.declare_parameter('run_in_loop', True)
    run_in_loop = node.get_parameter('run_in_loop').get_parameter_value().bool_value
    navigator = BasicNavigator()
    #navigator.waitUntilNav2Active()
    followpath = Path()
    followpath = read_file(pathfilename)
    record_path_pub_.publish(followpath)

    # # 检查并打印路径点
    # if followpath.poses:
    #     print("Loaded path:")
    #     for i, pose in enumerate(followpath.poses):
    #         print(f"Point {i}: x={pose.pose.position.x}, y={pose.pose.position.y}, theta={2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)}")

    #     record_path_pub_.publish(followpath)
    # else:
    #     print("No valid poses found in the path file!")
    while rclpy.ok():
        record_path_pub_.publish(followpath)
        # Go to our demos first goal pose

        goal_pose = PoseStamped()
        goal_pose = followpath.poses[0]
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        navigator.goToPose(goal_pose)
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()
        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached the first waypoint, now following the full path...')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # Follow path
        navigator.followPath(followpath)
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated distance remaining to goal position: ' +
                      '{0:.3f}'.format(feedback.distance_to_goal) +
                      '\nCurrent speed of the robot: ' +
                      '{0:.3f}'.format(feedback.speed))
        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Path followed successfully!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
       
        #navigator.lifecycleShutdown()
        if run_in_loop:
            continue
        else:
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
