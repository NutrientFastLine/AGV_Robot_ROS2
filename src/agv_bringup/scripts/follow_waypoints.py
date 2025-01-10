#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
import time
from copy import deepcopy
 
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
 
 
def main():
    rclpy.init()
 
    navigator = BasicNavigator()
 
    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # 位置信息
    inspection_route_pose = [
        [0.4, -0.0],
        [0.7, -0.4],
        [0.689, -1.606],
        [0.301, -2.712],
        [0.657, -2.376],
        [0.909, -1.240],
        [0.305, -0.157]
    ]
    # 旋转信息
    inspection_route_rotation = [
        [-0.0, 1.0],
        [-0.707, 0.707],
        [-0.781, 0.625],
        [0.917, -0.4],
        [0.726, 0.687],
        [0.707, 0.707],        
        [0.997, 0.024]
    ]

    # 初始化位姿
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # 等待导航系统激活
    navigator.waitUntilNav2Active()

    # 创建检查点
    inspection_points = []
    for idx, pt in enumerate(inspection_route_pose):
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        # 设置位置
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        
        # 设置旋转
        rotation = inspection_route_rotation[idx]
        inspection_pose.pose.orientation.z = rotation[0]
        inspection_pose.pose.orientation.w = rotation[1]
        
        # 添加深拷贝的对象
        inspection_points.append(deepcopy(inspection_pose))

    # 发送检查点
    navigator.followWaypoints(inspection_points)
 
    # Do something during your route (e.x. AI to analyze stock information or upload to the cloud)
    # Print the current waypoint ID for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
 
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
        exit(1)
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')
 
    # go back to start
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass
 
    exit(0)
 
 
if __name__ == '__main__':
    main()