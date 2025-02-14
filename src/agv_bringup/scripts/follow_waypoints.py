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

    # Inspection route (positions and rotations)
    inspection_route_pose = [
        [0.8829003548629211, -0.6716124160695748],  # First position
        [0.8645972603560618, -2.5498842201335457],  # First turn preparation
        [0.6655089471574702, -3.2070154855470903],  # First turn start
        [0.7245409423986262, -2.498352645148587],  # First turn complete
        [0.9332443871356275, -1.4992302965008208],  # Second turn preparation
        [0.8371992556458344, -0.0636335926482864],  # Second turn start
        [0.7140008402412348, -0.3181350647303341],  # Second turn complete
        [0.8829003548629211, -0.6716124160695748]   # Back to first position
    ]

    inspection_route_rotation = [
        [-0.7101589656309399, 0.7040413649310626],
        [-0.7140317555984329, 0.7001133136835921],
        [0.9812977353333209, -0.19249611588210203],
        [0.6761412825884306, 0.7367719904961588],
        [0.71559939845749, 0.6985109168275601],
        [0.9387719782805626, 0.34453907295863995],
        [-0.7703070980525857, 0.6376730939045525],
        [-0.7101589656309399, 0.7040413649310626]
    ]

    # Initialize pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for the navigation system to be active
    navigator.waitUntilNav2Active()

    # Create inspection points
    inspection_points = []
    for idx, pt in enumerate(inspection_route_pose):
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()

        # Set position
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]

        # Set rotation
        rotation = inspection_route_rotation[idx]
        inspection_pose.pose.orientation.z = rotation[0]
        inspection_pose.pose.orientation.w = rotation[1]

        # Add deep copy of the object
        inspection_points.append(deepcopy(inspection_pose))

    # Repeat the route until user interrupts
    while rclpy.ok():
        # Send inspection points
        navigator.followWaypoints(inspection_points)

        # Do something during the route (e.g., analyze stock info or upload to cloud)
        # Print the current waypoint ID for the demonstration
        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(inspection_points)}')

        # Get result and handle different task outcomes
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Inspection complete! Returning to start...')
        elif result == TaskResult.CANCELED:
            print('Inspection was canceled. Returning to start...')
            break  # Exit the loop if the task is canceled
        elif result == TaskResult.FAILED:
            print('Inspection failed! Returning to start...')
            break  # Exit the loop if the task fails

        # Go back to the start position
        # print('Returning to start...')
        # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        # navigator.goToPose(initial_pose)
        
        while not navigator.isTaskComplete():
            pass  # Wait for navigation to complete

        # Optionally, add a delay before repeating
        time.sleep(2)  # Wait 2 seconds before restarting the loop

    # Shutdown ROS when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
