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
 
# Shelf positions for picking
shelf_positions = {
    "shelf_A": [0.4, -0.0],
    "shelf_B": [0.6, -0.4],
    "shelf_C": [0.866, -0.806],
    "shelf_D": [0.829, -1.545]}

shelf_rotations = {
    "shelf_A": [-0.0, 1.0],
    "shelf_B": [-0.707, 0.707],
    "shelf_C": [-0.685, 0.728],
    "shelf_D": [-0.724, 0.690]}
 
# Shipping destination for picked products
shipping_positions = {
    "shelf_A": [0.4, -0.0],
    "shelf_B": [0.6, -0.4],
    "shelf_C": [0.866, -0.806],
    "shelf_D": [0.829, -1.545]}
shipping_rotations = {
    "shelf_A": [-0.0, 0.0],
    "shelf_B": [-0.707, 0.707],
    "shelf_C": [-0.685, 0.728],
    "shelf_D": [-0.724, 0.690]}
'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''
 
 
def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = 'shelf_A'
    request_destination = 'shelf_B'
    ####################
 
    rclpy.init()
 
    navigator = BasicNavigator()
 
    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = shelf_rotations[request_item_location][0]
    shelf_item_pose.pose.orientation.w = shelf_rotations[request_item_location][1]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)
    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
 
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_positions[request_destination][0]
        shipping_destination.pose.position.y = shipping_positions[request_destination][1]
        shipping_destination.pose.orientation.z = shipping_rotations[request_destination][0]
        shipping_destination.pose.orientation.w = shipping_rotations[request_destination][1]
        navigator.goToPose(shipping_destination)
 
    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)
 
    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)
 
    while not navigator.isTaskComplete():
        pass
 
    exit(0)
 
 
if __name__ == '__main__':
    main()