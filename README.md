# ROS_AutoTaxi

ROS_AutoTaxi is a self-driving car project developed using ROS Noetic. It simulates an autonomous taxi that performs various tasks such as autonomous navigation, sign detection, traffic light recognition, and lane tracking. The simulation uses the Prius model as the base vehicle.

## Features

- Autonomous navigation: The self-driving car is capable of navigating through the environment autonomously using sensor data and path planning algorithms.
- Sign detection: The car uses image processing techniques to detect and recognize traffic signs, allowing it to respond accordingly.
- Traffic light recognition: The car employs computer vision algorithms to detect and interpret traffic lights, enabling it to make appropriate driving decisions.
- Lane tracking: The car utilizes sensor data to perform lane tracking and maintain its position within the designated lanes.

## Dependencies

This project relies on the following ROS packages and dependencies:

- message_runtime
- std_msgs
- message_generation
- tf2_ros
- sensor_msgs
- prius_msgs
- prius_description
- fake_localization
- gazebo_ros
- roscpp
- rospy
- teb_local_planner
- robot_state_publisher
- pointcloud_to_laserscan
