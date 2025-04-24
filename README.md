# ROS2 Publisher-Subscriber Project

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)

A professional implementation of a ROS2 publisher-subscriber system done by Ashalen Govender demonstrating best practices in ROS2 Python development.

## Features
- Robust publisher node with configurable QoS
- Reliable subscriber node with logging
- Proper error handling and shutdown procedures
- Clean code structure with complete documentation
- Full colcon build system integration

## Installation
```bash
# Clone the repository
git clone https://github.com/Ashalen/ros2_pubsub_project.git
cd ros2_pubsub_project

# Build the package
colcon build

# Source the workspace
source install/setup.bash

## Usage
# Terminal 1 - Publisher
ros2 run pubsub_project publisher_node
# or
python3 src/pubsub_project/pubsub_project/publisher_node.py

# Terminal 2 - Subscriber
ros2 run pubsub_project subscriber_node
# or
python3 src/pubsub_project/pubsub_project/subscriber_node.py

# Terminal 3 - View topics
ros2 topic list
ros2 topic echo /greeting
# or
rqt_graph 
