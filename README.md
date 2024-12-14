# 2D Dynamic Object Grasping

## Overview

This repository focuses on **2D dynamic object grasping**, which involves detecting, tracking, and grasping objects in motion. It is designed to address challenges in robotics, computer vision, or machine learning applications.

## Features

- **Dynamic Object Detection**: Identify objects in a 2D space.
- **Motion Tracking**: Track moving objects in real-time.
- **Grasp Planning**: Compute optimal grasp points for dynamic objects.
- **Integration with Robotics Frameworks**: Compatible with frameworks like ROS (if applicable).

## Installation

Follow these steps to set up the project:

1. Clone the repository:
   ```bash
   git clone https://github.com/adityavkulkarni/2D_dynamic_object_grasping.git
   cd 2D_dynamic_object_grasping```
2. Move the contents of src.zip to your_workspace/src and make the workspace:
   ```bash
   catkin_make``
3. Run the below commands in 3 separate terminals:
   ```bash
   roslaunch fetch_gazebo simple_grasp.launch
   roslaunch fetch_moveit_config move_group.launch
   python3 grasp.py```
