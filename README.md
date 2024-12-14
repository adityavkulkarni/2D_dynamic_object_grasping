# 2D Dynamic Object Grasping

---

## Execution Steps

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

---

## Overview

This project focuses on developing a control system for a manipulator robot to accurately grasp objects moving along deterministic paths. By predicting and adapting to their future poses, the system aims to enhance the efficiency and accuracy of robotic grasping tasks.

## Objectives

- Develop a robotic system capable of intercepting and grasping moving objects.
- Predict object trajectories using **Recursive Least Squares (RLS)**.
- Integrate with tools like **ROS**, **MoveIt**, and **TRAC-IK** for inverse kinematics and grasp execution.

## Applications

The project has practical implications in industries that rely on automated systems, such as:

- Conveyor belt sorting.
- Mobile manipulation tasks.

---

## Methodology

### Simulation Environment

The simulation environment includes:

- **Gazebo**: Used for simulating the robot and its environment.
- **Components**:
  - Fetch robot.
  - Conveyor belt or table setup.
  - Python scripts for simulating cube motion.

Two approaches were used to simulate the cube's uniform motion:
1. Using a Gazebo table setup with Python scripts to move the cube.
2. Using the Gazebo conveyor belt plugin with adjustable speed via ROS services.

### Challenges Encountered

- High-frequency vertical oscillations of the cube during horizontal motion simulation in Gazebo.
- Cube retaining velocity after being grasped, leading to inaccuracies in simulations.

---

### Approach and Methodology

#### Trajectory Prediction
- **Recursive Least Squares (RLS)**:
  - Predicts future positions of moving objects based on real-time data.
  - Continuously refines parameters to minimize prediction error:
    \[
    e(t) = y(t) - w^T x(t)
    \]
  - Updates predictions using a gain vector \( K(t) \), ensuring efficient adaptation.

#### Interception Algorithm
1. **Initialization and Prediction**:
   - Record current time and predict future positions using RLS until convergence.
2. **Time Calculation**:
   - Define timesteps and calculate the time needed for the gripper to reach the object.
3. **Trajectory Sampling and IK Solution**:
   - Randomly sample points from predicted trajectories and compute Inverse Kinematics (IK) solutions using multi-threading.
4. **Execution**:
   - Select the first feasible step, execute the IK solution, and deploy the gripper to grasp the object.

#### Trajectory Planning
- The process involves two main steps:
  1. Positioning the robot above the intercept point.
  2. Performing a vertical descent to grasp the object.
- Uses **MoveIt** for collision-free path generation and time estimation.

---

## Results

- The system achieved a grasp success rate of **80%** for objects moving along deterministic paths.
- Identified challenges include handling non-linear paths and varying velocities, which remain areas for future research.

---

## Improvements and Future Work

1. **Enhanced Perception**:
   - Improve detection and prediction capabilities in dynamic environments.
2. **Non-linear Path Exploration**:
   - Extend current methods to handle circular, sinusoidal, or other complex trajectories.
3. **Variable Velocity Handling**:
   - Incorporate algorithms that adapt to non-linear object velocities.
4. **Reinforcement Learning Integration**:
   - Use reinforcement learning techniques to improve adaptability in unpredictable environments.

---

## Tools and Technologies

- **ROS**: For robot control and simulation integration.
- **MoveIt**: For trajectory planning and collision avoidance.
- **TRAC-IK**: For efficient inverse kinematics calculations.
- **Gazebo**: Simulation environment with conveyor belt plugin or table setups.

---

## Contributors

This project was developed by Group 8 as part of CS 6301:

- Saurav Dosi
- Aditya Kulkarni
- Feroz Hatha
