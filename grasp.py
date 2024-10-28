#!/usr/bin/env python

"""
CS 6301 Homework 4 Programming
Robot Control for Grasping
"""

import sys
import time
import rospy
import roslib
import tf
import numpy as np
import moveit_commander
import actionlib
import threading
import pandas as pd
import random

from transforms3d.quaternions import mat2quat, quat2mat
from geometry_msgs.msg import PoseStamped
from trac_ik_python.trac_ik import IK
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

roslib.load_manifest('gazebo_msgs')
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def get_gazebo_timestamp():
    rospy.wait_for_message('/clock', Clock)
    
    clock_msg = rospy.wait_for_message('/clock', Clock)
    
    return clock_msg.clock


def random_float(start=-0.4, end=0.4, multiple=0.01):
    # Calculate the number of multiples
    range_start = int(start / multiple)
    range_end = int(end / multiple)
    # Generate a list of possible multiples and select one randomly
    return random.choice([i * multiple for i in range(range_start, range_end + 1)])


def ros_quat(tf_quat): #wxyz -> xyzw
    quat = np.zeros(4)
    quat[-1] = tf_quat[0]
    quat[:-1] = tf_quat[1:]
    return quat
    
    
# rotation matrix about Y axis
def rotY(roty):
    RotY = np.array(
        [
            [np.cos(roty), 0, np.sin(roty), 0],
            [0, 1, 0, 0],
            [-np.sin(roty), 0, np.cos(roty), 0],
            [0, 0, 0, 1],
        ]
    )
    return RotY


# Convert quaternion and translation to a 4x4 tranformation matrix
# See Appendix B.3 in Lynch and Park, Modern Robotics for the definition of quaternion
def ros_qt_to_rt(rot, trans):
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans

    return obj_T


# Convert a ROS pose message to a 4x4 tranformation matrix
def ros_pose_to_rt(pose):
    qarray = [0, 0, 0, 0]
    qarray[0] = pose.orientation.x
    qarray[1] = pose.orientation.y
    qarray[2] = pose.orientation.z
    qarray[3] = pose.orientation.w

    t = [0, 0, 0]
    t[0] = pose.position.x
    t[1] = pose.position.y
    t[2] = pose.position.z

    return ros_qt_to_rt(qarray, t)


# Query pose of frames from the Gazebo environment
def get_pose_gazebo(model_name, relative_entity_name=''):

    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms(model_name, relative_entity_name)
            return resp1
        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)
    
    # query the object pose in Gazebo world T_wo
    res = gms_client(model_name, relative_entity_name) 
    T_wo = ros_pose_to_rt(res.pose)  
    box_pose = res.pose
    
    # query fetch base link pose in Gazebo world T_wb
    res = gms_client(model_name='fetch', relative_entity_name='base_link')
    T_wb = ros_pose_to_rt(res.pose)
    fetch_pose = res.pose

    
    ################ TO DO ##########################
    # compute the object pose in robot base link T_bo
    # use your code from homework 2
    T_bw = np.linalg.inv(T_wb) 
    # Compute T_bo = T_wb.T_wo
    T_bo = np.dot(T_bw, T_wo)
    ################ TO DO ##########################
    return T_bo, fetch_pose, box_pose
    

# publish tf for visualization
def publish_tf(trans, qt, model_name):

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(trans, qt, rospy.Time.now(), model_name, 'base_link')
        rate.sleep()
    
    
# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()    


def set_cube_random(box_pose):
    box_pose.position.y = random_float()
    set_model_pose("demo_cube", box_pose)


def set_model_pose(model_name, pose):
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # Create a ModelState object
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = pose
        # Call the service
        response = set_state(model_state)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def get_track_ik_solution(seed_state, trans, rotated_qt):
    retry = 30    
    sol = None
    while sol is None:
        # multithread and random state
        sol = ik_solver.get_ik(seed_state,
                        trans[0], trans[1], trans[2],
                        rotated_qt[0], rotated_qt[1], rotated_qt[2], rotated_qt[3])
        """rospy.loginfo('Solution from IK:')
        print(ik_solver.joint_names)                
        print(sol)"""
        if sol: break
        retry -= 1    
    return sol


def reset_objects():
    scene.clear()
    set_model_pose("demo_cube", box_pose)
    set_model_pose("fetch", fetch_pose)
    # group.go(joints, wait=True)
    # gripper.open()
    group.stop()


if __name__ == "__main__":
    """
    Main function to run the code
    """
    # intialize ros node
    rospy.init_node('planning_scene_block')
    rospy.set_param('/use_sim_time', True)

    # query the demo cube pose
    model_name = 'demo_cube'
    T, fetch_pose, box_pose = get_pose_gazebo(model_name)
    
    # translation
    trans = T[:3, 3]
    # quaternion in ros
    qt = ros_quat(mat2quat(T[:3, :3]))
    
    # publish the cube tf for visualization
    x = threading.Thread(target=publish_tf, args=(trans, qt, model_name))
    x.start()

    # # Setup clients
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso")
    torso_action.move_to([0.4, ])
    
    # --------- initialize moveit components ------
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander('arm')
    group.set_max_velocity_scaling_factor(0.8)
    group.set_max_acceleration_scaling_factor(0.8)
    
    gripper_group = moveit_commander.MoveGroupCommander('gripper')
    gripper_group.set_max_velocity_scaling_factor(1)
    gripper_group.set_max_acceleration_scaling_factor(1)

    pos_close = [0.02, 0.02]
    gripper_group.set_joint_value_target(pos_close)
    gripper_close_plan = gripper_group.plan()

    pos_open = [0.05, 0.05]
    gripper_group.set_joint_value_target(pos_open)
    gripper_open_plan = gripper_group.plan()

    gripper_group.execute(gripper_open_plan[1].joint_trajectory)
    gripper_group.stop()

    # planning scene
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    robot = moveit_commander.RobotCommander()
    
    # add objects into the planning scene
    rospy.sleep(1.0)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()    
    # add a box for robot base to avoid hitting the base
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.18
    scene.add_box("base", p, (0.56, 0.56, 0.4))
    
    # add a big box for the table
    p.pose.position.x = 0.9
    p.pose.position.y = 0
    p.pose.position.z = trans[2] - 0.06 / 2 - 0.51 - 0.2
    scene.add_box("table", p, (1, 5, 1))
    
    joints = group.get_current_joint_values()
        
    # define the IK solver from track_ik
    ik_solver = IK("base_link", "wrist_roll_link")
    
    # change the joint limit of torso_lift_joint in order to fix the torso lift
    lower_bound, upper_bound = ik_solver.get_joint_limits()
    lower_bound = list(lower_bound)
    upper_bound = list(upper_bound)
    lower_bound[0] = 0.4
    upper_bound[0] = 0.4
    ik_solver.set_joint_limits(lower_bound, upper_bound)

    # use initial seed as zeros
    seed_state = [0.0] * ik_solver.number_of_joints
    seed_state[0] = 0.4
    
    # Convert 90 degrees to radians
    angle = np.pi / -2

    # Create the rotation matrix for 90 degrees about the y-axis
    rotation_matrix = rotY(angle)

    # Apply the rotation to the transformation matrix T
    T_rotated = np.dot(rotation_matrix, T)

    # Extract the rotated quaternion
    rotated_qt = mat2quat(T_rotated[:3, :3])

    # Move directly initial pose
    trans_1 = [trans[0], trans[1], trans[2] + 0.5]
    sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)   
    joint_goal_init = sol1[1:]
    group.set_joint_value_target(joint_goal_init)
    plan = group.plan()
    trajectory = plan[1].joint_trajectory
    group.execute(plan[1].joint_trajectory)
    group.stop()
    
    results = []
    rospy.sleep(3.0)
    for i in range(2):
        set_cube_random(box_pose)
        T, fetch_pose, box_pose = get_pose_gazebo(model_name)
        trans = T[:3, 3]
        print(f"Iteration {i+1}: Cube postion = {trans}")

        ts_final = get_gazebo_timestamp()
        ts_sol1 = get_gazebo_timestamp()
        ts_sol_r1 = time.time()

        # Move directly above the cube
        trans_1 = [trans[0], trans[1], trans[2] + 0.5]
        sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        seed_state = sol1
        trans_1 = [trans[0], trans[1], trans[2] + 0.2]
        sol2 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        
        ts_sol2 = get_gazebo_timestamp()
        ts_sol_r2 = time.time()
    
        ts_move_1 = get_gazebo_timestamp()
        # Plan
        joint_goal = sol1[1:]
        group.set_joint_value_target(joint_goal)
        plan_1 = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_pose = trajectory.points[-1].time_from_start.to_sec()

        joint_goal = sol2[1:]
        group.set_joint_value_target(joint_goal)
        plan_2 = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_grasp = trajectory.points[-1].time_from_start.to_sec()

        # move to above the cube
        group.execute(plan_1[1].joint_trajectory)
        group.stop()

        # move to grasp the cube
        group.execute(plan_2[1].joint_trajectory)
        group.stop()
        
        ts_move_2 = get_gazebo_timestamp()

        gripper_group.set_joint_value_target(pos_close)
        gripper_open_plan = gripper_group.plan()
        gripper_group.execute(gripper_close_plan[1].joint_trajectory)
        gripper_group.stop()
        ts_grip = get_gazebo_timestamp()

        # Pick the cube
        seed_state = sol2
        trans_1 = [trans[0], trans[1], trans[2] + 0.5]
        sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        joint_goal = sol1[1:]
        group.set_joint_value_target(joint_goal)
        plan = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_pose_rev = trajectory.points[-1].time_from_start.to_sec()

        group.execute(plan[1].joint_trajectory)
        group.stop()
        ts_final2 = get_gazebo_timestamp()

        cur_T, cur_fetch_pose, cur_box_pose = get_pose_gazebo(model_name)
        grasp_status = 'SUCCESS' if round(cur_T[2][3] - T[2][3], 1) == 0.3 else 'FAIL'
        rospy.loginfo(f"Iteration: {i+1} Grasp status: {grasp_status}")
        results.append(
            {
                "iteration": i+1, 
                "grasp_status": grasp_status,
                "total_time": (ts_final2 - ts_final).to_sec(),
                "solution_time_sim": (ts_sol2 - ts_sol1).to_sec(),
                "solution_time_real": (ts_sol2 - ts_sol1).to_sec(),
                "grasp_pose_time": (ts_move_2 - ts_move_1).to_sec(),
                "grasp_pose_estimate": (estimated_duration_grasp + estimated_duration_pose),
                "offset": ((ts_move_2 - ts_move_1).to_sec() - (estimated_duration_grasp + estimated_duration_pose)),
                "grip_time": (ts_grip - ts_move_2).to_sec()
            }
            )
        time.sleep(5)
        reset_objects()
        group.set_joint_value_target(joint_goal_init)
        plan = group.plan()
        trajectory = plan[1].joint_trajectory
        group.execute(plan[1].joint_trajectory)
        group.stop()

        gripper_group.set_joint_value_target(pos_open)
        gripper_open_plan = gripper_group.plan()
        gripper_group.execute(gripper_open_plan[1].joint_trajectory)
        gripper_group.stop()
        time.sleep(2)
    
    for result in results:
        rospy.loginfo(f"RESULTS: ")
        print(f'Iteration: {result["iteration"]}')
        print(f'Grasp Status: {result["grasp_status"]}')
        print(f'Total Time: {result["total_time"]}')
        print(f'Solution calculation Time(sim): {result["solution_time_sim"]}')
        print(f'Solution calculation Time(real): {result["solution_time_real"]}')
        print(f'Pregrasp Movement Time: {result["grasp_pose_time"]}')
        print(f'Pregrasp Movement Estimate: {result["grasp_pose_estimate"]}')
        print(f'Pregrasp Movement Offset: {result["offset"]}')
        print(f'Gripping Time: {result["grip_time"]}')
    pd.DataFrame(results).to_csv("results.csv")
    rospy.signal_shutdown("")

