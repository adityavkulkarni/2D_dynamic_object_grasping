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
import logging
import pandas as pd

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
from gripper import Gripper



def get_gazebo_timestamp():
    rospy.wait_for_message('/clock', Clock)
    
    clock_msg = rospy.wait_for_message('/clock', Clock)
    
    return clock_msg.clock


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


def set_model_pose(model_name, pose):
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # Create a ModelState object
 

        model_state = ModelState()
        model_state.model_name = model_name
        """model_state.pose.position.x = 0.6
        model_state.pose.position.y = 0.1
        model_state.pose.position.z = 0.72433349956864

        model_state.pose.orientation.x = 2.6742690478262186e-15
        model_state.pose.orientation.y = -7.813298653223138e-16
        model_state.pose.orientation.z = -1.134997195467295e-15
        model_state.pose.orientation.w = 1"""
        model_state.pose = pose
        # Call the service
        response = set_state(model_state)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def get_track_ik_solution(seed_state, trans, rotated_qt):
    retry = 30    
    sol = None
    while sol is None:
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
    group.go(joints, wait=True)
    gripper.open()
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
    
    # gripper controller
    gripper = Gripper()
    gripper.open()
    
    # # Setup clients
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    
    # Raise the torso using just a controller
    rospy.loginfo("Raising torso")
    torso_action.move_to([0.4, ])
    
    # --------- initialize moveit components ------
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander('arm')
    group.set_max_velocity_scaling_factor(0.6)
    group.set_max_acceleration_scaling_factor(0.6)
    # planning scene
    scene = moveit_commander.PlanningSceneInterface()
    scene.clear()
    robot = moveit_commander.RobotCommander()
    
    # print information about the planner
    planning_frame = group.get_planning_frame()
    rospy.loginfo(f"Reference frame: {planning_frame}")

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    rospy.loginfo(f"End effector: {eef_link}")

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    rospy.loginfo(f"Robot Groups: {robot.get_group_names()}")

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    rospy.loginfo("Printing robot state")
    joint_state = robot.get_current_state().joint_state

    results = []

    for i in range(len(joint_state.name)):
        print(joint_state.name[i], joint_state.position[i])
    
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
    
    for i in range(10):
        # get the current joints
        joints = group.get_current_joint_values()
        # rospy.loginfo('current joint state of the robot')
        # print(group.get_active_joints())
        # print(joints)
        
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
        
        ################ TO DO ##########################
        # use the get_ik function from trac_ik to compute the joints of the robot for grasping the cube
        # return the solution to a "sol" variable
        # Refer to https://bitbucket.org/traclabs/trac_ik/src/master/trac_ik_python/
            # Convert 90 degrees to radians
        angle = np.pi / -2

        # Create the rotation matrix for 90 degrees about the y-axis
        rotation_matrix = rotY(angle)

        # Apply the rotation to the transformation matrix T
        T_rotated = np.dot(rotation_matrix, T)

        # Extract the rotated quaternion
        rotated_qt = mat2quat(T_rotated[:3, :3])

        # Move directly above the cube
        """retry = 30    
        sol = None
        while sol is None:
            sol = ik_solver.get_ik(seed_state,
                            trans[0], trans[1], trans[2] + 0.5,
                            rotated_qt[0], rotated_qt[1], rotated_qt[2], rotated_qt[3])
            rospy.loginfo('Solution from IK:')
            print(ik_solver.joint_names)                
            print(sol)
            if sol: break
            retry -= 1 """
        trans_1 = [trans[0], trans[1], trans[2] + 0.5]
        sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)   
        # move to the joint goal
        joint_goal = sol1[1:]

        # Retrieve estimated duration from the planned trajectory
        group.set_joint_value_target(joint_goal)
        plan = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_pose = trajectory.points[-1].time_from_start.to_sec()

        ts1 = get_gazebo_timestamp()
        group.execute(plan[1].joint_trajectory)
        group.stop()
        ts2 = get_gazebo_timestamp()

        """ts1 = get_gazebo_timestamp()
        group.go(joint_goal, wait=True)
        group.stop()
        ts2 = get_gazebo_timestamp()"""

        # rospy.loginfo("Moved directly above cube")
        
        rospy.sleep(3.0)

        # move down to contact the cube
        """
        retry = 30
        sol = None
        while sol is None:
            sol = ik_solver.get_ik(seed_state,
                            trans[0], trans[1], trans[2] + 0.2,
                            rotated_qt[0], rotated_qt[1], rotated_qt[2], rotated_qt[3])
            rospy.loginfo('Solution from IK:')
            print(ik_solver.joint_names)                
            print(sol)
            if sol: break
            retry -= 1"""
        seed_state = sol1
        trans_1 = [trans[0], trans[1], trans[2] + 0.2]
        sol2 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        ################ TO DO ##########################
        
        # move to the joint goal
        joint_goal = sol2[1:]
        # Retrieve estimated duration from the planned trajectory
        group.set_joint_value_target(joint_goal)
        plan = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_grasp = trajectory.points[-1].time_from_start.to_sec()
        
        ts3 = get_gazebo_timestamp()
        group.execute(plan[1].joint_trajectory)
        group.stop()
        ts4 = get_gazebo_timestamp()
        gripper.close()
        ts_gripper = get_gazebo_timestamp()

        """ts3 = get_gazebo_timestamp()

        group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()
        # close gripper
        gripper.close()
        ts4 = get_gazebo_timestamp()"""
        seed_state = sol2
        trans_1 = [trans[0], trans[1], trans[2] + 0.5]
        sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        joint_goal = sol1[1:]

        # Retrieve estimated duration from the planned trajectory
        group.set_joint_value_target(joint_goal)
        plan = group.plan()
        trajectory = plan[1].joint_trajectory
        estimated_duration_pose_rev = trajectory.points[-1].time_from_start.to_sec()

        ts1_rev = get_gazebo_timestamp()
        group.execute(plan[1].joint_trajectory)
        group.stop()
        ts2_rev = get_gazebo_timestamp()

        # move back
        # group.go(joints, wait=True)
        # group.stop()
        # ts5 = get_gazebo_timestamp()

        cur_T, cur_fetch_pose, cur_box_pose = get_pose_gazebo(model_name)
        # print(f"Current Box: \n{cur_T}\nPrev Box:{T}\nZ-diff: {cur_T[2][3] - T[2][3]}")
        grasp_status = 'SUCCESS' if round(cur_T[2][3] - T[2][3], 1) == 0.3 else 'FAIL'
        rospy.loginfo(f"Iteration: {i+1} Grasp status: {grasp_status}")
        results.append(
            (
                estimated_duration_pose, (ts2 - ts1).to_sec(), ((ts2 - ts1).to_sec())/estimated_duration_pose, 
                estimated_duration_grasp, (ts4 - ts3).to_sec(), ((ts4 - ts3).to_sec())/estimated_duration_grasp, 
                (ts_gripper - ts4).to_sec(),
                estimated_duration_pose_rev, (ts2_rev - ts1_rev).to_sec(), ((ts2_rev - ts1_rev).to_sec())/estimated_duration_pose_rev,
                grasp_status
            ))
        time.sleep(5)
        reset_objects()
        time.sleep(2)
    
    for result in results:
        rospy.loginfo(f"RESULTS: ")
        print(f"Estimated time from init pose to above cube: {result[0]} s")
        print(f"Time to move from init pose to above cube: {result[1]} s")
        print(f"Factor: {result[2]}")
        print(f"Estimated time time for grasp {result[3]} s")
        print(f"Time to move to grasp the cube: {result[4]} s")
        print(f"Factor: {result[5]}")
        print(f"Time to grip the cube: {result[6]} s")
        print(f"Estimated time to lift cube: {result[7]} s")
        print(f"Time to lift cube: {result[8]} s")
        print(f"Factor: {result[9]}")
        print(f"Grasp status: {result[10]}")
    
    pd.DataFrame(results, columns=[
        "Estimate_init_pregrasp", "Time_init_pregrasp", "Factor_init_pregrasp",
        "Estimate_pregrasp_grasp", "Time_pregrasp_grasp", "Factor_pregrasp_grasp",
        "Time_grip",
        "Estimate_grasp_postgrasp", "Time_grasp_postgrasp", "Factor_grasp_postgrasp",
        "Grasp"
    ]).to_csv("results.csv")
    

    time.sleep(10)
