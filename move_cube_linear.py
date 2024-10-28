#!/usr/bin/env python

"""
CS 6301 Homework 4 Programming
Robot Control for Grasping
"""

import rospy
import roslib
import numpy as np
import random
import time

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



if __name__ == "__main__":
    """
    Main function to run the code
    """
    # intialize ros node
    rospy.init_node('move_box')
    rospy.set_param('/use_sim_time', True)

    # query the demo cube pose
    model_name = 'demo_cube'
    T, fetch_pose, box_pose = get_pose_gazebo(model_name)
    box_pose.position.x=0.6
    box_pose.position.y=0.4
    box_pose.position.z=0.724341
    set_model_pose(model_name=model_name, pose=box_pose)

    iter = 10
    incr = -0.001
    while iter:
        if box_pose.position.y <= -0.4 or box_pose.position.y > 0.4:
            iter -= 1
            incr = 0.001
        box_pose.position.y += incr
        set_model_pose(model_name=model_name, pose=box_pose)
        time.sleep(0.00001)
    
    rospy.signal_shutdown("")

