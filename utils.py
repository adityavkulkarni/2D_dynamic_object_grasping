import rospy
import random
import numpy as np

from rosgraph_msgs.msg import Clock
from transforms3d.quaternions import mat2quat, quat2mat
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


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


def rotX(rotx):
    RotX = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(rotx), -np.sin(rotx), 0],
            [0, np.sin(rotx), np.cos(rotx), 0],
            [0, 0, 0, 1],
        ]
    )
    return RotX


def rotZ(rotz):
    RotZ = np.array(
        [
            [np.cos(rotz), -np.sin(rotz), 0, 0],
            [np.sin(rotz), np.cos(rotz), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    return RotZ


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
    

"""# publish tf for visualization
def publish_tf(trans, qt, model_name):

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(trans, qt, rospy.Time.now(), model_name, 'base_link')
        rate.sleep()"""


def set_cube_pose(box_pose, y=None):
    global position_list
    if y is None:
        y = random_float()
        while y in position_list and len(position_list) < 80:
            y = random_float()
        position_list.append(y)
    # global pos, incr
    # x = pos + incr
    # pos = y
    box_pose.position.y = y
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
        if sol is None:
            rospy.logerr('Solution from IK:')
            print(ik_solver.joint_names)                
            print(sol)
        if sol: break
        retry -= 1    
    return sol


def reset_objects(scene, box_pose, fetch_pose):
    scene.clear()
    box_pose.position.y = 0.4
    set_model_pose("demo_cube", box_pose)
    set_model_pose("fetch", fetch_pose)


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
