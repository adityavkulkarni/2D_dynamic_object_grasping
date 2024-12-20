import multiprocessing
import threading

import numpy as np
import rospy
from rosgraph_msgs.msg import Clock

from WaitForGazeboTime import WaitForGazeboTime
from predict_object_trajectory import HybridCurveFitter
# from get_cube_pose import get_cube_pose_and_timestamp
from gazebo_msgs.srv import GetModelState
from trac_ik_python.trac_ik import IK
import time
import pandas as pd
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#from grasp import mat2quat, rotY, rotZ, ros_quat, IK, get_pose_gazebo
from transforms3d.quaternions import mat2quat, quat2mat
from std_msgs.msg import Float32MultiArray

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

def get_track_ik_solution(seed_state_input, trans_input, rotated_qt_input):
    retry = 30
    sol = None
    ik_solver = IK("base_link", "wrist_roll_link")
    lower_bound, upper_bound = ik_solver.get_joint_limits()
    lower_bound = list(lower_bound)
    upper_bound = list(upper_bound)
    lower_bound[0] = 0.4
    upper_bound[0] = 0.4
    ik_solver.set_joint_limits(lower_bound, upper_bound)
    while sol is None:
        # multithread and random state
        sol = ik_solver.get_ik(seed_state_input,
                                    trans_input[0], trans_input[1], trans_input[2],
                                    rotated_qt_input[0], rotated_qt_input[1], rotated_qt_input[2],
                                    rotated_qt_input[3])
        """rospy.loginfo('Solution from IK:')
        print(ik_solver.joint_names)                
        print(sol)"""
        if sol: break
        retry -= 1
    return sol

def get_solution(task):
    task = task[0]
    print(f"Process {task['task_id']} started.")
    y = task["y"]
    seed_state = (
        0.4, -0.47298796080251726, -0.885309167697212, 0.9398159739359973, 1.477055173112182, -0.5653652160051996,
        1.2667744594915047, -1.0417966450715803)

    T, fetch_pose, box_pose = get_pose_gazebo("demo_cube")
    T[1, 3] = y
    # # translation
    # trans = T[:3, 3]
    # # quaternion in ros
    # # qt = ros_quat(mat2quat(T[:3, :3]))
    # angle = np.pi / 2
    # # angle = 0
    #
    # # Create the rotation matrix for 90 degrees about the y-axis
    # rotation_matrix_Y = rotY(angle)
    # rotation_matrix_Z = rotZ(angle)
    #
    # # Apply the rotation to the transformation matrix T
    # T_rotated = np.matmul(rotation_matrix_Y, T)
    # T_rotated = np.matmul(rotation_matrix_Z, T_rotated)
    # print(f"TRANSFORMATION MATRIX {task['task_id']}:")
    # print(T_rotated)
    #
    # # Extract the rotated quaternion
    # rotated_qt = ros_quat(mat2quat(T_rotated[:3, :3]))
    # # print("TRANS", trans)
    # # time.sleep(10)
    # # TODO: do how it's done in the homework
    #
    #
    # trans_1 = [trans[0], trans[1], trans[2] + 0.5]
    # print(f"TRANS1 {task['task_id']}", trans_1)
    # sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
    # seed_state = sol1
    # trans_2 = [trans[0], trans[1], trans[2] + 0.2]
    # print(f"TRANS2 {task['task_id']}", trans_2)
    # # time.sleep(10)
    # sol2 = get_track_ik_solution(seed_state, trans_2, rotated_qt)
    #
    # # result = f"Result from thread {thread_id}"  # Simulated result
    # # print(sol1, sol2)
    # # Store result in a thread-safe way
    # # with self.lock:
    # #     self.results[thread_id] = (sol1, sol2)

    trans = T[:3, 3]
    # quaternion in ros
    qt = ros_quat(mat2quat(T[:3, :3]))
    angle_y = np.pi / -2
    angle_z = np.pi / -2

    # Create the rotation matrix for 90 degrees about the y-axis
    rotation_matrix_Y = rotY(angle_y)
    rotation_matrix_Z = rotZ(angle_z)

    # Apply the rotation to the transformation matrix T
    T_rotated = np.dot(rotation_matrix_Y, T)
    T_rotated = np.dot(rotation_matrix_Z, T_rotated)


    # Extract the rotated quaternion
    rotated_qt = mat2quat(T_rotated[:3, :3])

    trans_1 = [trans[0], trans[1], trans[2] + 0.5]
    sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
    seed_state = sol1
    trans_2 = [trans[0], trans[1], trans[2] + 0.2]
    sol2 = get_track_ik_solution(seed_state, trans_2, rotated_qt)
    seed_state = sol2
    sol3 = get_track_ik_solution(seed_state, trans_1, rotated_qt)


    return sol1, sol2, sol3

class FindIntercept:
    def __init__(self):
        # intialize ros node
        rospy.init_node('planning_scene_block')
        rospy.set_param('/use_sim_time', True)
        self.results = {}
        # self.lock = threading.Lock()
        self.ik_solver = IK("base_link", "wrist_roll_link")
        self.t_pose_dist_df = pd.read_csv("results80.csv")

    def get_gazebo_timestamp(self):
        rospy.wait_for_message('/clock', Clock)

        clock_msg = rospy.wait_for_message('/clock', Clock)

        return clock_msg.clock

    def get_future_trajectory(self, start_time):
        omega = 0 # Angular frequency for periodic motion with period 5
        degree = 3  # Degree of polynomial for quadratic trend

        hybrid_rls_x = HybridCurveFitter(degree=degree, omega=omega)
        hybrid_rls_y = HybridCurveFitter(degree=degree, omega=omega)

        # Arrays to store online predictions
        pred_x = []
        pred_y = []

        current_gz_time = start_time
        # current_gz_time = 0

        # Online learning: Update the hybrid model with each new data point
        for _ in range(100):
            # cube_pose, _ = get_cube_pose_and_timestamp()
            _,_,cube_pose = get_pose_gazebo('demo_cube')
            t = self.get_gazebo_timestamp().to_sec()
            hybrid_rls_x.update(t, cube_pose.position.x)
            hybrid_rls_y.update(t, cube_pose.position.y)

            # print()
        range_values = np.arange(0, 30, 0.01)

        # Predict future values based on the current model
        for step in range_values:
            pred_x.append(hybrid_rls_x.predict(current_gz_time + step))
            pred_y.append(hybrid_rls_y.predict(current_gz_time + step))

        return pred_x, pred_y, range_values

    def send_points(self, points):
        # rospy.init_node('point_sender', anonymous=True)
        point_pub = rospy.Publisher('input_points', Float32MultiArray, queue_size=100)
        rate = rospy.Rate(1)  # 1 Hz

        print("SENDING POINTS: ", points)
        # while not rospy.is_shutdown():
        msg = Float32MultiArray()
        msg.data = points
        for _ in range(5):# Flatten list
            point_pub.publish(msg)
        rospy.loginfo(f"Sent points: {msg.data}")
            # rate.sleep()

    def find_intercept(self):
        self.results = None
        gz_time = self.get_gazebo_timestamp().to_sec()

        pred_x, pred_y, og_timesteps = self.get_future_trajectory(gz_time)
        y_traj = np.array(pred_y)

        mask = (y_traj >= -0.4) & (y_traj <= 0.4) & (y_traj >= 0.2) & (y_traj <= 0.35)
        y_traj = y_traj[mask]
        print(y_traj)
        timesteps = og_timesteps[mask]

        gripper_distance = 0.1
        cube_width = 0.06
        gripper_velocity = 0.1

        t_gripper = (gripper_distance - cube_width) / gripper_velocity

        timesteps = timesteps - t_gripper + gz_time

        # TODO: subtract distribution time
        desired_interval = 0.08
        filtered_values = y_traj[np.isclose((y_traj % desired_interval), 0, atol=1e-3)]

        # Ensure the last value is included if it matches the interval condition
        if not np.isclose(filtered_values[-1], y_traj[-1], atol=1e-1):
            filtered_values = np.append(filtered_values, y_traj[-1])

        # Randomly sample from filtered values
        num_samples = 1  # Adjust the number of samples as needed
        sampled_y = np.random.choice(filtered_values, size=num_samples, replace=False)

        self.send_points(sampled_y.tolist())

        # time.sleep(20)

        print("SAMPLED Y")
        print(sampled_y)

        print("TIMESTEPS", timesteps)
        sampled_timesteps = []
        for value in sampled_y:
            idx = np.where(y_traj == value)[0]  # Find the index of the value in the original y array
            if idx.size > 0:  # If value is found
                sampled_timesteps.append(timesteps[idx[0]])
        sampled_timesteps = np.array(sampled_timesteps)
        print(sampled_timesteps)
        # time.sleep(10)

        # Shared dictionary to store outputs
        results = {}


        # List to hold thread objects
        threads = []

        T, _, _ = get_pose_gazebo("demo_cube")

        tasks = []

        # Creating and starting 10 threads
        for i, y in enumerate(sampled_y.tolist()):
            T[1, 3] = y
            print(f"UPDATED T MATRIX {i}:", T)
            # print(T)
            tasks.append({"task_id": i+1, "y": y})

            # thread = threading.Thread(target=self.get_solution, args=(i,y))
            # threads.append(thread)
            # thread.start()

        # Waiting for all threads to complete or timeout
        # threshold_time = 1.5  # Timeout in seconds
        # for thread in threads:
        #     thread.join(timeout=threshold_time)
        # print("TASKS", tasks)
        # with multiprocessing.Pool(processes=num_samples) as pool:
        #     results = pool.map(get_solution, tasks)
        #     pool.terminate()

        results = [get_solution(tasks)]

        # Printing the collected results
        print("Results collected from processes:")
        print(results)
        # for thread_id, result in self.results.items():
        #     print(f"Thread {thread_id}: {results}")

        grasp_pose_times = np.interp(sampled_y, self.t_pose_dist_df['y_cord'], self.t_pose_dist_df['grasp_pose_time'])

        sampled_timesteps = sampled_timesteps - grasp_pose_times

        print(sampled_timesteps)
        # time.sleep(10)

        gz_time_now = self.get_gazebo_timestamp().to_sec()

        sampled_timesteps = sampled_timesteps - gz_time_now + 0

        first_possible_intercept_idx = (sampled_timesteps > 0).argmax() if np.any(sampled_timesteps > 0) else None

        print("SELECTED POINT:")
        print(first_possible_intercept_idx)
        print(sampled_y[first_possible_intercept_idx])
        print(sampled_timesteps[first_possible_intercept_idx])


        # TODO: wait for the remaining time in gz * gz_to_real_factor
        gz_time_waiter = WaitForGazeboTime(sampled_timesteps[first_possible_intercept_idx] + gz_time_now)
        gz_time_waiter.wait()

        self.results = results[first_possible_intercept_idx]
        print("FINALIZED TRAJ: ", self.results)


if __name__ == "__main__":

    fd = FindIntercept()
    fd.find_intercept()