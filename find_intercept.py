import threading

import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from predict_object_trajectory import HybridCurveFitter
# from get_cube_pose import get_cube_pose_and_timestamp
from gazebo_msgs.srv import GetModelState
from trac_ik_python.trac_ik import IK

#from grasp import mat2quat, rotY, rotZ, ros_quat, IK, get_pose_gazebo
from transforms3d.quaternions import mat2quat, quat2mat

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

class FindIntercept:
    def __init__(self):
        # intialize ros node
        rospy.init_node('planning_scene_block')
        rospy.set_param('/use_sim_time', True)
        self.results = {}
        self.lock = threading.Lock()
        self.ik_solver = IK("base_link", "wrist_roll_link")
        lower_bound, upper_bound = self.ik_solver.get_joint_limits()
        lower_bound = list(lower_bound)
        upper_bound = list(upper_bound)
        lower_bound[0] = 0.4
        upper_bound[0] = 0.4
        self.ik_solver.set_joint_limits(lower_bound, upper_bound)

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

    def get_solution(self, thread_id, T):
        print(f"Thread {thread_id} started.")
        # time.sleep(2)  # Simulating computation time

        print(f"Thread {thread_id} completed.")

        def get_track_ik_solution(seed_state, trans, rotated_qt):
            retry = 30
            sol = None
            while sol is None:
                # multithread and random state
                sol = self.ik_solver.get_ik(seed_state,
                                       trans[0], trans[1], trans[2],
                                       rotated_qt[0], rotated_qt[1], rotated_qt[2], rotated_qt[3])
                """rospy.loginfo('Solution from IK:')
                print(ik_solver.joint_names)                
                print(sol)"""
                if sol: break
                retry -= 1
            return sol

        seed_state = (
            0.4, -0.47298796080251726, -0.885309167697212, 0.9398159739359973, 1.477055173112182, -0.5653652160051996,
            1.2667744594915047, -1.0417966450715803)
        # T, fetch_pose, box_pose = get_pose_gazebo(model_name)
        # translation
        trans = T[:3, 3]
        # quaternion in ros
        qt = ros_quat(mat2quat(T[:3, :3]))
        angle = np.pi / -2

        # Create the rotation matrix for 90 degrees about the y-axis
        rotation_matrix_Y = rotY(angle)
        rotation_matrix_Z = rotZ(angle)

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

        result = f"Result from thread {thread_id}"  # Simulated result
        # print(sol1, sol2)
        # Store result in a thread-safe way
        with self.lock:
            self.results[thread_id] = (sol1, sol2, sol3)
        return (sol1, sol2)

    def find_intercept(self):
        gz_time = self.get_gazebo_timestamp().to_sec()

        pred_x, pred_y, timesteps = self.get_future_trajectory(gz_time)
        y_traj = np.array(pred_y)

        mask = (y_traj >= -0.4) & (y_traj <= 0.4)
        y_traj = y_traj[mask]
        print(y_traj)
        timesteps = timesteps[mask]

        gripper_distance = 0.1
        cube_width = 0.06
        gripper_velocity = 0.1

        t_gripper = (gripper_distance - cube_width) / gripper_velocity

        timesteps = timesteps - t_gripper + gz_time

        # subtract distribution time
        desired_interval = 0.1
        filtered_values = y_traj[np.isclose((y_traj % desired_interval), 0, atol=1e-3)]

        # Ensure the last value is included if it matches the interval condition
        if not np.isclose(filtered_values[-1], y_traj[-1], atol=1e-3):
            filtered_values = np.append(filtered_values, y_traj[-1])

        # Randomly sample from filtered values
        num_samples = 1  # Adjust the number of samples as needed
        sampled_y = np.random.choice(filtered_values, size=num_samples, replace=False)

        print(sampled_y)
        # return sampled_y.tolist()[-1]
        # Shared dictionary to store outputs
        results = {}


        # List to hold thread objects
        threads = []

        T, _, _ =get_pose_gazebo("demo_cube")

        # Creating and starting 10 threads
        for i, y in enumerate(sampled_y.tolist()):
            T[1, 3] = y
            thread = threading.Thread(target=self.get_solution, args=(i,T))
            threads.append(thread)
            thread.start()

        # Waiting for all threads to complete or timeout
        threshold_time = 1  # Timeout in seconds
        for thread in threads:
            thread.join(timeout=threshold_time)

        # Printing the collected results
        print("Results collected from threads:")
        for thread_id, result in self.results.items():
            print(f"Thread {thread_id}: {results}")



if __name__ == "__main__":

    fd = FindIntercept()
    fd.find_intercept()

