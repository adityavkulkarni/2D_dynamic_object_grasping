import rospy
import argparse
import moveit_commander
import threading
import time
import pandas as pd

from utils import *
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    """
    Main function to run the code
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--iters', help='iterations', type=int, default=1)
    parser.add_argument('--results', help='store results', type=bool, default=True)
    args = parser.parse_args()

    # intialize ros node
    rospy.init_node('planning_scene_block')
    rospy.set_param('/use_sim_time', True)

    # query the demo cube pose
    model_name = 'demo_cube'
    T, fetch_pose, box_pose = get_pose_gazebo(model_name)
    
    box_pose.position.y = 0.4
    set_model_pose(model_name, box_pose)
    T, fetch_pose, box_pose = get_pose_gazebo(model_name)
    # translation
    trans = T[:3, 3]
    # quaternion in ros
    qt = ros_quat(mat2quat(T[:3, :3]))
    
    """# publish the cube tf for visualization
    x = threading.Thread(target=publish_tf, args=(trans, qt, model_name))
    x.start()"""

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
    rotation_matrix_Y = rotY(angle)
    rotation_matrix_Z = rotZ(angle)

    # Apply the rotation to the transformation matrix T
    T_rotated = np.dot(rotation_matrix_Y, T)
    T_rotated = np.dot(rotation_matrix_Z, T_rotated)

    # Extract the rotated quaternion
    rotated_qt = mat2quat(T_rotated[:3, :3])

    # Move directly initial pose
    trans_1 = [trans[0], 0, trans[2] + 0.5]
    sol_init = get_track_ik_solution(seed_state, trans_1, rotated_qt)   
    seed_state = sol_init
    """
    (0.4, -0.47298796080251726, -0.885309167697212, 0.9398159739359973, 1.477055173112182, -0.5653652160051996, 1.2667744594915047, -1.0417966450715803)
    """
    print(sol_init)
    joint_goal_init = sol_init[1:]
    group.set_joint_value_target(joint_goal_init)
    plan_init = group.plan()
    group.execute(plan_init[1].joint_trajectory)
    group.stop()
    
    results = []
    rospy.sleep(3.0)
    from move_cube_linear import CubeMover
    mover = CubeMover()
    y_intercept = 0.198
    for i in range(args.iters):
        set_cube_pose(box_pose, y=0.4)
        mover.start()
        T, fetch_pose, box_pose = get_pose_gazebo(model_name)
        trans = T[:3, 3]
        trans = [trans[0], y_intercept-0.001, trans[2]]
        rospy.loginfo(f"Iteration {i+1}: Cube postion = {trans}")

        ts_final = get_gazebo_timestamp()
        ts_sol1 = get_gazebo_timestamp()
        ts_sol_r1 = time.time()

        # Move directly above the cube
        trans_1 = [trans[0], trans[1], trans[2] + 0.5]
        sol1 = get_track_ik_solution(seed_state, trans_1, rotated_qt)
        seed_state = sol1
        trans_2 = [trans[0], trans[1], trans[2] + 0.2]
        sol2 = get_track_ik_solution(seed_state, trans_2, rotated_qt)
        
        ts_sol2 = get_gazebo_timestamp()
        ts_sol_r2 = time.time()
        ts_move_1 = get_gazebo_timestamp()
        
        # Plan
        joint_goal1 = sol1[1:]
        group.set_joint_value_target(joint_goal1)
        plan1 = group.plan()
        estimated_duration_pose = plan1[1].joint_trajectory.points[-1].time_from_start.to_sec()

        # move to above the cube
        group.execute(plan1[1].joint_trajectory)
        group.stop()

        joint_goal2 = sol2[1:]
        group.set_joint_value_target(joint_goal2)
        plan2 = group.plan()
        estimated_duration_grasp = plan2[1].joint_trajectory.points[-1].time_from_start.to_sec()

        # move to grasp the cube
        group.execute(plan2[1].joint_trajectory)
        group.stop()
        T1, fetch_pose1, box_pose1 = get_pose_gazebo(model_name)
        rospy.loginfo(f"Cube pose before grasp: {T1[:3, 3]}")
        mover.stop()
        T1, fetch_pose1, box_pose1 = get_pose_gazebo(model_name)
        trans_grasp = T1[:3, 3]
        y_intercept = trans_grasp[1]
        rospy.loginfo(f"Cube pose after grasp: {trans_grasp}")
        ts_move_2 = get_gazebo_timestamp()

        gripper_group.set_joint_value_target(pos_close)
        gripper_open_plan = gripper_group.plan()
        gripper_group.execute(gripper_close_plan[1].joint_trajectory)
        gripper_group.stop()

        ts_grip = get_gazebo_timestamp()

        # Pick the cube
        seed_state = sol2
        trans_3 = [trans[0], trans[1], trans[2] + 0.5]
        sol3 = get_track_ik_solution(seed_state, trans_3, rotated_qt)
        joint_goal = sol3[1:]
        group.set_joint_value_target(joint_goal)
        plan3 = group.plan()
        trajectory = plan3[1].joint_trajectory
        estimated_duration_pose_rev = trajectory.points[-1].time_from_start.to_sec()

        group.execute(plan3[1].joint_trajectory)
        group.stop()
        ts_final2 = get_gazebo_timestamp()

        cur_T, cur_fetch_pose, cur_box_pose = get_pose_gazebo(model_name)
        grasp_status = 'SUCCESS' if round(cur_T[2][3] - T[2][3], 1) == 0.3 else 'FAIL'
        if grasp_status == 'SUCCESS':
            rospy.loginfo(f"Iteration: {i+1} Grasp status: {grasp_status}")
        else:
            rospy.logerr(f"Iteration: {i+1} Grasp status: {grasp_status}")
        results.append(
            {
                "iteration": i+1, 
                "grasp_status": grasp_status,
                "total_time": (ts_final2 - ts_final).to_sec(),
                "solution_time_sim": (ts_sol2 - ts_sol1).to_sec(),
                "solution_time_real": ts_sol_r2 - ts_sol_r1,
                "solution_time_offset": (ts_sol_r2 - ts_sol_r1) - ((ts_sol2 - ts_sol1).to_sec()),
                "grasp_pose_time": (ts_move_2 - ts_move_1).to_sec(),
                "grasp_pose_estimate": (estimated_duration_grasp + estimated_duration_pose),
                "offset": ((ts_move_2 - ts_move_1).to_sec() - (estimated_duration_grasp + estimated_duration_pose)),
                "grip_time": (ts_grip - ts_move_2).to_sec(),
                "trans_grasp":trans_grasp[1]
            }
            )
        rospy.sleep(5)
        gripper_group.set_joint_value_target(pos_open)
        gripper_open_plan = gripper_group.plan()
        gripper_group.execute(gripper_open_plan[1].joint_trajectory)
        gripper_group.stop()
        group.set_joint_value_target(joint_goal_init)
        plan_init = group.plan()
        group.execute(plan_init[1].joint_trajectory)
        group.stop()
        reset_objects(scene, box_pose, fetch_pose)
        time.sleep(2)
    
    for result in results:
        rospy.loginfo(f"RESULTS: ")
        print(f'Iteration: {result["iteration"]}')
        print(f'Grasp Status: {result["grasp_status"]}')
        print(f'Total Time: {result["total_time"]}')
        print(f'Solution calculation Time(sim): {result["solution_time_sim"]}')
        print(f'Solution calculation Time(real): {result["solution_time_real"]}')
        print(f'Solution calculation Time(diff): {result["solution_time_offset"]}')
        print(f'Pregrasp Movement Time: {result["grasp_pose_time"]}')
        print(f'Pregrasp Movement Estimate: {result["grasp_pose_estimate"]}')
        print(f'Pregrasp Movement Offset: {result["offset"]}')
        print(f'Gripping Time: {result["grip_time"]}')
        print(f'Position at the time of grasp: {result["trans_grasp"]}')
    if args.results:
        pd.DataFrame(results).to_csv("results.csv")
    rospy.signal_shutdown("")

