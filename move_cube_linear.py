#!/usr/bin/env python3

import math
from grasp import get_pose_gazebo, set_model_pose
import time
import time
import threading
import rospy
from gazebo_msgs.srv import SetModelState
import numpy as np
from gazebo_msgs.msg import ModelState


class CubeMover:
    def __init__(self, model_name='demo_cube', motion_type='linear', velocity=0.1):
        self.model_name = model_name
        self.box_pose = None
        self.sleep = 0.01
        self.velocity = velocity
        self.incr = -velocity * self.sleep
        self.iter = 10
        self.running = False

    def get_pose(self):
        # This function should query the current pose of the cube
        T, fetch_pose, box_pose = get_pose_gazebo(self.model_name)
        return box_pose

    def reset(self):
        pose = self.get_pose()
        pose.position.x = 0.6
        pose.position.y = 0
        pose.position.z = 0.724329
        self.box_pose = pose
        self.set_pose()

    def set_pose(self):
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            # Create a ModelState object
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose = self.box_pose
            # Call the service
            response = set_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def start(self, motion="linear"):
        self.running = True
        self.box_pose = self.get_pose()
        rospy.loginfo("Cube movement starting")

        def move_linear():
            while self.running:
                if self.box_pose.position.y <= -0.4 or self.box_pose.position.y > 0.4:
                    self.iter -= 1
                    self.incr *= -1

                self.box_pose.position.y += self.incr
                ret = self.set_pose()
                if ret == -1:
                    break
                time.sleep(self.sleep)

        def move_circular(radius=0.2):
            angular_speed = self.velocity / radius
            cur_pose = self.get_pose()
            center = (cur_pose.position.x + radius, cur_pose.position.y)
            angle = np.arctan2(cur_pose.position.x - center[0], cur_pose.position.y - center[1])
            t = angle / angular_speed
            while self.running:
                angle = angular_speed * t
                self.box_pose.position.x = center[0] + (radius * math.cos(angle))
                self.box_pose.position.y = center[1] + (radius * math.sin(angle))

                ret = self.set_pose()
                if ret == -1:
                    break
                t += self.sleep * 0.1
                time.sleep(self.sleep)

        # Start the movement in a new thread
        move = move_linear if motion == "linear" else move_circular
        thread = threading.Thread(target=move)
        thread.start()
        self.thread = thread

    def stop(self):
        # Stop the movement and wait for the thread to finish
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
"""def start_move():
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
        ret = set_model_pose(model_name=model_name, pose=box_pose)
        if ret == -1: break
        time.sleep(0.01)"""


if __name__ == "__main__":
    """
    Main function to run the code
    """
    # start_move()
    mover = CubeMover()
    mover.reset()
    mover.start()
    time.sleep(30)
    print(10)
    mover.stop()  # Call this to stop the movement

