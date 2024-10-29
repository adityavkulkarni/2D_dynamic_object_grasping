#!/usr/bin/env python3

"""
CS 6301 Homework 4 Programming
Robot Control for Grasping
"""
from grasp import get_pose_gazebo, set_model_pose
import time
import time
import threading
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class CubeMover:
    def __init__(self, model_name='demo_cube'):
        self.model_name = model_name
        self.box_pose = None
        self.incr = -0.001
        self.iter = 10
        self.running = False

    def get_pose(self):
        # This function should query the current pose of the cube
        T, fetch_pose, box_pose = get_pose_gazebo(self.model_name)
        return box_pose

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

    def start(self):
        self.running = True
        self.box_pose = self.get_pose()
        rospy.loginfo("Cube movement starting")
        """self.box_pose.position.x = 0.6
        self.box_pose.position.y = 0.4
        self.box_pose.position.z = 0.724341"""

        def move():
            while self.running:
                if self.box_pose.position.y <= -0.4 or self.box_pose.position.y > 0.4:
                    self.iter -= 1
                    self.incr *= -1

                self.box_pose.position.y += self.incr
                ret = self.set_pose()
                if ret == -1:
                    break
                time.sleep(0.01)

        # Start the movement in a new thread
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
    mover.start()
    time.sleep(10)
    print(10)
    mover.stop()  # Call this to stop the movement

