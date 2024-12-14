import rospy
from rosgraph_msgs.msg import Clock

class WaitForGazeboTime:
    def __init__(self, target_time):
        """
        Initialize the waiting mechanism.
        :param target_time: The expected simulation time in seconds.
        """
        self.target_time = target_time
        self.current_time = 0.0
        # rospy.init_node('wait_for_time', anonymous=True)
        rospy.Subscriber('/clock', Clock, self.clock_callback)

    def clock_callback(self, msg):
        """
        Callback to update the current simulation time.
        :param msg: Clock message containing the current simulation time.
        """
        self.current_time = msg.clock.secs + msg.clock.nsecs * 1e-9

    def wait(self):
        """
        Wait until the simulation time reaches the target time.
        """
        rospy.loginfo(f"Waiting for simulation time to reach {self.target_time} seconds.")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_time >= self.target_time:
                rospy.loginfo(f"Target time {self.target_time} seconds reached!")
                break
            rate.sleep()