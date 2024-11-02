#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def conveyor_mover():
    rospy.init_node('conveyor_mover', anonymous=True)
    pub = rospy.Publisher('/conveyor/moving_cube_position_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    position = 0.0
    speed = 0.01  # Reduced speed
    
    while not rospy.is_shutdown():
        position += speed
        if position > 1.0:  # Reset position when reaching limit
            position = -1.0
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        conveyor_mover()
    except rospy.ROSInterruptException:
        pass