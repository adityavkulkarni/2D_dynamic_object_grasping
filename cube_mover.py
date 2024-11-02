#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def cube_mover():
    # Initialize the node
    rospy.init_node('cube_mover', anonymous=True)
    
    # Create a publisher for model state
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
    # Wait for gazebo set_model_state service
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # Set the rate
    rate = rospy.Rate(10)  # 10 Hz
    
    # Initial position (matching the pose in simple_pick_place.sdf)
    x = 0.6  # Initial x position from SDF
    y = 0.1  # Initial y position from SDF
    z = 0.75  # Initial z position from SDF
    
    # Movement speed (meters per second)
    speed = 0.01
    
    while not rospy.is_shutdown():
        # Create a new model state message
        model_state_msg = ModelState()
        model_state_msg.model_name = "demo_cube"
        model_state_msg.pose.position.x = x
        model_state_msg.pose.position.y = y
        model_state_msg.pose.position.z = z
        
        # Set orientation (no rotation)
        model_state_msg.pose.orientation.x = 0
        model_state_msg.pose.orientation.y = 0
        model_state_msg.pose.orientation.z = 0
        model_state_msg.pose.orientation.w = 1
        
        # Set reference frame
        model_state_msg.reference_frame = "world"
        
        try:
            # Publish the new state
            pub.publish(model_state_msg)
            
            # Increment y position for next iteration
            y += speed
            
            # Optional: Reset position if the cube goes too far
            if y > 1.0:  # Adjust this value based on your needs
                y = 0.1  # Reset to initial y position
                
        except rospy.ROSInterruptException:
            pass
            
        rate.sleep()

if __name__ == '__main__':
    try:
        cube_mover()
    except rospy.ROSInterruptException:
        pass