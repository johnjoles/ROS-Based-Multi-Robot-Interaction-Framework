#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
import time

class MotionPlanNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('motion_plan_node', anonymous=True)
        
        # Publisher to the cmd_vel topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Set the rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def move_square(self):
        # Twist message instance
        vel_msg = Twist()
        
        # Define the side length time duration in seconds
        side_duration = 1.0  # time to move one side of the square
        turn_duration = 1.5  # time to make a 90-degree turn

        # Repeat 4 times to make a square
        for _ in range(1):
            # Move forward
            vel_msg.linear.x = 0.0  # Move forward with 0.2 m/s
            vel_msg.angular.z = 1.5
            self.publish_velocity(vel_msg, side_duration)

            # Stop
            vel_msg.linear.x = 0.0
            self.publish_velocity(vel_msg, 0.5)  # Stop for 0.5 seconds
            
            # Turn 90 degrees
            #vel_msg.angular.z = 0.7  # 0.5 rad/s rotation
            self.publish_velocity(vel_msg, turn_duration)

            # Stop rotation
            #vel_msg.angular.z = 0.0
            #self.publish_velocity(vel_msg, 0.5)  # Stop for 0.5 seconds

    def publish_velocity(self, vel_msg, duration):
        # Publish velocity for a given duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

    def stop_robot(self):
        # Stop the robot by publishing zero velocities
        vel_msg = Twist()
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        # Initialize the node and move the robot
        motion_planner = MotionPlanNode()
        motion_planner.move_square()
        motion_planner.stop_robot()
    except rospy.ROSInterruptException:
        pass

