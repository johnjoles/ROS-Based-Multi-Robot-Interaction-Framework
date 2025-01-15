#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Movement command initialization
        self.move_cmd = Twist()

        # Minimum safe distance for obstacle detection
        self.min_safe_distance = 1.0  # in meters
        self.prev_direction = None  # Keeps track of last movement direction to avoid flipping continuously

    def laser_callback(self, scan_data):
        # Defining the angular range of interest (-45° to +45°)
        min_angle = -0.785  # -45 degrees in radians
        max_angle = 0.785   # +45 degrees in radians

        # Calculating the corresponding indices for laser scan
        start_index = int((min_angle - scan_data.angle_min) / scan_data.angle_increment)
        end_index = int((max_angle - scan_data.angle_min) / scan_data.angle_increment)

        # Extract the laser scan data within the specified range
        forward_ranges = scan_data.ranges[start_index:end_index]

        # Calculate minimum distance in the forward scan range
        min_distance = min(forward_ranges) if forward_ranges else float('inf')

        # Split the ranges into left and right parts
        middle_index = len(forward_ranges) // 2
        left_ranges = forward_ranges[:middle_index]
        right_ranges = forward_ranges[middle_index:]

        total_left_ranges = sum(left_ranges)
        total_right_ranges = sum(right_ranges)

        # Check if an obstacle is detected in the front
        if min_distance < self.min_safe_distance:
            rospy.loginfo("Obstacle detected in front!")

            # Decision-making based on left vs right ranges
            if total_left_ranges > total_right_ranges:
                if self.prev_direction != "left":
                    rospy.loginfo("Obstacle on left side, turning right")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = 3.0  # Turn right
                    self.prev_direction = "right"
                else:
                    rospy.loginfo("Already turning right, continuing")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = 3.0
            elif total_left_ranges < total_right_ranges:
                if self.prev_direction != "right":
                    rospy.loginfo("Obstacle on right side, turning left")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = -3.0  # Turn left
                    self.prev_direction = "left"
                else:
                    rospy.loginfo("Already turning left, continuing")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = -3.0
            else:
                # If left and right ranges are similar, just turn randomly
                rospy.loginfo("Left and right ranges equal, choosing random turn")
                self.move_cmd.linear.x = -0.5
                self.move_cmd.angular.z = random.choice([3.0, -3.0])
                self.prev_direction = None  # No clear preference

        else:
            rospy.loginfo("Path is clear. Moving forward.")
            # Move forward if no obstacle is detected in the front
            self.move_cmd.linear.x = 0.5  # Move forward at 0.3 m/s
            self.move_cmd.angular.z = 0.0  # No rotation

        # Publish the movement command
        self.vel_pub.publish(self.move_cmd)

    def run(self):
        # Set the rate at which to run the loop
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Keep publishing the last set command
            self.vel_pub.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Create the obstacle avoidance object and run it
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
