#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        # Movement command initialization
        self.move_cmd = Twist()

        # Minimum safe distance for obstacle detection
        self.min_safe_distance = 1.0  # in meters

        # Desired goal position
        self.goal_x = -3.0  # Set the goal x-coordinate
        self.goal_y = 3.0  # Set the goal y-coordinate

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        self.rate = rospy.Rate(10)
        self.turn_timeout = 5
        self.turn_counter = 0
        self.rotation_limit = 5
        self.no_rotation = 0

    def pose_callback(self, msg):
        """Callback to update current position and orientation of robot."""
        try:
            robot_index = msg.name.index('combined_robot')  # Replace with your robot's name
        except ValueError:
            rospy.logerr("Robot name not found in /gazebo/model_states.")
            return

        # Update position
        self.current_x = msg.pose[robot_index].position.x
        self.current_y = msg.pose[robot_index].position.y

        # Update orientation
        orientation = msg.pose[robot_index].orientation
        _, _, self.current_orientation = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )


    def restart(self):
        """Handles robot restart logic when too many rotations occur."""
        self.turn_counter += 1
        if self.turn_counter > self.turn_timeout:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0  # Stop turning
            self.turn_counter = 0
            self.vel_pub.publish(self.move_cmd)
            rospy.sleep(1.0)

            self.move_cmd.linear.x = -0.1
            self.vel_pub.publish(self.move_cmd)
            rospy.sleep(1.0)

            self.move_cmd.linear.x = 0.0
            self.vel_pub.publish(self.move_cmd)
            rospy.sleep(1.0)

            self.turn_counter = 0

    def laser_callback(self, scan_data):
        """Callback for processing laser scan data."""
        regions = [
            min(scan_data.ranges[0:143]),
            min(scan_data.ranges[144:287]),
            min(scan_data.ranges[288:431]),
            min(scan_data.ranges[432:575]),
            min(scan_data.ranges[576:713])
        ]

        right_side = regions[0]
        right_front_side = regions[1]
        front_side = regions[2]
        left_front_side = regions[3]
        left_side = regions[4]

        #rospy.loginfo(regions[3:0:-1])

        if self.no_rotation > self.rotation_limit:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.vel_pub.publish(self.move_cmd)
            rospy.sleep(5.0)
            self.no_rotation = 0

    def move_and_turn(self, linear_speed, angular_speed, duration):
        """Helper function to move and turn the robot."""
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.move_cmd.linear.x = linear_speed
            self.move_cmd.angular.z = angular_speed
            self.vel_pub.publish(self.move_cmd)
            rate.sleep()

    def move_towards_goal(self):
        """Calculate the movement towards the goal position."""
        
        rospy.loginfo(f"Robot Position: x={self.current_x}, y={self.current_y}")

        # Calculate deltas
        delta_x = self.goal_x - self.current_x
        delta_y = self.goal_y - self.current_y

        # Calculate distance and angle to the goal
        distance_to_goal = sqrt(delta_x**2 + delta_y**2)
        angle_to_goal = atan2(delta_y, delta_x)

        # Stop if close enough to the goal
        if distance_to_goal < 0.1:  # 10 cm tolerance
            rospy.loginfo("Goal reached!")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.vel_pub.publish(self.move_cmd)
            return False  # Stop moving

        # Adjust angular velocity
        angle_difference = angle_to_goal - self.current_orientation
        angle_difference = (angle_difference + 3.14) % (2 * 3.14) - 3.14  # Normalize angle

        if abs(angle_difference) > 0.1:
            # Rotate towards the goal
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.5 * angle_difference
        else:
            # Move straight towards the goal
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0

        self.vel_pub.publish(self.move_cmd)
        return True  # Continue moving


    def run(self):
        """Main loop."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.move_towards_goal():  # Move towards the goal
                rospy.loginfo("Stopping robot after goal reached.")
                break  # Exit the loop after goal is reached
            rate.sleep()


if __name__ == '__main__':
    try:
        # Create the obstacle avoidance object and run it
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
