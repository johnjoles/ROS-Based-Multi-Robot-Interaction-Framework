#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pi
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates

class RobotMovement:
    def __init__(self):
        rospy.init_node('robot_movement_node', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        # Movement command initialization
        self.move_cmd = Twist()

        # Minimum safe distance for obstacle detection
        self.min_safe_distance = 1.0 # in meters

        # Goal coordinates
        self.goal_x = -1.0
        self.goal_y = 0.5

        # Robot's position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        # Laser scan regions
        self.right_side = self.right_front_side = self.front_side = self.left_front_side = self.left_side = float('inf')

        # Update rate
        self.rate = rospy.Rate(10)  # 10 Hz

        self.obstacle_detected = False

 
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

    def laser_callback(self, scan_data):
        """Callback for processing laser scan data."""
        regions = [
            min(scan_data.ranges[0:144]),
            min(scan_data.ranges[144:288]),
            min(scan_data.ranges[288:432]),
            min(scan_data.ranges[432:576]),
            min(scan_data.ranges[576:720])
        ]

        self.right_side = regions[0]
        self.right_front_side = regions[1]
        self.front_side = regions[2]
        self.left_front_side = regions[3]
        self.left_side = regions[4]

        # Check if any region has an obstacle closer than the minimum safe distance
        self.obstacle_detected = any(region < self.min_safe_distance for region in regions)

    def calculate_distance_to_goal(self):
        """Euclidean distance to the goal."""
        return sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

    def calculate_angle_to_goal(self):
        """Angle to the goal relative to the robot's current heading."""
        return atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

    def turning(self,angular_speed):
        """Turn the robot at the specified angular speed."""
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = angular_speed
        self.vel_pub.publish(self.move_cmd)
        direction = "Left" if angular_speed > 0 else "Right"
        rospy.loginfo(f"Obstacles Free and Turning {direction} with angular speed: {angular_speed}")

    def no_obst_forward(self, speed):
        """Move the robot forward at the specified speed."""
        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = 0.0
        self.vel_pub.publish(self.move_cmd)
        rospy.loginfo(f"Moving Obstacle Free Forward with speed: {speed}")

    def move_forward(self, speed):
        """Move the robot forward at the specified speed."""
        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = 0.0
        self.vel_pub.publish(self.move_cmd)
        rospy.loginfo(f"Moving Forward with speed: {speed}")

    def turn_90_degrees(self, angular_speed):
        """Turn the robot 90 degrees in 2 seconds."""
        start_time = rospy.Time.now()
        duration = rospy.Duration(2)  # 2 seconds
        direction = "Left" if angular_speed > 0 else "Right"

        while rospy.Time.now() - start_time < duration:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = angular_speed
            self.vel_pub.publish(self.move_cmd)
            self.rate.sleep()  # Ensure the loop respects the node's update rate

        rospy.loginfo(f"Turning 90 Degree {direction} with angular speed: {angular_speed}")
        # Stop the robot after completing the turn
        self.move_cmd.angular.z = 0.0
        self.vel_pub.publish(self.move_cmd)
        rospy.loginfo("Completed 90-degree turn.")
    def turn(self, angular_speed):
        """Turn the robot at the specified angular speed."""
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = angular_speed
        self.vel_pub.publish(self.move_cmd)
        direction = "Left" if angular_speed > 0 else "Right"
        rospy.loginfo(f"Turning {direction} with angular speed: {angular_speed}")

    def move_backward(self, speed):
        """Move the robot backward at the specified speed."""
        self.move_cmd.linear.x = -speed
        self.move_cmd.angular.z = 0.0
        self.vel_pub.publish(self.move_cmd)
        rospy.loginfo(f"Moving Backward with speed: {speed}")

    def avoid_obstacles(self, front_side, left_front_side, right_front_side, left_side, right_side):
        """Main obstacle avoidance logic."""

        if front_side > self.min_safe_distance and left_front_side > self.min_safe_distance and right_front_side > self.min_safe_distance:
            rospy.loginfo("No Obstacles Anywhere -- Move Forward")
            linear_speed = max(0.1, min(0.5, 0.2 * (front_side - self.min_safe_distance)))
            self.move_forward(linear_speed)

        elif front_side < self.min_safe_distance:

            if left_front_side > self.min_safe_distance and right_front_side > self.min_safe_distance:

                if left_front_side > right_front_side and left_side > right_side:
                    self.turning(0.25)
                elif right_front_side > left_front_side and right_side > left_side:
                    self.turning(-0.25)

            elif right_front_side < self.min_safe_distance:
                rospy.loginfo("Obstacles in Front Right -- Turn Left")
                self.turn(0.25)

            elif left_front_side < self.min_safe_distance:
                rospy.loginfo("Obstacles in Front Left -- Turn Right")
                self.turn(-0.25)

            elif left_front_side < self.min_safe_distance and right_front_side < self.min_safe_distance:

                rospy.loginfo("Blocked in Front and on Both Sides -- Moving Back")
                self.move_backward(0.15)

                if front_side > left_front_side and front_side > right_front_side:
                    rospy.loginfo("Narrow Passage Detected -- Robot can't move")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)
                    return False
                                   
                elif left_side < right_side:
                    #rospy.loginfo("Turning Right to Clear Path")
                    self.turn_90_degrees(-0.5)
                    
                elif left_side > right_side:
                    #rospy.loginfo("Turning Left to Clear Path")
                    self.turn_90_degrees(0.5)

                else:
                    print("Robot can't Move, Path is Blocked")
                    self.move_cmd.linear.x = 0.0
                    self.move_cmd.angular.z = 0.0
                    self.vel_pub.publish(self.move_cmd)
                    return False


        elif front_side > self.min_safe_distance and left_front_side < self.min_safe_distance and right_front_side < self.min_safe_distance:
            rospy.loginfo("Obstacles in Front Right and Front Left -- Move Slowly Forward")
            linear_speed = max(0.1, min(0.5, 0.5 * (front_side - self.min_safe_distance)))
            self.move_forward(linear_speed)


    def navigate_to_goal(self):
        """Navigate towards the goal while avoiding obstacles."""
        rospy.loginfo(f"Robot Position: x={self.current_x}, y={self.current_y}")

        # Calculate deltas to goal
        delta_x = self.goal_x - self.current_x
        delta_y = self.goal_y - self.current_y

        # Calculate distance and angle to the goal
        distance_to_goal = sqrt(delta_x**2 + delta_y**2)
        angle_to_goal = atan2(delta_y, delta_x)

        # Adjust angular velocity
        angle_difference = angle_to_goal - self.current_orientation
        angle_difference = (angle_difference + 3.14) % (2 * 3.14) - 3.14  # Normalize angle


        if abs(distance_to_goal) < 0.3:  # 30 cm tolerance
            rospy.loginfo("Goal reached!")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.vel_pub.publish(self.move_cmd)
            return False
            

        elif distance_to_goal >= 0.3:
            print("Goal Distance :", distance_to_goal)
            print("Angle Difference :", angle_difference)

            if self.obstacle_detected:
                self.avoid_obstacles(self.front_side, self.left_front_side, self.right_front_side, self.left_side, self.right_side)

            elif abs(angle_difference) >= 0.25:
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.5 * angle_difference
                self.vel_pub.publish(self.move_cmd)

            else:
                linear_speed = max(0.1, min(0.5, 0.2 * (self.front_side - self.min_safe_distance)))
                #print("Linear Speed : ", linear_speed)
                self.no_obst_forward(linear_speed)
           

        self.vel_pub.publish(self.move_cmd)
        return True  # Continue moving

    def run(self):
        """Main loop to handle navigation and obstacle avoidance."""

        rospy.loginfo("Starting Robot Motion...")
        while not rospy.is_shutdown():
            if self.navigate_to_goal() == False:
                break
            self.navigate_to_goal()
            self.vel_pub.publish(self.move_cmd)
            self.rate.sleep()

        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.vel_pub.publish(self.move_cmd)
        rospy.loginfo("Robot Motion terminated.")

if __name__ == '__main__':
    try:
        robot_motion = RobotMovement()
        robot_motion.run()
    except rospy.ROSInterruptException:
        pass
