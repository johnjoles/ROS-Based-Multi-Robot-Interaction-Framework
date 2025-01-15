#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates

class RobotPosition:
    def __init__(self):
        rospy.init_node('robot_position_node', anonymous=True)
        
        # Subscriber to Gazebo model states
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # Variables to store robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

    def model_states_callback(self, msg):
        """Callback to get robot's position from Gazebo model states."""
        # Extract the robot's position (assuming the robot's name is 'robot')
        robot_index = msg.name.index('ulrich_robot')  # Replace 'robot' with your robot's name
        self.robot_x = msg.pose[robot_index].position.x
        self.robot_y = msg.pose[robot_index].position.y

        rospy.loginfo(f"Robot Position: x={self.robot_x}, y={self.robot_y}")

    def get_position(self):
        """Returns the current position."""
        return self.robot_x, self.robot_y

if __name__ == '__main__':
    try:
        robot_position = RobotPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
