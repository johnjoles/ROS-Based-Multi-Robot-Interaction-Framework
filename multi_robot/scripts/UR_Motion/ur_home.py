#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

def send_joint_trajectory_goal():
    # Initialize the ROS node
    rospy.init_node("send_trajectory_goal", anonymous=True)
    
    # Create a publisher to the follow_joint_trajectory goal topic
    trajectory_pub = rospy.Publisher("/ur10_arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=10)
    
    # Wait for the publisher to register
    rospy.sleep(1)

    # Create a JointTrajectory message
    trajectory_msg = FollowJointTrajectoryActionGoal()
    trajectory_msg.goal.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    # Define trajectory points
    
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, -3.25, -1.57, 0.0]  # Target positions for each joint
    point.velocities = [0.0] * 6  # No velocity, just position control
    point.time_from_start = rospy.Duration(5.0)  # Reach the goal in 5 seconds

    # Add the point to the trajectory message
    trajectory_msg.goal.trajectory.points.append(point)

    # Publish the trajectory goal
    rospy.loginfo("Sending joint trajectory goal...")
    trajectory_pub.publish(trajectory_msg)
    rospy.loginfo("Goal sent.")

if __name__ == "__main__":
    try:
        send_joint_trajectory_goal()
    except rospy.ROSInterruptException:
        pass
