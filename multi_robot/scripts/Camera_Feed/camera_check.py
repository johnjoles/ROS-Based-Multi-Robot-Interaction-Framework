#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    """Callback function to process the image from the camera."""
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the image
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("camera_opencv_node", anonymous=True)

    # Create a CvBridge instance
    bridge = CvBridge()

    # Subscribe to the camera topic (modify the topic name as per your setup)
    camera_topic = "top/camera/image_raw"
    rospy.Subscriber(camera_topic, Image, image_callback)

    rospy.loginfo("Camera node started, waiting for images...")
    
    # Keep the script alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera node.")
    finally:
        cv2.destroyAllWindows()
