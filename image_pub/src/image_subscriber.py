#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # Convert the ROS Image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Save the image
        image_filename = "/root/catkin_ws/src/image_pub/capture.jpg"  # Specify the desired save path
        cv2.imwrite(image_filename, cv_image)
        rospy.loginfo("Image saved to %s", image_filename)

    except Exception as e:
        rospy.logerr("Error processing the image: %s", str(e))

def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)

    # Set the topic name to subscribe to
    image_topic = "camera/image"

    # Subscribe to the image topic
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
