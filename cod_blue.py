#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2

lower_blue = (110, 50, 50)
upper_blue = (130, 255, 255)

rospy.init_node("blue_image_publisher_node")
image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
color_pub = rospy.Publisher("color", String, queue_size=10)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

bridge = CvBridge()

def image_callback(image_msg):
    image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_res = cv2.bitwise_and(image, image, mask=blue_mask)
    
    blue_msg = bridge.cv2_to_imgmsg(blue_res, "rgb8")
    blue_msg.header.stamp = image_msg.header.stamp
    image_pub.publish(blue_msg)

    if cv2.countNonZero(blue_mask) > 0:
        color_pub.publish("BLUE")

        # Perform play behavior (adjust the movement commands as needed)
        play_cmd = Twist()
        play_cmd.linear.x = 0.5  # Move forward
        play_cmd.angular.z = 1.0  # Rotate
        cmd_vel_pub.publish(play_cmd)
    else:
        # Stop the robot when not detecting blue color
        stop_cmd = Twist()
        cmd_vel_pub.publish(stop_cmd)

def main():
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.loginfo("BLUE detection on")
    rospy.spin()

if __name__ == '__main__':
    main()
