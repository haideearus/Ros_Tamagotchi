#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2

lower_red = (0, 50, 50)
upper_red = (10, 255, 255)

rospy.init_node("red_image_publisher_node")
image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
color_pub = rospy.Publisher("color", String, queue_size=10)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

bridge = CvBridge()

def image_callback(image_msg):
    image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    red_res = cv2.bitwise_and(image, image, mask=red_mask)

    red_msg = bridge.cv2_to_imgmsg(red_res, "rgb8")
    red_msg.header.stamp = image_msg.header.stamp
    image_pub.publish(red_msg)

    if cv2.countNonZero(red_mask) > 0:
        color_pub.publish("RED")

        rotate_cmd = Twist()
        rotate_cmd.angular.z = 1.0  # Set angular velocity for rotation
        cmd_vel_pub.publish(rotate_cmd)
    else:
        stop_cmd = Twist()
        cmd_vel_pub.publish(stop_cmd)

def main():
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.loginfo("Red detection on")
    rospy.spin()

if __name__ == '__main__':
    main()
