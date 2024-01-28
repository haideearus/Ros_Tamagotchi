#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import pygame

lower_violet = (130, 50, 50)
upper_violet = (170, 255, 255)

rospy.init_node("violet_mage_publisher_node")
image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
color_pub = rospy.Publisher("color", String, queue_size=10)

bridge = CvBridge()

# Initialize Pygame
pygame.init()
melody_file = "/home/andreea/catkin_ws/src/my_robot_controler/scripts/melody.mp3"
is_color_detected = False

def play_melody():
    pygame.mixer.music.load(melody_file)
    pygame.mixer.music.play()

def stop_melody():
    pygame.mixer.music.stop()

def image_callback(image_msg):
    global is_color_detected

    image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    violet_mask = cv2.inRange(hsv, lower_violet, upper_violet)
    violet_res = cv2.bitwise_and(image, image, mask=violet_mask)

    violet_msg = bridge.cv2_to_imgmsg(violet_res, "rgb8")
    violet_msg.header.stamp = image_msg.header.stamp

    image_pub.publish(violet_msg)

    if cv2.countNonZero(violet_mask) > 0:
        if not is_color_detected:
            is_color_detected = True
            color_pub.publish("VIOLET")
            play_melody()
    else:
        if is_color_detected:
            is_color_detected = False
            stop_melody()

def main():
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.loginfo("Violet detection on")
    rospy.spin()

if __name__ == '__main__':
    main()
