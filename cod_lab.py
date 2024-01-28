#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TamagotchiBehavior:
    def __init__(self):
        self.state = "STOP"
        self.color_sub = rospy.Subscriber("color", String, self.color_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def color_callback(self, msg):
        self.state = msg.data

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.state == "BLUE":
                self.publish_move(0.5, 1.0)
                rospy.sleep(3)
                self.state = "STOP"
            elif self.state == "VIOLET":
                self.publish_move(0.0, 0.0)
                rospy.sleep(3)
                self.state = "STOP"
            elif self.state == "RED":
                self.publish_move(0.0, 1.0)
                rospy.sleep(3)
                self.state = "STOP"
            else:
                self.publish_move(0.5, 0.0)
            rate.sleep()

    def publish_move(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("master_node")
    rospy.loginfo("Master node is running")
    state_machine = TamagotchiBehavior()
    state_machine.run()
