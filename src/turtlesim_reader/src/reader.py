#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def read():
    rospy.init_node('move_reader', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #read script one line at a time
    script = open("/home/alex/catkin_ws/src/turtlesim_reader/src/move_script.txt", "r")
    lines = script.readlines()
    while not rospy.is_shutdown():
        for line in lines:
            if line.strip('\n').lower() == "forward":
                vel_msg.linear.x = 1
                vel_msg.angular.z = 0
            elif line.strip('\n').lower() == "backward":
                vel_msg.linear.x = -1
                vel_msg.angular.z = 0
            elif line.strip('\n').lower() == "left":
                vel_msg.angular.z = 0.75
                vel_msg.linear.x = 0
            elif line.strip('\n').lower() == "right":
                vel_msg.angular.z = -0.75
                vel_msg.linear.x = 0
            else:
                rospy.loginfo(line)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            rospy.loginfo(vel_msg)

            t0 = rospy.Time.now().to_sec()
            current_distance = 0
            while current_distance < 1:
                velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_distance = (t1-t0)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        read()
    except rospy.ROSInterruptException: pass
