#!/usr/bin/env python
import rospy
import requests
from geometry_msgs.msg import Twist

URL = "http://usr.coe.utah.edu:3100/averages_and_reset"
  

def read():
    rospy.init_node('move_reader', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #read script one line at a time
    script = open("/home/alex/catkin_ws/src/turtlesim_reader/src/move_script.txt", "r")
    lines = script.readlines()
    while not rospy.is_shutdown():
	r = requests.get(url = URL)
	data = r.json()
	print(data)
	vel_msg.linear.x = data[2] - data[3]
    	vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
	vel_msg.angular.z = data[0]-data[1]

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
