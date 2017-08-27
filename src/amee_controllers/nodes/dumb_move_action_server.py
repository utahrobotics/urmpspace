#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import actionlib
import tf, tf2_ros

from geometry_msgs.msg import Twist
from phidgets_ros.msg import ActuatorStatesProcessed
from amee_controllers.msg import DumbMoveAction, DumbMoveFeedback, DumbMoveResult, DumbMoveGoal, DumbMoveAction, DumbMoveFeedback, DumbMoveResult

class DumbMoveServer(object):
    """Server to respond to dumb move goals, using the action interface.
    Dumb move goals are simply just an x, y, and degree transform from the map
    frame, which in our case corresponds to the center of the collector bin."""
    def __init__(self):
        # Object to store feedback
        self.feedback = DumbMoveFeedback()
        # TODO: add safety timeouts
        # self.stall_timeout = 2
        # self.linear_stall_threshold = 0.1
        # self.angular_stall_threshold = 0.1
        self.goal_tolerance = 0.05
        self.angle_tolerance = 5 * math.pi/180 # 3 deg

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.server = actionlib.SimpleActionServer("DumbMove_action", DumbMoveAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def publish_feedback(self):
        """Fill the feedback message and publish to the client"""
        #TODO
        self.server.publish_feedback(self.feedback)

    def execute_cb(self, goal):
        """Called when a new goal is accepted."""
        start_time = rospy.Time.now()
        goal.rad = goal.deg * math.pi/180

        # # Get max_speed of goal
        # if goal.max_speed == 0.0:
        #     goal.max_speed = 2.0
        # else:
        #     goal.max_speed = max(0.2, min(2.0, goal.max_speed))

        translation_done = False # x and y are finished
        done = False # x, y, and the angle (deg) change are finished
        rate = rospy.Rate(10.0) # 10hz
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                # User cancelled the request
                cmd_vel_pub.publish(Twist()) # publish 0.0 message to motor driver
                rospy.loginfo("DumbMove action: PREEMPTED")
                self.server.set_preempted()
                return

            try:
                trans = self.tfBuffer.lookup_transform('base_link', 'map', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue


            # Command the roboclaw
            cmd = Twist()

            curr_x =  trans.transform.translation.x
            curr_y =  trans.transform.translation.y
            rot = trans.transform.rotation
            curr_angle = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

            cmd.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            cmd.angular.z = max(-2.0, min(2.0, cmd.angular.z))

            cmd.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            cmd.linear.x = max(-2.0, min(2.0, cmd.linear.x))



            self.publish_feedback(start_time, "OK")

            self.server.publish_feedback(self.feedback)
            rate.sleep()


        # If we reached our goal, set succeeded. else it was a CTRL-C or crash
        if done:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()



if __name__ == '__main__':
    rospy.init_node("dumb_move_action_server")
    # rospy.init_node("dumb_move_action_server", log_level=rospy.DEBUG)
    server = DumbMoveServer()
    rospy.spin()
