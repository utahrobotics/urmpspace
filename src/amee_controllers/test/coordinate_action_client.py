#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import actionlib
import sys

from std_msgs.msg import Float32
from amee_controllers.msg import CoordinateAction, CoordinateFeedback, CoordinateResult, CoordinateGoal


def feedback_cb(feedback):
    pass
    # print(feedback)

def position_client(arm, bucket, sled, cancel=False):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('coordinate_action', CoordinateAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waitign for server")
    client.wait_for_server()
    print("done waiting for server")
    # Creates a goal to send to the action server.
    goal = CoordinateGoal(arm=arm, bucket=bucket, sled=sled)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # client.feedback_cb =
    if cancel:
        decision = raw_input("Cancel Y/n?")
        if not decision.startswith('n'):
            client.cancel_goal()

    print("waiting for result...")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        try:
            arm = float(sys.argv[1])
            bucket = float(sys.argv[2])
            sled = float(sys.argv[3])
        except:
            arm = 100.0
            bucket = 100.0
            sled = 100.0

        cancel = len(sys.argv) < 5
        print(len(sys.argv))
        print(cancel)

        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('position_action_client')
        result = position_client(arm, bucket, sled, cancel)
        print(result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    print("done")
    rospy.spin()
