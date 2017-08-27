#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import actionlib
import sys

from amee_controllers.msg import DumbMoveAction, DumbMoveFeedback, DumbMoveResult, DumbMoveGoal


def feedback_cb(feedback):
    print(feedback)


def position_client(x, y, deg, cancel=False):
    print('dumb_move_action')
    client = actionlib.SimpleActionClient('dumb_move_action', DumbMoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waitign for server")
    client.wait_for_server()
    print("done waiting for server")
    # Creates a goal to send to the action server.
    goal = DumbMoveGoal(x=x, y=y, deg=deg)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

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
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            deg = float(sys.argv[3])
        except:
            x = 8
            y = 8
            deg = 0

        cancel = len(sys.argv) < 5
        print(sys.argv)

        rospy.init_node('dumb_move_action_client')
        result = position_client(x, y, deg, cancel)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

    print("done")
    rospy.spin()
