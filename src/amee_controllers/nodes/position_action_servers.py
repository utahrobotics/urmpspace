#!/usr/bin/env python
from __future__ import division
import rospy
import actionlib

from std_msgs.msg import Float32
from phidgets_ros.msg import ActuatorStatesProcessed
from amee_controllers.msg import ActuatorPosAction, ActuatorPosFeedback, ActuatorPosResult, ActuatorPosGoal


class PositionControllerServer(object):
    """Commands the actuators to certain states by listening to commands
    and actuator states and publishing to velocity. Enforces limit positions.
    """
    def __init__(self, name):
        self.name = name # must be 'arm', 'bucket', or 'sled'
        # Objects to store feedback and results
        self.feedback = ActuatorPosFeedback()
        self.result = ActuatorPosResult()

        # Get paramters from ROS
        params = rospy.get_param("/control")
        self.sensor_timeout = params["sensor_timeout"] # time to wait for sensor before pausing action
        self.stall_threshold = params["stall_threshold"] # time to wait to move beyond stall threshold value
        self.stall_timeout = params["stall_timeout"] # mm to move in the stall timeout time

        # Get the limits for the current device
        self.limits = params[self.name]
        self.low_lim = self.limits["low"]
        self.high_lim = self.limits["high"]
        self.goal_range = self.limits["goal_range"]

        # Currently only p and i are implemented. The system is quiet damped, so
        # we probably will only need p
        self.p = self.limits["pid"]["p"]
        self.i = self.limits["pid"]["i"]
        self.d = self.limits["pid"]["d"]


        # Initliaze member variables
        self.encoders_are_publishing = False # true if we are hearing from the actuator state publisher
        self.aborted = False # in trying to move the actuator, there was a safety timeout and we stopped so the robot wouldn't blow up

        # Try to get the start position of the device
        try:
            actuator_states = rospy.wait_for_message("/actuator_states/proc", ActuatorStatesProcessed, timeout=1)
            self.current_pos = self.get_sensor_val(actuator_states)
            self.previous_pos = self.current_pos
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr("Phidget node is not publishing!")
            self.current_pos = 0


        self.sensor_callback_time = rospy.Time.now() - rospy.Duration(self.sensor_timeout) # so the thing doesn't move without seeing a message
        self.actuator_pub = rospy.Publisher("/%s/vel" % self.name, Float32, queue_size=10)
        self.state_sub = rospy.Subscriber("/actuator_states/proc", ActuatorStatesProcessed, self.state_callback, queue_size=10)
        self.server = actionlib.SimpleActionServer(self.name+'_position_action', ActuatorPosAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def __del__(self):
        """On exit, stop the motor"""
        self.actuator_pub.publish(0.0)

    def get_sensor_val(self, actuator_states):
        """Process the interface sensor message to average and get the positions
        of the motors"""
        if self.name == "arm":
            val = actuator_states.arm
        elif self.name == "bucket":
            val = actuator_states.bucket
        elif self.name == "sled":
            val = actuator_states.sled
        else:
            raise Exception("Name must be arm, bucket, or sled")
        return val

    def state_callback(self, actuator_states):
        """"Callback for actuator states published"""
        self.sensor_callback_time = rospy.Time.now()
        self.current_pos = self.get_sensor_val(actuator_states)
        rospy.logdebug("%s state callback pos: %.2f", self.name, self.current_pos)

    def safety_callback(self, event):
        """Track how fast the actuator is moving. If it does not move a certain
        distance in the expected time, then something probably got jammed or
        is caught on something and we should abort"""
        try:
            if self.encoders_are_publishing:
                if abs(self.current_pos - self.previous_pos) < self.stall_threshold:
                    rospy.logdebug("Safety callback safety abort. Haven't moved more than the threshold")
                    self.safety_abort()
        except Exception as e:
            rospy.logerr(e)
        self.previous_pos = self.current_pos

    def fill_result(self, exit_message):
        """Fill all the fields of the result message to return to the user"""
        self.result.end_pos = self.current_pos
        self.result.exit_message = exit_message
        self.result.total_time = self.feedback.runtime


    def safety_abort(self):
        """Stop the motors, log the error, set the exit message, and abort"""
        self.actuator_pub.publish(0.0)
        self.aborted = True

        abort_string = '{} action ABORTED. SAFETY TIMEOUT. Did not move at least {} mm in {} seconds'.format(self.name, self.stall_threshold, self.stall_timeout)
        rospy.logerr(abort_string)

        self.fill_result(abort_string)
        self.encoders_are_publishing = False
        self.server.set_aborted(result=self.result)

    def execute_cb(self, goal):
        """Called when a new goal is accepted. Move the actuator to the given
        goal position by driving the output proportional to how far you are from
        the goal. Also checks the actuator limits and will abort if the robot
        is taking too long to move, which is likely if we get into a state
        of failure (inflection point)"""
        start_time = rospy.Time.now()
        self.feedback.status_message = "OK"
        self.aborted = False

        # The goal position, 0.0, should keep the actuator exactly where it was (no movement)
        if goal.pos == 0.0:
            self.actuator_pub.publish(0.0)
            self.fill_result("Default goal 0.0. Did not move actuator.")
            self.server.set_succeeded(result=self.result)
            return

        if goal.pos > self.high_lim:
            limit_message = "Goal exceeded upper limit, clipped to {}".format(self.high_lim)
            self.feedback.status_message = limit_message
            rospy.logwarn(limit_message)
            goal.pos = self.high_lim

        if goal.pos < self.low_lim:
            limit_message = "Goal exceeded lower limit, clipped to {}".format(self.low_lim)
            self.feedback.status_message = limit_message
            rospy.logwarn(limit_message)
            goal.pos = self.low_lim

        safety_timer = rospy.Timer(rospy.Duration(self.stall_timeout), self.safety_callback)

        # integral and derivative control for PID
        # (not currently used because the gains in the config file are set to 0)
        integral = 0
        derivative = 0
        error_last = goal.pos - self.current_pos

        r = rospy.Rate(10) # 10hz
        dt = 1/10
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                # User cancelled the request
                self.actuator_pub.publish(0.0)
                rospy.loginfo('%s action: PREEMPTED', self.name)
                self.fill_result("Goal cancelled by user")
                self.server.set_preempted(result=self.result)
                break

            # If the safety timeouts were reached
            if self.aborted:
                break

            self.feedback.runtime = (rospy.Time.now() - start_time).to_sec()
            self.feedback.current_pos = self.current_pos
            self.feedback.status_message = "OK"

            rospy.logdebug("goal: %.2f   feedback : %.2f", goal.pos, self.feedback.current_pos)
            debug_str = "time diff : {}".format(rospy.Time.now() - self.sensor_callback_time)
            rospy.logdebug(debug_str)

            if (rospy.Time.now() - self.sensor_callback_time).to_sec() > self.sensor_timeout:
                self.actuator_pub.publish(0.0)
                self.feedback.status_message = "Encoders have not published in awhile"
                rospy.loginfo(self.feedback.status_message)
                self.server.publish_feedback(self.feedback)
                self.encoders_are_publishing = False
                r.sleep()
                continue

            self.encoders_are_publishing = True

            debug_str = "pos range: {}".format(self.current_pos + self.goal_range)
            rospy.logdebug(debug_str)

            if goal.pos > (self.current_pos + self.goal_range) or goal.pos < (self.current_pos - self.goal_range):
                error = goal.pos - self.current_pos
                output = self.p * error + self.i * integral + self.d * derivative
                rospy.logdebug("raw_output: {}".format(output))
                output = max(-1, min(1, output))
                self.actuator_pub.publish(output)
                integral += error * dt
                derivative = (error - error_last) / dt
                error_last = error
                rospy.logdebug("integral: {} derivative: {} error: {} output: {}".format(integral, derivative, error, output))
            else:
                self.actuator_pub.publish(0.0)
                rospy.loginfo('%s action: succeeded' % self.name)
                # wait for phidgets node to give you the last position
                try:
                    actuator_states = rospy.wait_for_message("/actuator_states/proc", ActuatorStatesProcessed, timeout=1)
                except:
                    rospy.logerr("Phidget node is not publishing!")

                self.fill_result(self.feedback.status_message)
                self.server.set_succeeded(result=self.result)
                break

            error_last = error
            self.server.publish_feedback(self.feedback)
            r.sleep()

        self.actuator_pub.publish(0.0)
        safety_timer.shutdown() # stop checking the safety movement limits when we are done with the goal


if __name__ == '__main__':
    rospy.init_node("position_controller_action_server")
    #rospy.init_node("position_controller_action_server", log_level=rospy.DEBUG)
    server = PositionControllerServer("arm")
    server = PositionControllerServer("bucket")
    server = PositionControllerServer("sled")
    rospy.spin()
