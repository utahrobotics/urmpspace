# Package overview

This packages contains higher level controllers for the robot, allowing users
to command the robot actuators to certain positions, while enforcing limits.

These use the more complicated and powerful ROS inteface: [actionlib][ros_action].
If you are familiar with ROS services, actions are similar but also allow you
to give feedback on the goal.  If you are not familiar with either of these,
it is kind of like you are calling a specific function on the action server
instead of the normal fire and forget publishing.  (It is still implemented
as topics and be called that way, but that is too much detail for now).
[Read the documentation][ros_action]

The nodes folder contains all of the action servers (written in python).
The action folder contains action files, similar to the msg or srv files
if you are familiar with those.

The test folder contains various action clients for testing.  You can check these
to test and figure out how the action servers work or to base your own clients
off of, so you write code to call the servers. There is also some other crap
in the test folder.


# Nodes

## competition_smach
The competition_smach node is the unfinished top-level node for running the
autonomous competition run. It implements a [finite state machine (FSM)][fsm]
using the ros [smach] package.

It reads a sequence of states from the mining_params.yaml file to generate
goals and state information.  So you can edit the sequence of states and what
the positions are, just by editting a yaml file.

See the smach_proto.png for the overall layout look.

![This was the dream](https://github.com/utahrobotics/usr_ws/blob/kinetic-devel/src/amee_controllers/smach_proto.png)


## position_action_server
Send a position command to the arm, bucket, or sled.  This controller will attempt
to drive it to the desired position, using feedback of the current state from
the phidgets node.

**NOTE:** This controller uses a goal.pos of 0.0 to indicate that this actuator
should not move at all.  This is convenient for chaining sequences of position
commands together in the coordinate action server.

#### Action API
Namespaces: `arm_position_action`, `bucket_position_action`,`sled_position_action`

Action goal<br>
`pos` (std_msgs/Float32)<br>
Position to drive linear actuator to. If 0, do not move actuator.

## coordinate_action_server
Send positions for the arm, bucket, and sled to move to. This implicitly enforces
limits expressed in the `config/control.yaml` file.  It also enforces further
safety limits on the positions of the arm and sled, so that the arm does not run
into the treads.  This safety limit is configured inside of the node, using a
polyfit quadratic function on data collected by manually driving the actuators
to positions that were on the border of being safe positions.

To command an actuator to stay in its previous position, send a command of 0.0.
Since the default of an initialized goal is 0.0, you can also just not set
the actuator you want to stay still.

#### Action API
Namespace:`coordinate_action`

Action goal<br>
`arm` (std_msgs/Float32)<br>
Position to drive arm to. If 0 or not set, do not move arm.<br>
`bucket` (std_msgs/Float32)<br>
Position to drive bucket to. If 0 or not set, do not move bucket.<br>
`sled` (std_msgs/Float32)<br>
Position to drive sled to. If 0 or not set, do not move sled.

<!-- # The config files -->

# Config

The control.yaml contains safety and control parameters for the position and
coordinate action servers.  mining params contain information about the
sequence of the states and what they are.  This allows easy configuration.


# About [smach]

To view the current state of the smach, run:
`rosrun rqt_smach rqt_smach`
NOTE: If you are running this on your own smach, you must implement the
introspection server, see competition_smach.py.

smach is a ROS package that allows you to create state machines, good for
linking together several actions and behaviors in sequence for more complicated
autonomous behavior.  It was used by Willow Garage in their milestone with the
PR2, including being able to [plug itself in][pr2_plugs].

The code for our state machine lives in the `competition_smach.py` file.  The
top level consists of a [Concurrence container][conc] for autonomy and a MonitorState
for teleop.  The Concurrence container allows us to run a MonitorState at the
same time as the main autonomy path.  We listen to the `/teleop_toggle` topic,
and if anything is published to this topic, we abort all of the autonomy
sequence and enter teleop mode.


The most important state is the `autonomy_sm`, which handles the main sequence.
The main control is handled through [Coordinate actions](#coordinate_action_server),
and [MoveBase actions][move_base].  The parameters for each of the of these
actions (where they move the arms or drive the base).

Instructions for killing:
Publish two messages to `/teleop_toggle`, then CTRL-C the node.


TODO:
Once full state is implemented, grab a png of the full machine.



[ros_action]: http://wiki.ros.org/actionlib
[conc]: http://wiki.ros.org/smach/Tutorials/Concurrence%20container
[smach]: http://wiki.ros.org/smach
[pr2_plugs]: https://github.com/PR2/pr2_plugs
[move_base]: http://wiki.ros.org/move_base
[fsm]: https://en.wikipedia.org/wiki/Finite-state_machine
