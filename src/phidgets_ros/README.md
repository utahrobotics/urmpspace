
## Prerequisites
**Important**: For this package, you must install both the
[C libraries for Phidgets](http://www.phidgets.com/docs/OS_-_Linux) and
[Python Phidgets module](http://www.phidgets.com/docs/Language_-_Python).

You will also likely need to set the udev rules, so you can have permissions
for the devices. See http://www.phidgets.com/docs/OS_-_Linux#Setting_udev_Rules.
The best way is to sudo cp the file from the udev folder in the Phidgets library
(libphidgets) to your `/etc/udev/rules.d`

Once you have changed it, reload the file:<br>
`sudo udevadm control --reload-rules && udevadm trigger`

Here is the API for the Phigets library, if you are curious:
http://www.phidgets.com/documentation/web/PythonDoc/Phidgets.Devices.html


I have also included the example code that was included in the Phidgets Python
library for the Bridge, Encoder, and InterfaceKit (`phidgets_example_code`).  These
were used as baselines for the node, but none of the code in here runs on the robot.

----


## ROS Nodes
### phidgets_node
The phidgets node contains code for "attaching" all the Phidgets devices, [Wheatstone Bridge Board][bridge], [4 Input Encoder Board][encoder], and [InterfaceKit Board][ik].

This node handles collecting the actuator sensor data from the Phidgets boards and publishing it to ROS messages so they
can be used for sensing where the robot actuators are (see [state_publisher in amee][state_pub]) and for closed loop control (see [amee_controllers package][controllers]).

The [Interface Kit][ik] reads the potentiometer value from the linear actuators (bucket and arms), which tells you how far the linear actuator is extended.

The [Encoder board][encoder] reads the encoder values from the idler gears mounted on the sled to measure how much sled has moved.
To get the absolue position, the sled must first be "homed", by driving it to contact the rear limit switch.  This homed
position is 0.0 for the position and ~300 (mm) is the front.

The code for the [Wheatstone bridge][bridge] has been started and commented out.  The Wheatstone Bridge
is meant to take in reading from the load cells, which measure the amount of strain in the arms.
In principle, this can give us data about how full our bucket is.  This is code that still needs
be worked on.  New members? :)


We used millimeters in this package, instead of the [standard unit of meters][rep].  This was chosen because the linear
actuators values are in this range, and the numbers were easy to work with.  It would probably be better to stick to m in the future.


#### Subscribed topics

`/pololu/limit_switch` ([pololu_ros/LimitSwitch](https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/pololu_ros/msg/LimitSwitch.msg)<br>

When the rear limit switch is triggered (bool true), the sled is considered homed and is set to the 0 position.

#### Published topics
`/actuator_states/raw/` ([phidgets_ros/ActuatorStates](https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/phidgets_ros/msg/ActuatorStates.msg))<br>
Sensed states of all the acutators.  Arms and bucket measurements are from
linear actuator potentiometers.  The sled is based on encoder data, with a required homing
sequence of driving the sled all the way to the back to trigger the rear limit
switch.  If this is not triggered, the node publishes a NaN value for the sled.


`/actuator_states/proc/` ([phidgets_ros/ActuatorStatesProcessed](https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/phidgets_ros/msg/ActuatorStatesProcessed.msg))<br>
Same information as ActuatorStates, but with averaged values for the linear actuators, so that
the message only contains 3 total float values for the arm, bucket, and sled positions.


#### Parameters

See [config.yaml file](https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/phidgets_ros/config/config.yaml)


[bridge]: http://www.phidgets.com/docs/1046_User_Guide
[encoder]: http://www.phidgets.com/docs/1047_User_Guide
[ik]: http://www.phidgets.com/docs/1018_User_Guide
[state_pub]: https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/amee/src/state_publisher.cpp
[controllers]: https://github.com/jrobe/robotic-mining/tree/Kinetic/urmpspace/src/amee_controllers
[rep]: http://www.ros.org/reps/rep-0103.html
