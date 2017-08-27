# The Gazebo packages

These packages contain the code specific to running the robot in simulation. The
urdf from the amee package is also required.

### amee_gazebo
This package contains mostly xml, of launch files to display the robot in Gazebo
and descriptions of Gazebo worlds.

To just display the robot in simulation, without having control:
`roslaunch amee_gazebo gazebo.launch`

#### Worlds
I have created a few custom worlds to place the robot in.  The nasa_minimal is
a simplified (boxy) version of the competition arena.  For a more realistic one,
use nasa_arena. Note: this requires more resources and if you contact the walls,
you fly far away.

You can also try messing around with custom worlds. See the gazebosim tutorials.


### amee_sim_control
This package contains ros controllers that allow you to actually move the robot
in simulation.

To drive the robot base in simulation with a keyboard:
`roslaunch amee_sim_control drive.launch`

To use a controller, see the `joy_teleop.launch`. You may need to edit some
parameters.



### TODOS
- Convert the code in the controller to listen to the main teleop node in amee.
- Convert other topics to be consistent with that used in the actual robot
#### For fun
##### Create a robot game in the simulation
Create a custom world, and use existing [Gazebo plugins][occupied_plugin] or
write your own to create a robot game in simulation.  It could be a teleop setup
or you could add autonomy components, so there is some challenge to program the
robot to solve a challenge.


[plugin_101]: http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin
[occupied_plugin]: http://gazebosim.org/tutorials?tut=occupiedevent&cat=plugins
