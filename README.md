# giskard_examples

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Indigo``` installed on ```Ubuntu 14.04```:
```
source /opt/ros/indigo/setup.bash          # start using ROS Indigo
mkdir -p ~/giskard_ws/src                  # create directory for workspace
cd ~/giskard_ws                            # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_examples/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/giskard_ws/devel/setup.bash       # source new overlay
```
Note, the above instructions have been tested to also work under ```ROS Kinetic``` installed on ```Ubuntu 16.04```. Just replace any occurance of ```indigo``` with ```kinetic```.

## Playthings
### PR2 + Interactive Markers + Upper-Body Cartesian Position Control

* For a trial using on the real robot, run this command:

```
roslaunch /etc/ros/indigo/robot.launch                                 # on the robot
roslaunch giskard_examples pr2_interactive_markers.launch sim:=false   # on your local machine - this will start rviz too
```

* For a trial using ```iai_naive_kinematics_sim```, run this command:

```
roslaunch giskard_examples pr2_interactive_markers.launch
```

Now, wait until you see the message:

```Controller started.```

in the console.

Note: Shall you move the rviz markers before this message is printed, TF extrapolation into the past errors will be printed in the console.

* For a trial in ```gazebo simulator```, run these commands:

```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch giskard_examples pr2_interactive_markers.launch sim:=false
```

Use the interactive markers to give commands to controller controlling the both arms and the torso.
![rviz view](https://raw.githubusercontent.com/SemRoCo/giskard_examples/master/docs/pr2_interactive_markers.png)

## API of nodes
### ```controller_action_server```
Acts as a convenience interface in front of the ```whole_body_controller```. Added values is twofold: (1) offers action interface with its feedback- and cancelation-semantics, and (2) provides intelligent parsing of motion goals to meet the specific requirements of the ```whole_body_controller```.

#### Provided actions
* ```~move``` (giskard_msgs/WholeBody): Movement command to be executed. The server supports only one goal at a time, and provides the following intelligent parsing of motion goals:
  - Automatic transformation of all goal poses of type ```geometry_msgs/PoseStamped``` into the reference frame of the controller using ```TF2```.
  - Support of partial commands for body parts by using the previous commands for the respective body parts.
  - Frequent publishing of internal monitoring flags, e.g. "left arm moving" or "right arm position converged " which determine succeeded movements.

#### Called actions
* ```/tf2_buffer_server``` (tf2_msgs/LookupTransform): Connection to ```TF2``` buffer server to transform Cartesian goal poses.

#### Subscribed topics
* ```~feedback``` (giskard_msgs/ControllerFeedback): Low-level feedback from ```whole_body_controller```; used to determine when a movement goal succeeded.
* ```~current_command_hash``` (std_msgs/UInt64): Hash of command currently pursued by ```whole_body_controller```; acts as a safe-guard to synchronize ```whole_body_controller``` and ```controller_action_server```.

#### Published topics
* ```~command``` (giskard_msgs/WholeBodyCommand): Command to ```whole_body_controller```, repeated published at high frequency to kick watchdog in ```whole_body_controller```.

#### Parameters


