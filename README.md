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

![rviz view](https://raw.githubusercontent.com/SemRoCo/giskard_examples/master/docs/pr2_interactive_markers.png)

* For a trial using on the real robot, run this command:

```
roslaunch /etc/ros/indigo/robot.launch                                 # on the robot
roslaunch giskard_examples pr2_interactive_markers.launch sim:=false   # on your local machine - this will start rviz too
```

* For a trial using ```iai_naive_kinematics_sim```, run this command:

```
roslaunch giskard_examples pr2_interactive_markers.launch
```

* For a trial in ```gazebo simulator```, run these commands:

```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch giskard_examples pr2_interactive_markers.launch sim:=false
```

Use the interactive markers to give commands to controller controlling the both arms and the torso.

Additionally, there is a test-client which sends a sequence of goals to the upper-body-controller. You can start it by typing:

```
roslaunch giskard_examples pr2_test_action_server.launch
```
The test-client alternates joint goals and Cartesian goals for each of the arms. So, it is a great reference to see which type of command can be send to the action interface.

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

#### Published topics
* ```~command``` (giskard_msgs/WholeBodyCommand): Command to ```whole_body_controller```, repeated published at high frequency to kick watchdog in ```whole_body_controller```.

#### Parameters
Several threshold parameters determine when a motion succeeded:
* ```~thresholds/motion_old``` (Double): Duration (in seconds) required after which a motion goal is considered old.
* ```~thresholds/bodypart_moves/left_arm``` (Double): Speed threshold (in rad/s) for the fastest joint of the left arm to consider it moving.
* ```~thresholds/bodypart_moves/right_arm``` (Double): Speed threshold (in rad/s) for the fastest joint of the right arm to consider it moving.
* ```~thresholds/bodypart_moves/torso``` (Double): Speed threshold (in rad/s) for the torso joint to consider it moving.

A set of identifiers are used to associated the feedback from the ```whole_body_controller``` with the respective body parts.
* ```~body_controllables/left_arm``` (List of Strings): Names of controllable variables that form the left arm of the robot.
* ```~body_controllables/right_arm``` (List of Strings): Names of controllable variables that form the right arm of the robot.
* ```~body_controllables/torso``` (String): Name of controllable variable making up the torso of the robot.

Finally, there are a couple of parameters which influence the behavior of the node:
* ```~frame_id``` (String): Reference frame (known to ```TF```) into which all Cartesian goal poses shall be transformed.
* ```~l_frame_id``` (String): Reference frame (known to ```TF```) of the left end-effector for Cartesian commands.
* ```~r_frame_id``` (String): Reference frame (known to ```TF```) of the right end-effector for Cartesian commands.
* ```~update_period``` (Double) Time (in seconds) between updates, i.e. feedback publishes to client and commands publishes to ```whole_body_controller```.
* ```~server_timeout``` (Double): Time (in seconds) after which the server aborts in case it fails to initialize its action interface.


