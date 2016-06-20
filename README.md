# giskard_examples

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Indigo```:
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
## Examples
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
