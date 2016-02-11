# giskard_examples

## pr2_controller

Start the naive kinematics simulator for the PR2:
```
roslaunch iai_naive_kinematics_sim pr2.launch sim_frequency:=250
```

Open RVIZ to visualize the PR2:
```
rviz
```

Configure RVIZ:
* set FixedFrame to ```base_link```
* add RobotModel plugin
* add InteractiveMarkers plugin

Start the constraint-based pr2 controller:
```
roslaunch giskard_examples pr2_left_upper_body_qp_cartesian_position_control.launch
```

Use the interactive marker to give commands to controller controlling the left arm and the torso.
![rviz view](https://raw.githubusercontent.com/airballking/giskard_examples/master/docs/pr2_interactive_markers.png)
