# giskard_examples

## pr2_controller

Start the naive kinematics simulator for the PR2:
```
roslaunch iai_naive_kinematics_sim pr2.launch
```

Open RVIZ to visualize the PR2:
```
rviz
```

Start the constraint-based pr2 controller:
```
roslaunch giskard_examples pr2_left_upper_body_qp_cartesian_position_control.launch
```

Send your goal to the PR2 controller, e.g.:
```
rostopic pub /pr2_controller/goal geometry_msgs/Point "x: 0.5                     
y: 0.0            
z: 0.4"
```
