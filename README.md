# giskard_examples

## pr2_controller

Start the loopback controller:
```
roslaunch giskard_examples pr2_left_arm_loopback_controller.launch
```

Start the pr2 controller:
```
roscd giskard_examples
rosrun giskard_examples pr2_controller controller_specs/pr2_qp_position_control.yaml
```

Send your goal to `/pr2_controller/goal`, e.g.:
```
rostopic pub /pr2_controller/goal geometry_msgs/Point "x: 0.5                     
y: 0.0            
z: 0.4"
```
