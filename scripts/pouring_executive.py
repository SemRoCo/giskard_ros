#!/usr/bin/env python

import yaml
import rospy
import rospkg
from std_msgs.msg import String, Empty, Float64MultiArray

buffer = []
max_buffer = 10
threshold = 0.05
joint_names = ['torso_lift_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
flag = False

def callback(data):
    # calculate next value
    val = True
    for vel_cmd in data.data:
        if abs(vel_cmd) > abs(threshold):
            val = False
            break

    # update buffer
    if len(buffer) == max_buffer:
        buffer.pop(0)
    buffer.append(val)

    # make decision
    next_flag = True
    if len(buffer) < max_buffer:
        next_flag = False
    else:
        for val in buffer:
            if not val:
                next_flag = False
                break

    # set flag
    global flag
    flag = next_flag


def prepare_decider(thresh, buffer_size):
    rospy.loginfo("threshold=%f, buffer-size=%d", thresh, buffer_size)
    global buffer
    buffer = []
    global flag
    flag = False
    global threshold
    threshold = thresh
    global max_buffer
    max_buffer = buffer_size

def read_and_pub_yaml(pub, path):
    if not rospy.is_shutdown():
        msg = String()
        with open(path, 'r') as myfile:
                msg.data=myfile.read()
        rospy.loginfo("publishing goal: %s", path)
        pub.publish(msg)

def trigger_motion(pub, path, thresh, buffer_size):
    prepare_decider(thresh, buffer_size)
    read_and_pub_yaml(pub, path)

def wait_for_flag():
    r = rospy.Rate(10) # 10hz
    while (not rospy.is_shutdown()) and (not flag):
        r.sleep()

def pouring_executive(path, controller_specs):
    rospy.init_node('pouring_executive', anonymous=False)
    pub = rospy.Publisher('/yaml_controller/goal', String, queue_size=1)
    empty_pub = rospy.Publisher('/pouring_executive/finished', Empty, queue_size=1)
    rospy.Subscriber("/yaml_controller/cmd", Float64MultiArray, callback)
    rospy.sleep(.3)
    for controller_spec in controller_specs:
        trigger_motion(pub, path + controller_spec["controller-file"], 0.05, controller_spec["max-twist-buffer-size"] / 10)
        wait_for_flag()
    empty_pub.publish(Empty())
    rospy.loginfo("DONE")

if __name__ == '__main__':
    args = rospy.myargv()
    print(args)
    if len(args) == 2:
        experiment_num = args[1]
        r = rospkg.RosPack()
        package_path = r.get_path('giskard_ros')
        experiment_path = "/experiments/" + experiment_num + "/"
        path = package_path + experiment_path
        with open(path + "PR2_experiment.yaml", 'r') as stream:
            try:
                controller_specs = yaml.load(stream)["controller-specs"]
                try:
                    pouring_executive(path, controller_specs)
                except rospy.ROSInterruptException:
                    pass
            except yaml.YAMLError as exc:
                print(exc)
