#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState

def cmd_publisher():
    rospy.init_node('cmd_pub', anonymous=True)
    pub = rospy.Publisher('/joint_state_separator/joint_states', JointState, queue_size=1)
    rospy.sleep(.3)
    if not rospy.is_shutdown():
        msg = JointState()
        msg.name.append("torso_lift_joint")
        msg.name.append("l_shoulder_pan_joint")
        msg.name.append("l_shoulder_lift_joint")
        msg.name.append("l_upper_arm_roll_joint")
        msg.name.append("l_elbow_flex_joint")
        msg.name.append("l_forearm_roll_joint")
        msg.name.append("l_wrist_flex_joint")
        msg.name.append("l_wrist_roll_joint")
        msg.name.append("r_shoulder_pan_joint")
        msg.name.append("r_shoulder_lift_joint")
        msg.name.append("r_upper_arm_roll_joint")
        msg.name.append("r_elbow_flex_joint")
        msg.name.append("r_forearm_roll_joint")
        msg.name.append("r_wrist_flex_joint")
        msg.name.append("r_wrist_roll_joint")
        for i in range(len(msg.name)):
            msg.velocity.append(0.1 * i)
        pub.publish(msg)

if __name__ == '__main__':
    try:
        cmd_publisher()
    except rospy.ROSInterruptException:
        pass
