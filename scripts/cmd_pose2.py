#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

def pose_publisher():
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('pose_stamped_transformer/in', PoseStamped, queue_size=1)
    rospy.sleep(.3)
    if not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = 0.644
        msg.pose.position.y = 0.079
        msg.pose.position.z = 1.152
        msg.pose.orientation.x = 0.755
        msg.pose.orientation.y = -0.552 
        msg.pose.orientation.z = -0.303
        msg.pose.orientation.w = -0.183
        pub.publish(msg)

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
