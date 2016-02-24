#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from optparse import OptionParser

def publish_cup_prepose(cup_position, maker_position, maker_frame_id):
    rospy.init_node('publish_cup_prepose', anonymous=False)
    pub = rospy.Publisher('~goal', PoseStamped, queue_size=1)
    rospy.sleep(.3)
    if not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = maker_frame_id
        msg.pose.position.x = cup_position[0] - maker_position[0]
        msg.pose.position.y = cup_position[1] - maker_position[1]
        msg.pose.position.z = cup_position[2] - maker_position[2]
        msg.pose.orientation.w = 1.0
        pub.publish(msg)

if __name__ == '__main__':
    usage = "usage: %prog -c x_cup y_cup z_cup -m x_maker y_maker z_maker"
    parser = OptionParser(usage)
    parser.add_option("-c", "--cup", type="float", nargs=3, dest="cup",
            default=[0.0, 0.0, 0.0], help="Position of the cup bottom in world/map frame.")
    parser.add_option("-m", "--maker", type="float", nargs=3, dest="maker",
            default=[0.0, 0.0, 0.0], help="Position of the maker bottom in world/map frame.")
    parser.add_option("-f", "--frame_id", type="string", dest="frame_id",
            default="maker_frame", help="Frame_id in which to express the prepose.")



    (options, args) = parser.parse_args()

    if len(args) != 0:
        print "Expected no extra arguments. Stopping."
    else:
        try:
            publish_cup_prepose(options.cup, options.maker, options.frame_id)
        except rospy.ROSInterruptException:
            pass
