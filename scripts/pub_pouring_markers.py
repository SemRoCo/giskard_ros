#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA

def gray():
    return ColorRGBA(0.7, 0.7, 0.7, 1.0)

def dark_gray():
    return ColorRGBA(0.5, 0.5, 0.5, 1.0)

def opaque():
    return ColorRGBA(0.0, 0.0, 0.0, 1.0)

def transparent():
    return ColorRGBA(1.0, 1.0, 1.0, 0.0)

def white():
    return ColorRGBA(1.0, 1.0, 1.0, 1.0)

def transparent_red():
    return ColorRGBA(1.0, 0.0, 0.0, 0.7)

def leg_xy_offset():
    return 0.8/2 - 0.03/2 - 0.005

def make_marker(frame_id, id, pose, scale, color, frame_locked):
    msg = Marker()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "pouring_visualization"
    msg.id = id
    msg.action = Marker.ADD
    msg.pose = pose
    msg.scale = scale
    msg.color = color
    msg.frame_locked = frame_locked
    return msg

def make_cube_marker(frame_id, id, pose, scale, color, frame_locked=False):
    msg = make_marker(frame_id, id, pose, scale, color, frame_locked)
    msg.type = Marker.CUBE
    return msg

def make_cylinder_marker(frame_id, id, pose, scale, color, frame_locked=False):
    msg = make_marker(frame_id, id, pose, scale, color,frame_locked)
    msg.type = Marker.CYLINDER
    return msg

def make_mesh_marker(frame_id, id, pose, scale, color, path, frame_locked=False):
    msg = make_marker(frame_id, id, pose, scale, color, frame_locked)
    msg.type = Marker.MESH_RESOURCE
    msg.mesh_resource = path
    return msg

def make_table_markers():
    markers = []
    markers.append(make_cube_marker("base_footprint", 1, Pose(Point(x=0.6, z=0.72 - 0.05/2), Quaternion(w=1.0)), Vector3(0.8, 0.8, 0.05), gray()))
    markers.append(make_cube_marker("base_footprint", 2, Pose(Point(0.6+leg_xy_offset(), leg_xy_offset(), 0.67/2), Quaternion(w=1.0)), Vector3(0.03, 0.03, 0.67), dark_gray()))
    markers.append(make_cube_marker("base_footprint", 3, Pose(Point(0.6+leg_xy_offset(), -leg_xy_offset(), 0.67/2), Quaternion(w=1.0)), Vector3(0.03, 0.03, 0.67), dark_gray()))
    markers.append(make_cube_marker("base_footprint", 4, Pose(Point(0.6-leg_xy_offset(), leg_xy_offset(), 0.67/2), Quaternion(w=1.0)), Vector3(0.03, 0.03, 0.67), dark_gray()))
    markers.append(make_cube_marker("base_footprint", 5, Pose(Point(0.6-leg_xy_offset(), -leg_xy_offset(), 0.67/2), Quaternion(w=1.0)), Vector3(0.03, 0.03, 0.67), dark_gray()))
    return markers

def table_marker_publisher():
    rospy.init_node('table_marker_publisher')
    cup_on_table = rospy.get_param("~cup_on_table")
    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
    rospy.sleep(.3)
    if not rospy.is_shutdown():
        msg = MarkerArray()
        msg.markers = make_table_markers()
        if cup_on_table:
            msg.markers.append(make_mesh_marker("base_link", len(msg.markers) + 1, Pose(Point(x=0.4, z=0.71), Quaternion(z=1.0)), Vector3(1, 1, 1), white(), "package://giskard_examples/object_meshes/cup_eco_orange.dae", frame_locked=True))
        else:
            msg.markers.append(make_mesh_marker("l_gripper_tool_frame", len(msg.markers) + 1, Pose(Point(x=-0.01, z=-0.02), Quaternion(z=1.0)), Vector3(1, 1, 1), white(), "package://giskard_examples/object_meshes/cup_eco_orange.dae", frame_locked=True))
        msg.markers.append(make_mesh_marker("r_gripper_tool_frame", len(msg.markers) + 1, Pose(Point(x=-0.01, z=0.02), Quaternion(z=1.0)), Vector3(1, 1, 1), white(), "package://giskard_examples/object_meshes/sigg_bottle.dae", frame_locked=True))
#        msg.markers.append(make_cylinder_marker("cup_bottom_frame", len(msg.markers) +1, Pose(position=Point(z=0.07)), Vector3(0.085, 0.085, 0.14), transparent_red(), True))
        pub.publish(msg)

if __name__ == '__main__':
    try:
        table_marker_publisher()
    except rospy.ROSInterruptException:
        pass
