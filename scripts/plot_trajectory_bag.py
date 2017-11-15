#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import argparse

parser = argparse.ArgumentParser(description='Plot a trajectory goal of type "control_msgs/FollowJointTrajectoryGoal" '
                                             'from a ROS bag into a PDF file.')
parser.add_argument('filename', metavar='BAG', help='filename of the input ROS bag file')
parser.add_argument('-p', metavar='PDF', help='filename of the output PDF file',
                    default='output.pdf', dest='output')
parser.add_argument('-t', metavar='TOPIC', help='name of the topic to be extracted from the ROS bag',
                    default='/whole_body_controller/follow_joint_trajectory/goal', dest='topic')
args = parser.parse_args()

# read message into a map for easier navigation
bag = rosbag.Bag(args.filename)
trajectories = []
for topic, msg, t in bag.read_messages(topics=[args.topic]):

    trajectory = {}
    time_points = []
    for name in msg.goal.trajectory.joint_names:
        trajectory[name] = {}
        trajectory[name]['pos'] = []
        trajectory[name]['vel'] = []

    for trajectory_point in msg.goal.trajectory.points:
        time_points.append(trajectory_point.time_from_start.to_sec())
        for name, pos, vel in zip(msg.goal.trajectory.joint_names, trajectory_point.positions, trajectory_point.velocities):
            trajectory[name]['pos'].append(pos)
            trajectory[name]['vel'].append(vel)

    trajectories.append(trajectory)
bag.close()

# generate the output PDF from the trajectory map
# only use the first trajectory, for the moment
with PdfPages(args.output) as pdf:
    for joint_name in trajectories[0].keys():
        plt.subplot(2, 1, 1)
        plt.plot(time_points, trajectory[joint_name]['pos'])
        plt.title(joint_name)
        plt.ylabel('position [m] or [rad]')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.plot(time_points, trajectory[joint_name]['vel'])
        plt.ylabel('velocity [m/s] or [rad/s]')
        plt.xlabel('time [s]')
        plt.grid()
        pdf.savefig()
        plt.close()