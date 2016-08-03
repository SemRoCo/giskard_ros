#!/usr/bin/env python

# 
# Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
# 
# 
# This file is part of giskard_examples.
# 
# giskard_examples is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 
# of the License, or (at your option) any later version.  
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License 
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# 

import rospy
from giskard_msgs.msg import WholeBodyCommand, ControllerFeedback
from multiprocessing import Process
import matplotlib.pyplot as plt

def init_feedbacks(keys):
    feedbacks = {}
    for key in keys:
        feedbacks[key] = []
    return feedbacks

keys = ["l_trans_error", "l_rot_error", "r_trans_error", "r_rot_error"] 
feedbacks = init_feedbacks(keys)
processes = []

def plot_graph(data):
    data_copy = dict(data)
    min_plot_length = 100
    if (len(data_copy[keys[0]]) > min_plot_length and len(data_copy[keys[1]]) > min_plot_length):
        plt.figure()
        plt.subplot(211)
        plt.ylabel("left arm control errors")
        position, = plt.plot(data_copy[keys[0]], 'r', label="position")
        rotation, = plt.plot(data_copy[keys[1]], 'b', label="orientation")
        plt.legend(handles=[position, rotation], loc="best")
    if (len(data_copy[keys[2]]) > min_plot_length and len(data_copy[keys[3]]) > min_plot_length):
        plt.subplot(212)
        plt.ylabel("right arm control errors")
        position, = plt.plot(data_copy[keys[2]], 'r', label="position")
        rotation, = plt.plot(data_copy[keys[3]], 'b', label="orientation")
        plt.legend(handles=[position, rotation], loc="best")
    plt.show()
    plt.close()


def feedback_callback(data):
    for d in data.doubles:
        for key in keys:
            if d.semantics == key:
                feedbacks[key].append(d.value)

def command_callback(data):
    global feedbacks
    p = Process(target=plot_graph, args=(feedbacks,))
    p.start()
    processes.append(p)
    feedbacks = init_feedbacks(keys)

def start_plotter():
    rospy.init_node('whole_body_plot', anonymous=False)
    rospy.Subscriber("/whole_body_controller/feedback", ControllerFeedback, feedback_callback)
    rospy.Subscriber("/whole_body_controller/current_command", WholeBodyCommand, command_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_plotter()
    except rospy.ROSInterruptException:
        pass
    for p in processes:
        p.join()
