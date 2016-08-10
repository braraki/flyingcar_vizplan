#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Float64
from map_maker.srv import *
from map_maker.msg import *

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

import math
import matplotlib.pyplot as plt
import time
import random

import networkx as nx
from enum import Enum
import numpy as np

import tf

import thread

from map_maker import gen_adj_array_info_dict


import rosbag


path_info_list = []

bag = rosbag.Bag('recorder_test.bag', 'w')







def record_path(data):
	global path_info_list
	num_IDs = data.num_IDs
	ID = data.ID
	path = data.path
	times = data.times
	planning_time = data.planning_time
	info = (num_IDs, ID, path, times, planning_time)
	if info not in path_info_list:
		path_info_list.append(info)
		path_num = len(path_info_list)
		if path_num == 1:
			#print(path_info_list)
			i = HiPathTime(num_IDs, ID, path, times, planning_time)
			bag.write('Path_'+str(path_num), i)
			bag.close()
	if rospy.is_shutdown():
		bag.close()







def runner():
	rospy.Subscriber('~time_path_topic', HiPathTime, record_path)
	#print('in runner')
	rospy.spin()










if __name__ == "__main__":
	rospy.init_node("recorder")
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	runner()

