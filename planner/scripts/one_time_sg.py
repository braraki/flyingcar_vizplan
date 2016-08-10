#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
from map_maker.msg import *
from planner.srv import *

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

import math
import matplotlib.pyplot as plt
import time
import random

import csv
import networkx as nx
from enum import Enum
import numpy as np

class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}

park_dict = {}

def generate_passangers(cf_num):
	global park_dict
	start_list = []
	end_list = []
	for p in range(cf_num):
		start = random.choice(park_dict.keys())
		end = random.choice(park_dict.keys())
		start_list.append(start)
		end_list.append(end)
	return((start_list, end_list))

def response(req):
	(start_list, end_list) = generate_passangers(req.cf_num)
	return situationResponse(start_list, end_list)

def info_sender():
	s = rospy.Service('send_situation', situation, response)
	print('ready to send info back')
	rospy.spin()

def map_maker_client():
	global park_dict
	rospy.wait_for_service('send_map')
	try:
		print('calling')
		func = rospy.ServiceProxy('send_map', MapTalk)
		resp = func()
		print('recieved')
		category_list = resp.category_list
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		adj_array = A
		for ID in range(num_IDs):
			if static_category_dict[category_list[ID]] == Category.park:
				x = (x_list[ID])/1000.0
				y = (y_list[ID])/1000.0
				z = (z_list[ID])/1000.0
				park_dict[ID] = (x, y, z)
		info_sender()
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	rospy.init_node('one_time_sit_gen')
	print('test')
	map_maker_client()
