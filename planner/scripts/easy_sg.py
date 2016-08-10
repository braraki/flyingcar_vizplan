#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
from map_maker.msg import *
from planner.srv import *
from planner.msg import *

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


from map_maker import gen_adj_array_info_dict

'''
class Category(Enum):
	mark = 0
	land = 1
	park = 2
	interface = 3
	cloud = 4
	waypoint = 5

static_category_dict = {0: Category.mark, 1: Category.land, 2: Category.park, 3: Category.interface, 4: Category.cloud, 5: Category.waypoint}
'''

park_dict = {}

in_use = {}

def get_remaining(cf_ID):
	global park_dict
	global in_use
	starters = []
	finishers = []
	for cf in in_use:
		v = in_use[cf]
		if cf == cf_ID:
			starters.append(v[1])
		else:
			starters.append(v[0])
			finishers.append(v[1])
	true_remains = []
	not_start = []
	not_fin = []
	for ID in park_dict:
		if ID not in starters:
			if ID not in finishers:
				true_remains.append(ID)
			else:
				not_start.append(ID)
		elif ID not in finishers:
			not_fin.append(ID)
	return((true_remains, not_start, not_fin))

def select_ID(true_remains, not_start, not_fin, start=False):
	if len(true_remains)>0:
		return(random.choice(true_remains))
	if start:
		list1 = not_start
		list2 = not_fin
	else:
		list1 = not_fin
		list2 = not_start
	if len(list1)>0:
		return(random.choice(list1))
	if len(list2)>0:
		return(random.choice(list2))
	return(random.choice(park_dict.keys()))

def generate_spots(cf_ID):
	global park_dict
	global in_use
	(true_remains, not_start, not_fin) = get_remaining(cf_ID)
	if cf_ID in in_use:
		spots = in_use[cf_ID]
		start = spots[1]
		end = select_ID(true_remains, not_start, not_fin)
	else:
		start = select_ID(true_remains, not_start, not_fin, True)
		if start in true_remains:
			true_remains.remove(start)
		if start in not_start:
			not_start.remove(start)
		if start in not_fin:
			not_fin.remove(start)
		end = select_ID(true_remains, not_start, not_fin)
	in_use[cf_ID] = (start, end)
	print('start and end')
	print((start, end))
	return((start, end))

def response(req):
	(start_ID, end_ID) = generate_spots(req.cf_ID)
	return situationResponse(start_ID, end_ID)

def info_sender():
	s = rospy.Service('send_situation', situation, response)
	#print('ready to send info back')
	rospy.spin()
'''
def map_maker_client():
	global park_dict
	rospy.wait_for_service('send_map')
	try:
		#print('calling')
		func = rospy.ServiceProxy('send_map', MapTalk)
		resp = func()
		#print('recieved')
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
		t=1
		#print("service call failed")
'''
def setup_situation(data):
	global in_use
	starting_IDs = data.starting_IDs
	for index in range(len(starting_IDs)):
		in_use[index] = (starting_IDs[index], starting_IDs[index])


if __name__ == "__main__":
	rospy.init_node('easy_sg')
	#print('test')
	info_dict = gen_adj_array_info_dict.map_maker_client('send_map')[0]
	Category = gen_adj_array_info_dict.Category
	for ID in info_dict:
		c = info_dict[ID][1]
		if c == Category.park:
			park_dict[ID] = info_dict[ID][0]
	rospy.Subscriber('~StartingID_topic', setup_IDs, setup_situation)
	info_sender()

