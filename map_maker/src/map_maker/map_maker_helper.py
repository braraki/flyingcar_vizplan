#!/usr/bin/env python

import rospy
from map_maker.srv import *

import time

import networkx as nx
from enum import Enum
import numpy as np

mark_x = []
mark_y = []

class Category(Enum):
	land = 0
	park = 1
	interface = 2
	cloud = 3
	waypoint = 4
	air_waypoint = 5

static_category_dict = {0: Category.land, 1: Category.park, 2: Category.interface, 3: Category.cloud, 4: Category.waypoint, 5: Category.air_waypoint}

def get_marks():
	return((mark_x, mark_y))

def is_air(node_category):
	if node_category == Category.interface or node_category == Category.cloud or node_category == Category.air_waypoint:
		return True
	else:
		return False

#map_topic should be 'send_map' or 'send_complex_map'
def map_maker_client(map_topic='send_complex_map'):
	global mark_x
	global mark_y
	rospy.wait_for_service(map_topic)
	try:
		print('calling')
		func = rospy.ServiceProxy(map_topic, MapTalk)
		resp = func()
		print('recieved')
		category_list = resp.category_list
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		mark_x = resp.mark_x
		mark_y = resp.mark_y
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		return((info_dict, A))
	except rospy.ServiceException, e:
		print("service call failed")

'''
if __name__ == "__main__":
	map_maker_client()
'''