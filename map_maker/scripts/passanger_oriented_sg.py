#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from map_maker.srv import *
from map_maker.msg import *

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

spawn_delay = .2
spawn_chance = .02
chance = 0
predicted_spawn_turns = 0
while chance < .5:
	chance += spawn_chance*(1 - spawn_chance)**predicted_spawn_turns
	predicted_spawn_turns += 1
predicted_spawn_time = predicted_spawn_turns*spawn_delay
print('predicted spawn turns: '+str(predicted_spawn_turns))
print('predicted spawn time: '+str(predicted_spawn_time))

def spawn_passanger(park_IDs):
	#print('spawn')
	value = random.random()
	if value < spawn_chance:
		ID1 = random.choice(park_IDs)
		unfound = True
		while unfound:
			ID2 = random.choice(park_IDs)
			if ID2 != ID1:
				unfound = False
		return((ID1, ID2))
	return(None)

def spawn_loop(info_dict):

	park_IDs = []
	for id in info_dict:
		info = info_dict[id]
		c = info[1]
		if c == Category.park:
			park_IDs.append(id)

	pub = rospy.Publisher('passanger_requests', PassangerIDs, queue_size = 10)
	rospy.init_node('requester', anonymous = True)
	rate = rospy.Rate(1/float(spawn_delay))
	while not rospy.is_shutdown():
		IDs = spawn_passanger(park_IDs)
		if IDs != None:
			print(IDs)
			pub.publish(IDs[0], IDs[1])
		rate.sleep()

def map_maker_client():
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
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])/1000.0
			y = (y_list[ID])/1000.0
			z = (z_list[ID])/1000.0
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		spawn_loop(info_dict)
	except rospy.ServiceException, e:
		print("service call failed")


if __name__ == "__main__":
	print('test')
	map_maker_client()
