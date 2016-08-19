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


from map_maker import map_maker_helper
'''
park_dict = {}

in_use = {}

started = []

def get_remaining():
	global park_dict
	global in_use
	starters = []
	finishers = []
	for cf in in_use:
		v = in_use[cf]
		if v[0] != None:
			starters.append(v[0])
		if v[1] != None:
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

def select_ID(true_remains, not_start, not_fin, start=False, avoid = None):
	if avoid in true_remains:
		true_remains.remove(avoid)
	if len(true_remains)>0:
		return(random.choice(true_remains))
	if start:
		list1 = not_start
		list2 = not_fin
	else:
		list1 = not_fin
		list2 = not_start
	if avoid in list1:
		list1.remove(avoid)
	if len(list1)>0:
		return(random.choice(list1))
	if avoid in list2:
		list2.remove(avoid)
	if len(list2)>0:
		return(random.choice(list2))
	final_list = park_dict.keys()
	if avoid in final_list:
		final_list.remove(avoid)
	return(random.choice(final_list))

def generate_spots(cf_ID):
	global park_dict
	global in_use
	global started
	(true_remains, not_start, not_fin) = get_remaining()
	if cf_ID in in_use:
		spots = in_use[cf_ID]
		start = spots[1]
		end = select_ID(true_remains, not_start, not_fin, False, start)
	else:
		start = select_ID(true_remains, not_start, not_fin, True)
		end = select_ID(true_remains, not_start, not_fin, False, start)
	in_use[cf_ID] = (start, end)
	if not cf_ID in started:
		started.append(cf_ID)
	print('start and end')
	print((start, end))
	return((start, end))

def response(req):
	(start_ID, end_ID) = generate_spots(req.cf_ID)
	return situationResponse(start_ID, end_ID)

def info_sender():
	s = rospy.Service('send_situation', situation, response)
	#print('ready to send info back')
	#rospy.spin()

def setup_situation(data):
	global in_use
	starting_IDs = data.starting_IDs
	for index in range(len(starting_IDs)):
		in_use[index] = (None, starting_IDs[index])


if __name__ == "__main__":
	rospy.init_node('easy_sg')
	#print('test')
	info_dict = map_maker_helper.map_maker_client('send_map')[0]
	Category = map_maker_helper.Category
	for ID in info_dict:
		c = info_dict[ID][1]
		if c == Category.park:
			park_dict[ID] = info_dict[ID][0]
	rospy.Subscriber('~StartingID_topic', setup_IDs, setup_situation)
	info_sender()
	rospy.spin()


	'''
follow = bool(rospy.get_param('/easy_sg/follow'))
follow_node_list = rospy.get_param('/easy_sg/follow_node_list')

park_dict = {}

class single_fly:
	def __init__(self):
		self.start = None
		self.end = None
		self.num_paths = 0

	def set_path(self, end):
		self.start = self.end
		self.end = end
		self.num_paths += 1

	def set_first_path(self, start, end):
		self.start = start
		self.end = end
		self.num_paths += 1

	def get_path(self):
		return((self.start, self.end))

class system:
	def __init__(self):
		self.fly_dict = {}

	def response(self, req):
		self.update_path(req.cf_ID)
		(start_ID, end_ID) = self.get_path(req.cf_ID)
		return situationResponse(start_ID, end_ID)

	def update_path(self, cf_ID):
		f = self.fly_dict[cf_ID]
		off_limits = []
		allowed_IDs = park_dict.keys()
		if f.num_paths == 0:
			if follow:
				start = follow_node_list[0][cf_ID]
				end = follow_node_list[1][cf_ID]
				f.set_first_path(start, end)
			else:
				starts = allowed_IDs[:]
				ends = allowed_IDs[:]
				for f2 in self.fly_dict.values():
					if f2.start in starts:
						starts.remove(f2.start)
					if f2.end in ends:
						ends.remove(f2.end)
				start = random.choice(starts)
				end = random.choice(ends)
				f.set_first_path(start, end)
		else:
			if follow and len(follow_node_list) > f.num_paths+1:
				end = follow_node_list[f.num_paths + 1][cf_ID]
				f.set_path(end)
			else:
				ends = allowed_IDs[:]
				if f.end in ends and len(ends)>1:
					ends.remove(f.end)
				for f2 in self.fly_dict.values():
					if f2 != f and len(ends) > 1:
						if f2.end in ends:
							ends.remove(f2.end)
				for f2 in self.fly_dict.values():
					if f2 != f and len(ends)>1:
						if f2.start in ends:
							ends.remove(f2.start)
				end = random.choice(ends)
				f.set_path(end)

	def get_path(self, cf_ID):
		f = self.fly_dict[cf_ID]
		return f.get_path()

	def info_sender(self):
		s = rospy.Service('send_situation', situation, self.response)
		#print('ready to send info back')
		#rospy.spin()

	def setup_situation(self, data):
		starting_IDs = data.starting_IDs
		for index in range(len(starting_IDs)):
			f = single_fly()
			if not follow:
				f.set_first_path(starting_IDs[index], starting_IDs[index])
			self.fly_dict[index] = f


if __name__ == "__main__":
	rospy.init_node('easy_sg')
	#print('test')
	info_dict = map_maker_helper.map_maker_client('send_map')[0]
	Category = map_maker_helper.Category
	for ID in info_dict:
		c = info_dict[ID][1]
		if c == Category.park:
			park_dict[ID] = info_dict[ID][0]
	sys = system()
	rospy.Subscriber('~StartingID_topic', setup_IDs, sys.setup_situation)
	sys.info_sender()
	rospy.spin()