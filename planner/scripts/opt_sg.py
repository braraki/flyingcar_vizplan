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

import networkx as nx
from enum import Enum
import numpy as np


from map_maker import gen_adj_array_info_dict

park_dict = {}

in_use = {}

started = []

class single_fly:
	def __init__(self):
		self.start = None
		self.end = None
		self.num_paths = 0

	def set_start(self, start):
		self.start = start

	def set_end(self, end):
		self.end = end

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
		self.update_path(self, req.cf_ID)
		(start_ID, end_ID) = self.get_path(self.cf_ID)
		return situationResponse(start_ID, end_ID)

	def update_path(self, cf_ID):
		f = self.fly_dict[cf_ID]
		off_limits = []
		allowed_IDs = park_dict.keys()
		if f.num_paths == 0:
			starts = allowed_IDs[:]
			ends = allowed_IDs[:]
			for f2 in self.fly_dict.values():
				if f2.start in starts:
					starts.remove(f2.start)
				if f2.end in ends:
					ends(f2.end)
			start = random.choice(starts)
			end = random.choice(ends)
			f.set_first_path(start, end)
		else:
			ends = allowed_IDs[:]
			'''
			if f.end in ends:
				ends.remove(f.end)
			'''
			for f2 in self.fly_dict.values():
				if f2.num == f.num + 1:
					if f.end in ends:
						ends.remove(f.end)
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
			f.set_first_path(starting_IDs[index], starting_IDs[index])
			self.fly_dict[index] = f


if __name__ == "__main__":
	rospy.init_node('easy_sg')
	#print('test')
	info_dict = gen_adj_array_info_dict.map_maker_client('send_map')[0]
	Category = gen_adj_array_info_dict.Category
	for ID in info_dict:
		c = info_dict[ID][1]
		if c == Category.park:
			park_dict[ID] = info_dict[ID][0]
	sys = system()
	rospy.Subscriber('~StartingID_topic', setup_IDs, sys.setup_situation)
	sys.info_sender()
	rospy.spin()