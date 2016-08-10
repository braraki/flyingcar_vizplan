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

import networkx as nx
from enum import Enum
import numpy as np

import thread

from map_maker import gen_adj_array_info_dict
#arguments

cf_num = int(rospy.get_param('/highlighter/cf_num'))
z_coefficient = float(rospy.get_param('/highlighter/z_coefficient'))
continuous = bool(rospy.get_param('/highlighter/continuous'))
land_vel = float(rospy.get_param('/highlighter/land_vel'))
air_vel = float(rospy.get_param('/highlighter/air_vel'))
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
##search

##this will search through the dictionary returned from a landscape in the 
##get_true_connection_dict function

current_time = 0.0

class SearchNode:
	def __init__(self, state, parent, cost=0):
		self.state = state
		self.parent = parent
		self.cost = cost

	def path(self):
		if self.parent == None:
			return [self.state]
		else:
			return self.parent.path() + [self.state]
	
class PriorityQueue:
	def __init__(self):
		self.data = []
	def push(self, item, cost):
		self.data.append((cost, item))
	def pop(self):
		self.data.sort()
		return self.data.pop(0)[1]
	def is_empty(self):
		return len(self.data) == 0
 
def a_star(successors, start_state, goal_test, heuristic=lambda x: 0):
	start_time = time.time()
	if goal_test(start_state):
		return [start_state]
	start_node = SearchNode(start_state, None, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	expanded = set()
	while not agenda.is_empty():
		parent = agenda.pop()
		if parent.state not in expanded:
			expanded.add(parent.state)
			if goal_test(parent.state):
				total_time = time.time() - start_time
				return (parent.path(), total_time)
			for child_state, cost in successors(parent.state):
				child = SearchNode(child_state, parent, parent.cost+cost)
				if child_state in expanded:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None

#single crazyflie
class system:
	def __init__(self, adj_array, info_dict, cf_ID, pubTime):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.cf_ID = cf_ID
		self.pubTime = pubTime
		self.end_pos = None
		self.cf_pos = None
		self.park_IDs = []
		for id in self.info_dict:
			info = self.info_dict[id]
			c = info[1]
			if c == Category.park:
				self.park_IDs.append(id)
		self.p = None
		self.times = []
		self.planning_time = None

	def generate_random_path(self):
		if self.end_pos == None:
			ID1 = random.choice(self.park_IDs)
		else:
			for id in self.info_dict:
				if self.info_dict[id][0] == self.end_pos:
					ID1 = id
					break
		unfound = True
		while unfound:
			ID2 = random.choice(self.park_IDs)
			if ID2 != ID1:
				unfound = False
		self.end_pos = self.info_dict[ID2][0]
		(p, self.planning_time) = self.find_path(ID1, ID2)
		return(p)

	def find_path(self, ID1, ID2):
		(x2, y2, z2) = self.info_dict[ID2][0]
		def successors(id):
			(x1, y1, z1) = self.info_dict[id][0]
			sucs = []
			row = self.adj_array[id]
			for (ID2, value) in enumerate(row):
				if value == 1:
					(fx, fy, fz) = self.info_dict[ID2][0]
					dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + z_coefficient*(z1-fz)**2)**.5
					sucs.append((ID2, dist_traveled))
			return(sucs)
		def goal_test(id):
			return  ID2== id
		def dist(id):
			(x1, y1, z1) = self.info_dict[id][0]
			dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
			return(dist)
		return(a_star(successors, ID1, goal_test, dist))
	'''
	def update_cf_pos(self, pos):
		#print('position updated: '+str(pos))
		self.cf_pos = pos
		if self.is_finished():
			self.publish_new_path()


	def is_finished(self):
		if self.cf_pos == self.end_pos:
			return(True)
		if self.end_pos != None and self.cf_pos != None:
			(x1, y1, z1) = self.cf_pos
			(x2, y2, z2) = self.end_pos
			dist = ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)**.5
			#print(dist)
			return(dist < .02)
		return(False)
	'''

	def is_finished(self):
		last_time = self.times[len(self.times) - 1]
		return(time.time() > last_time)

	def publish_new_path(self):
		self.p = self.generate_random_path()
		self.make_times()
		print(self.times)
		if self.p != None and self.p != []:
			#self.pub.publish(cf_num, self.cf_ID, self.p)
			self.pubTime.publish(cf_num, self.cf_ID, self.p, self.times, self.planning_time)
			print('published')

	def publish_old_path(self):
		if self.p != None and self.p != []:
			#self.pub.publish(cf_num, self.cf_ID, self.p)
			self.pubTime.publish(cf_num, self.cf_ID, self.p, self.times, self.planning_time)


	def make_times(self):
		global current_time
		self.times = []


		'''switch # to change between timing methods'''
		#time1 = current_time + 1
		time1 = time.time() + 2




		self.times.append(time1)
		for p_i in range(len(self.p)-1):
			ID1 = self.p[p_i]
			ID2 = self.p[p_i + 1]
			(x1, y1, z1) = self.info_dict[ID1][0]
			c1 = self.info_dict[ID1][1]
			(x2, y2, z2) = self.info_dict[ID2][0]
			c2 = self.info_dict[ID2][1]
			dist = ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)**.5
			if c1 == Category.cloud or c1 == Category.interface or c1 == Category.air_waypoint or c2 == Category.cloud or c2 == Category.interface or c2 == Category.air_waypoint:
				v = air_vel
			else:
				v = land_vel
			t = dist/v
			time1 += t
			self.times.append(time1)





		

class full_system:
	def __init__(self, adj_array, info_dict):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.system_list = []
		#self.pub = rospy.Publisher('~path_topic', HiPath, queue_size = 10)
		self.pubTime = rospy.Publisher('~time_path_topic',HiPathTime, queue_size=10)
		self.runner()

	def runner(self):
		for ID in range(cf_num):
			sys = system(self.adj_array, self.info_dict, ID, self.pubTime)
			self.system_list.append(sys)
		#rospy.init_node('highlighter', anonymous = False)
		#rospy.Subscriber('~SimPos_topic', SimPos, self.pos_update)
		for sys in self.system_list:
			sys.publish_new_path()
		#thread.start_new_thread ( self.double_check , ())
		#rospy.spin()
		self.double_check()

	#publishes constant old paths
	def double_check(self):
		while True:
			for sys in self.system_list:
				if sys.is_finished() and continuous:
					sys.publish_new_path()
				else:
					sys.publish_old_path()
			time.sleep(.1)

	#response to simulator, updates position accordingly
	#time update hard coded in, will be a problem
	'''
	def pos_update(self, data):
		global current_time
		current_time += .01
		x_list = data.x
		y_list = data.y
		z_list = data.z
		for id in range(len(x_list)):
			sys = self.system_list[id]
			x = x_list[id]
			y = y_list[id]
			z = z_list[id]
			sys.update_cf_pos((x, y, z))
	'''
'''
def map_maker_client():
	rospy.wait_for_service('send_complex_map')
	try:
		print('calling')
		func = rospy.ServiceProxy('send_complex_map', MapTalk)
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
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[category_list[ID]]
			#print(category_list[ID])
			info_dict[ID] = ((x, y, z), c)
		fs = full_system(A, info_dict)
		fs.runner()
	except rospy.ServiceException, e:
		print("service call failed")
'''

if __name__ == "__main__":
	print('test')
	rospy.init_node('highlighter', anonymous = True)
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	fs = full_system(A, info_dict)
	fs.runner()