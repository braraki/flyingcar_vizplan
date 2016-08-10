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

delay = float(rospy.get_param('/simulator/delay'))

current_time = 0.0
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
#returns list of points from path
def analyse(p, times, info_dict):
	'''
	spots = []
	for index in range(len(p)-1):
		ID1 = p[index]
		ID2 = p[index + 1]
		(x1, y1, z1) = info_dict[ID1][0]
		(x2, y2, z2) = info_dict[ID2][0]
		c1 = info_dict[ID1][1]
		c2 = info_dict[ID2][1]
		if c1 == Category.interface or c1 == Category.cloud or c2 == Category.interface or c2 == Category.cloud:
			step_dist = air_step_dist
		else:
			step_dist = ground_step_dist 
		dist = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**.5
		num_divisions = int(dist/step_dist) + 1
		for d in range(num_divisions):
			x = (x1) + (d/float(num_divisions))*(x2 - x1)
			y = (y1) + (d/float(num_divisions))*(y2 - y1)
			z = (z1) + (d/float(num_divisions))*(z2 - z1)
			spots.append(x, y, z)
	spots.append(x2, y2, z2)
	return(spots)
	'''
	'''
	spots = {}
	last_time = int(times[0]/float(delay))*float(delay)
	last_ID = p[0]
	for index in range(len(p)-1):
		ID2 = p[index+1]
		t2 = int(times[index+1]/float(delay))*float(delay)
		(x1, y1, z1) = info_dict[last_ID][0]
		(x2, y2, z2) = info_dict[ID2][0]
		dist = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**.5
		delta_t = int((t2 - last_time)/float(delay))
		for dt in range(delta_t):
			t = last_time + dt*float(delay)
			t = round(t, 2)
			x = (x1) + (dt/float(delta_t))*(x2 - x1)
			y = (y1) + (dt/float(delta_t))*(y2 - y1)
			z = (z1) + (dt/float(delta_t))*(z2 - z1)		
			spots[t] = (x, y, z)
		print((t,(x,y,z)))
		last_ID = ID2
		last_time = t2
	#print(spots)
	return(spots)
	'''
	spots = {}
	last_ID = 0
	last_time = int(times[0]/float(delay))*float(delay)
	current_time = round(last_time, 2)
	end_time = int(times[len(times)-1]/float(delay))*float(delay)
	while current_time <= end_time:
		if True:#last_ID < len(times) - 1:
			if current_time > times[last_ID+1]:
				last_ID += 1
		last_time = times[last_ID]
		next_time = times[last_ID+1]
		i = info_dict[p[last_ID]]
		(x1, y1, z1) = i[0]
		i2 = info_dict[p[last_ID+1]]
		(x2, y2, z2) = i2[0]
		frac = (current_time - last_time)/float(next_time - last_time)
		x = frac*(x2 - x1) + x1
		y = frac*(y2 - y1) + y1
		z = frac*(z2 - z1) + z1
		spots[current_time] = (x, y, z)
		current_time += delay
		current_time = round(current_time, 2)
	#print(sorted(spots.keys()))
	return(spots)

def test_distance(x_list, y_list, z_list):
	fly_buffer = .4
	ground_buffer = .05
	grounded =  []
	flying = []
	for index in range(len(x_list)):
		x = x_list[index]
		y = y_list[index]
		z = z_list[index]
		if z > .01:
			for (x2, y2, z2) in flying:
				dist = ((x2 - x)**2 + (y2 -y)**2 + (z2 -z)**2)**.5
				if dist < fly_buffer:
					print('FLY TOO CLOSE: '+str(dist))
					if z>.75 or z2>.75:
						print('HEIGHT 1: '+str(z))
						print('HEIGHT 2: '+str(z2))
						print(" ")
			flying.append((x, y, z))
		else:
			for (x2, y2, z2) in grounded:
				dist = ((x2 - x)**2 + (y2 -y)**2 + (z2 -z)**2)**.5
				if dist < ground_buffer:
					print('GROUND TOO CLOSE: '+str(dist))
			grounded.append((x, y, z))

#controls single crazyflie
class system:
	def __init__(self, info_dict, adjacency_array, path, times, ID):
		self.info_dict = info_dict
		self.adjacency_array = adjacency_array
		self.path = path
		self.ID = ID
		self.times = times
		self.spots = analyse(self.path, self.times, info_dict)

	def get_position(self, time):
		time = round(time, 2)
		if time in self.spots:
			return(self.spots[time])
			self.last_spot = self.spots[time]
		#this is bug catching, I don't like it (mostly)
		else:
			if time > self.times[len(self.times)-1]:
				i = self.info_dict[self.path[len(self.path)-1]]
				(x, y, z) = i[0]
				return(x, y, z)
			elif time < self.times[0]:
				i = self.info_dict[self.path[0]]
				(x, y, z) = i[0]
				return(x, y, z)
			return((1,1,1))

	def new_path(self, path, times):
		if path != self.path:
			self.path = path
			self.times = times
			self.spots = analyse(self.path, self.times, self.info_dict)

#controls all crazyflies
class full_system:
	def __init__(self, info_dict, adjacency_array):
		self.info_dict = info_dict
		self.adjacency_array = adjacency_array
		self.pub = rospy.Publisher('~SimPos_topic', SimPos, queue_size = 10)
		self.go = False
		self.system_list = []
		self.x_list = []
		self.y_list = []
		self.z_list = []
		self.cf_num = None

	def runner(self):
		rospy.Subscriber('~time_path_topic', HiPathTime, self.act)
		rospy.Subscriber('~Start_SimPos_topic', SimPos, self.setup)
		#print('in runner')
		rospy.spin()

	#assigns path info
	def act(self, data):
		#print('act')
		if not self.go:
			self.collect_info(data)
		sys = self.system_list[data.ID]
		sys.new_path(data.path, data.times)
			#self.sub_run()

	#sends constant position messages (thread)
	def sub_run(self):
		global current_time
		rate = rospy.Rate(1/float(delay))
		start_time = time.time()
		reps = 0
		while True:#self.go:
			actual_time = time.time()
			
			#if reps%1000 == 0:
				#print('elapsed: '+str(actual_time - start_time))
				#print('sim time: '+str(current_time))
				#print('true clock time: '+str(actual_time))
			for index in range(len(self.system_list)):
				sys = self.system_list[index]
				if sys != None:

					'''if you switch the comments for the loc line, you should be able to go
					between the computers actual time (big numbers) and the simulated time'''

					#loc = sys.get_position(current_time)
					loc = sys.get_position(round(actual_time, 2))

					self.x_list[index] = loc[0]
					self.y_list[index] = loc[1]
					self.z_list[index] = loc[2]

					#(self.x_list[index], self.y_list[index], self.z_list[index]) = loc
			if not rospy.is_shutdown():
				test_distance(self.x_list, self.y_list, self.z_list)
				self.pub.publish(self.x_list, self.y_list, self.z_list)
				#print(self.x_list)
				#print(current_time)
				#print('published: ' + str((self.x_list[0], self.y_list[0], self.z_list[0])))
				rate.sleep()
				#print(current_time)
				current_time += delay
				reps += 1

	#path info for a crazyflies first path
	def collect_info(self, data):
		if self.cf_num == None:
			self.cf_num = data.num_IDs
			self.system_list = [None]*self.cf_num
			self.x_list = [None]*self.cf_num
			self.y_list = [None]*self.cf_num
			self.z_list = [None]*self.cf_num
		sys = system(self.info_dict, self.adjacency_array, data.path, data.times, data.ID)
		self.system_list[data.ID] = sys
		if None not in self.system_list:
			self.go = True
			#self.sub_run()
			#thread.start_new_thread ( self.sub_run , ())
			#print('thread initialized')

	def setup(self, data):
		self.cf_num = len(data.x)
		self.x_list = list(data.x)
		self.y_list = list(data.y)
		self.z_list = list(data.z)
		self.system_list = [None]*self.cf_num
		thread.start_new_thread( self.sub_run, ())
'''

def map_maker_client():
	rospy.wait_for_service('send_complex_map')
	try:
		#print('calling')
		func = rospy.ServiceProxy('send_complex_map', MapTalk)
		resp = func()
		#print('recieved')
		x_list = resp.x_list
		y_list = resp.y_list
		z_list = resp.z_list
		num_IDs = resp.num_IDs
		adjacency_array = resp.adjacency_array
		A = np.array(adjacency_array)
		A.shape = (num_IDs, num_IDs)
		info_dict = {}
		for ID in range(num_IDs):
			x = (x_list[ID])
			y = (y_list[ID])
			z = (z_list[ID])
			c = static_category_dict[resp.category_list[ID]]
			info_dict[ID] = ((x, y, z),c)
		#print(info_dict)
		fs = full_system(info_dict, A)
		fs.runner()
	except rospy.ServiceException, e:
		print("service call failed: "+str(e))
'''

if __name__ == "__main__":
	#print('test')
	rospy.init_node("sim_node")	
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	fs = full_system(info_dict, A)
	fs.runner()

