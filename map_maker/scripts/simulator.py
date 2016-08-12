#!/usr/bin/env python

import rospy
from map_maker.srv import *
from map_maker.msg import *

import time

import numpy as np

import thread

from map_maker import map_maker_helper


#arguments

delay = float(rospy.get_param('/simulator/delay'))
delay_2 = 1
while delay_2 > delay:
	delay_2 *= .1

fake_time = 0.0

#returns list of points from path
def analyse(p, times, info_dict):
	spots = {}
	last_ID = 0
	last_time = int(times[0]/float(delay_2))*float(delay_2)
	current_time = round(last_time, 2)
	end_time = int(times[len(times)-1]/float(delay_2))*float(delay_2)
	while current_time <= end_time:
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
		current_time += delay_2
		current_time = round(current_time, 2)
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
class flie:
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
		self.flie_list = []
		self.x_list = []
		self.y_list = []
		self.z_list = []
		self.cf_num = None
		self.running = False
		rospy.Subscriber('~time_path_topic', HiPathTime, self.act)
		rospy.Subscriber('~Start_SimPos_topic', SimPos, self.setup)

	#assigns path info
	def act(self, data):
		if not self.go:
			self.collect_info(data)
		sys = self.flie_list[data.ID]
		sys.new_path(data.path, data.times)

	#sends constant position messages (thread)
	def sub_run(self):
		self.running = True
		global fake_time
		rate = rospy.Rate(1/float(delay))
		start_time = time.time()
		reps = 0
		while not rospy.is_shutdown():
			actual_time = time.time()
			for index in range(len(self.flie_list)):
				sys = self.flie_list[index]
				if sys != None:

					'''if you switch the comments for the loc line, you should be able to go
					between the computers actual time (big numbers) and the simulated time'''

					#loc = sys.get_position(fake_time)
					loc = sys.get_position(round(actual_time, 2))

					(self.x_list[index], self.y_list[index], self.z_list[index]) = loc
			if not rospy.is_shutdown():
				test_distance(self.x_list, self.y_list, self.z_list)
				self.pub.publish(self.x_list, self.y_list, self.z_list)
				rate.sleep()
				fake_time += delay
				reps += 1

	#path info for a crazyflies first path
	def collect_info(self, data):
		if self.cf_num == None:
			self.cf_num = data.num_IDs
			self.flie_list = [None]*self.cf_num
			self.x_list = [None]*self.cf_num
			self.y_list = [None]*self.cf_num
			self.z_list = [None]*self.cf_num
		f = flie(self.info_dict, self.adjacency_array, data.path, data.times, data.ID)
		self.flie_list[data.ID] = f
		if None not in self.flie_list:
			self.go = True
			if not self.running:
				thread.start_new_thread( self.sub_run, ())

	def setup(self, data):
		self.cf_num = len(data.x)
		self.x_list = list(data.x)
		self.y_list = list(data.y)
		self.z_list = list(data.z)
		self.flie_list = [None]*self.cf_num
		if not self.running:
			thread.start_new_thread( self.sub_run, ())

if __name__ == "__main__":
	#print('test')
	rospy.init_node("sim_node")	
	(info_dict, A) = map_maker_helper.map_maker_client('send_complex_map')
	Category = map_maker_helper.Category
	fs = full_system(info_dict, A)
	rospy.spin()

