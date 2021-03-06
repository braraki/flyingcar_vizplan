#!/usr/bin/env python

import rospy
from map_maker.srv import *
from map_maker.msg import *

import math
import networkx as nx
import numpy as np

from map_maker import map_maker_helper

#parameter
ideal_way_point_d = float(rospy.get_param('/complex_map/ideal_way_point_d'))
land_vel = float(rospy.get_param('/complex_map/land_vel'))
air_vel = float(rospy.get_param('/complex_map/air_vel'))
time_step = float(rospy.get_param('/complex_map/time_step'))
optimal = bool(rospy.get_param('/complex_map/optimal'))

if optimal:
	air_way_point_d = air_vel*(time_step)
	land_way_point_d = land_vel*(time_step)
else:
	#air_way_point_d should be large
	air_way_point_d = 5
	land_way_point_d = ideal_way_point_d
	#represents the proportion of the way from land to interface the waypoint should be
	air_waypoint_frac = .1


def get_num_waypoints2(ID1, ID2, info_dict):
	(x1, y1, z1) = info_dict[ID1][0]
	c1 = info_dict[ID1][1]
	(x2, y2, z2) = info_dict[ID2][0]
	c2 = info_dict[ID2][1]
	dist = ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5
	grounded = False
	if c1 != Category.cloud and c2 != Category.cloud and c1 != Category.interface and c2 != Category.interface:
		waypoint_d = land_way_point_d
		grounded = True
	else:
		waypoint_d = air_way_point_d
	if grounded or optimal:
		raw_num = dist/float(waypoint_d)
		low = int(math.floor(raw_num))
		hi = int(math.ceil(raw_num))
		min_diff = None
		chosen = None
		for value in range(low - 1, hi + 2):
			if value >= 0:
				d = dist/float(value + 1)
				diff = abs(waypoint_d - d)
				if min_diff == None:
					min_diff = diff
					chosen = value
				elif diff < min_diff:
					min_diff = diff
					chosen = value
		return(chosen)
	elif not optimal:
		if c1 != Category.cloud and c2 != Category.cloud and (c1 == Category.interface or c2 == Category.interface):
			return(1)
		else:
			return(0)

#returns info_dict and adjacency_array with waypoints added
def get_new_info(info_dict, adjacency_array):
	ID_num = len(info_dict)

	new_info_dict = info_dict.copy()
	G = nx.DiGraph()

	e_list = []
	used_pairs = []
	for (ID1, row) in enumerate(adjacency_array):
		pass_1 = False
		G.add_node(ID1)
		info1 = info_dict[ID1]
		c1 = info1[1]
		(x1, y1, z1) = info1[0]
		if c1 != Category.land and c1 != Category.park:
			pass_1 = True
		for (ID2, value) in enumerate(row):
			if value == 1 and (ID2, ID1) not in used_pairs:
				used_pairs.append((ID1, ID2))
				both_ways = False
				row = adjacency_array[ID2]
				value2 = row[ID1]
				if value2 == 1:
					both_ways = True
				pass_2 = False
				info2 = info_dict[ID2]
				c2 = info2[1]
				(x2, y2, z2) = info2[0]
				if c2 != Category.land and c2 != Category.park:
					pass_2 = True
				if pass_1 and pass_2 and z1 == z2:
					e_list.append((ID1, ID2))
					if both_ways:
						e_list.append((ID2, ID1))
				else:
					(x1, y1, z1) = info1[0]
					(x2, y2, z2) = info2[0]
					dist = ((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)**.5
					nw = get_num_waypoints2(ID1, ID2, info_dict)
					last_ID = ID1
					if pass_1 or pass_2:
						wp_c = Category.air_waypoint
					else:
						wp_c = Category.waypoint
					for wp_num in range(nw):
						if optimal or (not pass_1 and not pass_2):
							wp_x = x1 + ((wp_num + 1)/float(nw + 1))*(x2 - x1)
							wp_y = y1 + ((wp_num + 1)/float(nw + 1))*(y2 - y1)
							wp_z = z1 + ((wp_num + 1)/float(nw + 1))*(z2 - z1)
						else:
							wp_x = x1 + air_waypoint_frac*(x2 - x1)
							wp_y = y1 + air_waypoint_frac*(y2 - y1)
							wp_z = z1 + air_waypoint_frac*(z2 - z1)
						wp_ID = ID_num
						ID_num += 1
						new_info_dict[wp_ID] = ((wp_x, wp_y, wp_z), wp_c)
						e_list.append((last_ID, wp_ID))
						if both_ways:
							e_list.append((wp_ID, last_ID))
						last_ID = wp_ID
						#print('made way point')
					e_list.append((last_ID, ID2))
					if both_ways:
						e_list.append((ID2, last_ID))
	for e in e_list:
		G.add_edge(e[0], e[1])
	A = nx.to_numpy_matrix(G)
	return(new_info_dict, A)

#sends info out
class sender:
	def __init__(self, info_dict, adjacency_matrix, mark_x, mark_y):
		self.mark_x = mark_x
		self.mark_y = mark_y
		A2 = adjacency_matrix.flatten()
		A3 = A2.tolist()
		A4 = A3[0]
		self.A5 = []
		for fl in A4:
			self.A5.append(int(fl))

		self.num_nodes = len(info_dict)
		coordinate_list = [None]*self.num_nodes
		self.x_list = [None]*self.num_nodes
		self.y_list = [None]*self.num_nodes
		self.z_list = [None]*self.num_nodes
		self.category_list = [None]*self.num_nodes

		for ID in info_dict:
			((self.x_list[ID], self.y_list[ID], self.z_list[ID]), self.category_list[ID]) = info_dict[ID]
			self.category_list[ID] = info_dict[ID][1].value

	def response(self, req):
		return MapTalkResponse(self.category_list, self.x_list, self.y_list, self.z_list, self.num_nodes, self.A5, self.mark_x, self.mark_y)

	def info_sender(self):
		s = rospy.Service('send_complex_map', MapTalk ,self.response)
		print('complex map ready to send info back')

if __name__ == "__main__":
	print('complex map started')
	rospy.init_node('complex_map_maker_server')
	(info_dict, A) = map_maker_helper.map_maker_client('send_map')
	(mark_x, mark_y) = map_maker_helper.get_marks()
	Category = map_maker_helper.Category
	analysis = get_new_info(info_dict, A)
	s = sender(analysis[0], analysis[1], mark_x, mark_y)
	s.info_sender()
	rospy.spin()