#!/usr/bin/env python

import rospy
from map_maker.srv import *
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from map_maker import map_maker_helper

#imported parameters
map_road_ratio = float(rospy.get_param('/mapmaker/map_road_ratio'))
map_tile_size = float(rospy.get_param('/mapmaker/map_tile_size'))

map_interface_height = float(rospy.get_param('/mapmaker/map_interface_height'))
map_cloud_height = float(rospy.get_param('/mapmaker/map_cloud_height'))
map_num_cloud_layers = int(rospy.get_param('/mapmaker/map_num_cloud_layers'))
map_cloud_layer_dist = float(rospy.get_param('/mapmaker/map_cloud_layer_dist'))
map_cloud_density = int(rospy.get_param('/mapmaker/map_cloud_density'))
helipad_height = float(rospy.get_param('/mapmaker/helipad_height'))

air_vel = float(rospy.get_param('/mapmaker/air_vel'))
time_step = float(rospy.get_param('/mapmaker/time_step'))
optimal = bool(rospy.get_param('/mapmaker/optimal'))

#imported map parameters
map_num_long = int(rospy.get_param('/mapmaker/map_num_long'))
map_num_wide = int(rospy.get_param('/mapmaker/map_num_wide'))
non_fly_list = rospy.get_param('/mapmaker/non_fly_list')
map_pre_dict = rospy.get_param('/mapmaker/map_pre_dict')

#converts strings to tuples
for k in map_pre_dict.keys():
	map_pre_dict[eval(k)] = map_pre_dict[k]
	del map_pre_dict[k]
for index in range(len(non_fly_list)):
	non_fly_list[index] = eval(non_fly_list[index])

print(non_fly_list)
print(map_pre_dict)

Category = map_maker_helper.Category

class node:
	def __init__(self, category, x, y, z = 0, angle = None):
		self.category = category
		self.x = x
		self.y = y
		self.z = z
		self.angle = angle
		self.ID = None
		self.successors = []
		self.add_successor(self)

	def add_successor(self, suc):
		if suc not in self.successors:
			self.successors.append(suc)

	def add_graph_node(self, graph):
		graph.add_node(self.ID)

	def add_graph_edges(self, graph):
		for suc in self.successors:
			graph.add_edge(self.ID, suc.ID)

class tile:
	def __init__(self, length, width, exitnodelist = [], flyable = True, x = None, y = None, elevation = 0):
		self.length = length
		self.width = width
		if 'C' in exitnodelist and len(exitnodelist)>1:
			exitnodelist.remove('C')
		self.exitnodelist = []
		for i in exitnodelist:
			if i not in self.exitnodelist:
				self.exitnodelist.append(i)
		self.flyable = flyable
		self.elevation = elevation
		self.x = x
		self.y = y
		self.node_list = []

	#tells if a node is over a tile
	def is_contained(self, new_x, new_y):
		if self.x - .5*self.length <= new_x <= self.x +.5*self.length:
			if self.y -.5*self.width <= new_y <= self.y +.5*self.width:
				return(True)
		return(False)

	#sets the tiles coordinates
	def set_co(self, x, y):
		self.x = x
		self.y = y

	#creates the land node for a tile (with the exception of the helipad park node)
	def create_land_nodes(self):
		delta_x = self.length*map_road_ratio*.25
		delta_y = self.width*map_road_ratio*.25
		#helipad
		if self.exitnodelist == ['C']:
			n = node(Category.park, self.x, self.y, self.elevation, get_rud_angle(0,0))
			self.node_list.append(n)
		#dead-end
		elif len(self.exitnodelist) == 1:
			dx = delta_x
			dy = delta_y
			x_coeff = -1
			y_coeff = -1
			if 'N' in self.exitnodelist:
				dy *= -1
				y_coeff = 1
			elif 'S' in self.exitnodelist:
				y_coeff = 1
			elif 'E' in self.exitnodelist:
				dx *= -1
				x_coeff = 1
			elif 'W' in self.exitnodelist:
				x_coeff = 1
			n1 = node(Category.land, self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
			n2 = node(Category.land, self.x+dx*x_coeff, self.y+dy*y_coeff, self.elevation, get_rud_angle(dx*x_coeff, dy*y_coeff))
			self.node_list.append(n1)
			self.node_list.append(n2)

		#non-split
		elif len(self.exitnodelist) == 2:
			dx = delta_x
			dy = delta_y
			#verticle
			if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
				dy = 0
			#horizontal
			elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
				dx = 0
			#postive slope turn
			elif ('E' in self.exitnodelist and 'S' in self.exitnodelist) or ('W' in self.exitnodelist and 'N' in self.exitnodelist):
				dx *= 1
				dy *= -1
			n1 = node(Category.land, self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
			n2 = node(Category.land, self.x-dx, self.y-dy, self.elevation, get_rud_angle(-dx, -dy))
			self.node_list.append(n1)
			self.node_list.append(n2)
		#intersection
		elif len(self.exitnodelist) > 2:
			for dx in [-1*delta_x, 1*delta_x]:
				for dy in [-1*delta_y, 1*delta_y]:
					n = node(Category.land, self.x+dx, self.y+dy, self.elevation, get_rud_angle(dx, dy))
					self.node_list.append(n)
		
	#makes connections to nodes within the same tile, connects 90 degree off things anti-clockwise, also connects so U-turns and lane crossing are possible
	def connect_own(self):
		for node1 in self.node_list:
			for node2 in self.node_list:
				if node1.angle != None and node2.angle != None:
					if node1.category == Category.land and node2.category == Category.land:
						if (node2.angle - node1.angle)%360 == 90:
							node1.add_successor(node2)						
						if len(self.exitnodelist) == 2:
							if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
								node1.add_successor(node2)
								node2.add_successor(node1)
							elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
								node1.add_successor(node2)
								node2.add_successor(node1)					

	#adds parking nodes to a tile
	#numbers are ugly, but they put the parking nodes half way between the end of the road and the end of the tile
	def add_and_connect_parking(self):
		dx = 0
		dy = 0
		#normal road
		left_x_dist = .5*self.length*(1 - map_road_ratio)
		left_y_dist = .5*self.width*(1 - map_road_ratio)
		road_x_dist = self.length*map_road_ratio
		road_y_dist = self.width*map_road_ratio
		if len(self.exitnodelist) == 2:
			straight = False
			if 'N' in self.exitnodelist and 'S' in self.exitnodelist:
				dx = .5*(road_x_dist + left_x_dist)
				straight = True
			elif 'E' in self.exitnodelist and 'W' in self.exitnodelist:
				dy = .5*(road_y_dist + left_y_dist)
				straight = True
			elif 'N' in self.exitnodelist and 'E' in self.exitnodelist:
				road_x = -1*.25*road_x_dist
				road_y = -1*.25*road_y_dist
				space_x = -1*.5*(.5*road_x_dist + left_x_dist)
				space_y = -1*.5*(.5*road_y_dist + left_y_dist)
			elif 'W' in self.exitnodelist and 'N' in self.exitnodelist:
				road_x = 1*.25*road_x_dist
				road_y = -1*.25*road_y_dist
				space_x = 1*.5*(.5*road_x_dist + left_x_dist)
				space_y = -1*.5*(.5*road_y_dist + left_y_dist)
			elif 'S' in self.exitnodelist and 'W' in self.exitnodelist:
				road_x = 1*.25*road_x_dist
				road_y = 1*.25*road_y_dist
				space_x = 1*.5*(.5*road_x_dist + left_x_dist)
				space_y = 1*.5*(.5*road_y_dist + left_y_dist)
			elif 'E' in self.exitnodelist and 'S' in self.exitnodelist:
				road_x = -1*.25*road_x_dist
				road_y = 1*.25*road_y_dist
				space_x = -1*.5*(.5*road_x_dist + left_x_dist)
				space_y = 1*.5*(.5*road_y_dist + left_y_dist)
			if straight:
				n1 = node(Category.park, self.x + dx, self.y + dy, self.elevation, get_rud_angle(dx, dy))
				n2 = node(Category.park, self.x - dx, self.y - dy, self.elevation, get_rud_angle(-1*dx, -1*dy))
			else:
				n1 = node(Category.park, self.x + road_x + space_x, self.y + road_y, self.elevation, get_rud_angle(road_x, road_y))
				n2 = node(Category.park, self.x + road_x, self.y + road_y + space_y, self.elevation, get_rud_angle(road_x, road_y))
			for n3 in self.node_list:
				if n3.angle == n1.angle:
					n3.add_successor(n1)
					n1.add_successor(n3)
				if n3.angle == n2.angle:
					n3.add_successor(n2)
					n2.add_successor(n3)
			self.node_list.append(n1)
			self.node_list.append(n2)

		#3 way intersection and dead-end
		elif len(self.exitnodelist) == 3 or len(self.exitnodelist) == 1 and 'C' not in self.exitnodelist:
			if 'N' not in self.exitnodelist and 'S' in self.exitnodelist:
				dy = .5*(road_y_dist + left_y_dist)
			elif 'S' not in self.exitnodelist and 'N' in self.exitnodelist:
				dy = -1*.5*(road_y_dist + left_y_dist)
			elif 'E' not in self.exitnodelist and 'W' in self.exitnodelist:
				dx = .5*(road_x_dist + left_x_dist)
			elif 'W' not in self.exitnodelist and 'E' in self.exitnodelist:
				dx = -1*.5*(road_x_dist + left_x_dist)
			angle = get_rud_angle(dx, dy)
			n1 = node(Category.park, self.x + dx, self.y + dy, self.elevation, angle)
			self.node_list.append(n1)
			for n in self.node_list:
				if (n.angle - angle)%360 == 45:
					n1.add_successor(n)
				elif (angle - n.angle)%360 == 45:
					n.add_successor(n1)

	#returns position of bottom left corner
	def get_mark_pos(self):
		mx = self.x - self.length*.5
		my = self.y - self.width*.5
		return((mx, my))

	#finds a node that matches certain constraints, which side of the tile it should be on
	#and which axis of alignment is most important (necessary to do corners and deadends
	#as the correct node may not be in the predicted quadrant, rather further back on the dominant axis)
	def find_node(self, x_sign, y_sign, dom_axis):
		max_x = 0
		max_y = 0
		chosen_n = None
		for n in self.node_list:
			if n.category == Category.land:
				if (n.x - self.x)*x_sign >= max_x:
					if (n.y - self.y)*y_sign >= max_y:
						max_x = (n.x - self.x)*x_sign
						max_y = (n.y - self.y)*y_sign
						chosen_n = n
		#catching corners
		if chosen_n == None:
			if dom_axis == 'x':
				for n in self.node_list:
					if n.category == Category.land:
						if (n.x - self.x)*x_sign >= max_x:
							max_x = (n.x - self.x)*x_sign
							chosen_n = n
			if dom_axis == 'y':
				for n in self.node_list:
					if n.category == Category.land:
						if (n.y - self.y)*y_sign >= max_y:
							max_y = (n.y - self.y)*y_sign
							chosen_n = n
		return(chosen_n)

	#displays the board, not the nodes and edges; single lines for the road running to the center of cells
	def display(self):
		length = self.length
		width = self.width
		base_x = self.x - .5*length
		base_y = self.y - .5*width
		xs = [base_x, base_x + length, base_x+length, base_x, base_x]
		ys = [base_y, base_y, base_y+width, base_y+width, base_y]
		plt.plot(xs, ys, 'k')
		if not self.flyable:
			plt.fill(xs, ys, 'r')
		for letter in self.exitnodelist:
			if letter == 'C':
				x = base_x + .5*length
				y = base_y +.5*width
			if letter == 'N':
				x = base_x + .5*length
				y = base_y + width
			if letter == 'S':
				x = base_x + .5*length
				y = base_y
			if letter == 'E':
				x = base_x + length
				y = base_y + .5*width
			if letter == 'W':
				x = base_x
				y = base_y + .5*width
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'g')
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'go')

#think about remove interface and cloud arguments, never will be used
class landscape:
	#initiates a landscape
	def __init__(self, num_long, num_wide, tile_dict = {}):
		self.num_long = num_long
		self.num_wide = num_wide
		self.tile_dict = tile_dict
		x_shift = (self.num_long - 1)*.5
		y_shift = (self.num_wide - 1)*.5
		for x in range(num_long):
			for y in range(num_wide):
				if (x,y) not in tile_dict:
					self.tile_dict[(x, y)] = None
				else:
					t = tile_dict[(x, y)]
					t.set_co((x - x_shift)*t.length, (y - y_shift)*t.width)
					t.create_land_nodes()
					t.connect_own()
					t.add_and_connect_parking()
					#t.add_mark_nodes()
		self.interface = None
		self.cloud = None

	#displays every tile
	def display(self):
		for co in self.tile_dict.keys():
			t = self.tile_dict[co]
			if t != None:
				t.display()
		plt.show(block = False)
			
	#checks if road runs into neighboring tiles, assumes map is valid
	def ground_tile_successors(self, tile_coordinates):
		t = self.tile_dict[tile_coordinates]
		(x, y) = tile_coordinates
		successors = []
		if 'N' in t.exitnodelist and y<self.num_wide-1:
			successors.append((x, y+1))
		if 'S' in t.exitnodelist and y>0:
			successors.append((x, y-1))
		if 'E' in t.exitnodelist and x<self.num_long-1:
			successors.append((x+1, y))
		if 'W' in t.exitnodelist and x>0:
			successors.append((x-1, y))
		return(successors)

	#does not assume valid map, verifies the ground tile successors and constructs a dict
	def get_true_connection_dict(self):
		checked_d = {}
		for co1 in self.tile_dict:
			sucs = self.ground_tile_successors(co1)
			good_sucs = []
			for s in sucs:
				if s in self.tile_dict:
					if self.ground_tile_successors(s):
						good_sucs.append(s)
			checked_d[co1] = good_sucs
		return(checked_d)

	#assumes two tiles connect validly, finds and adds the connection between their nodes
	def cross_tile_connect_node(self, tile1, tile2):
		x1_sign = 1
		y1_sign = 1
		x2_sign = 1
		y2_sign = 1
		dom_axis = 'x'
		if tile2.y > tile1.y and tile2.x == tile1.x:
			y2_sign = -1
		elif tile2.y < tile1.y and tile2.x == tile1.x:
			x1_sign = -1
			y1_sign = -1
			x2_sign = -1
		elif tile2.x > tile1.x and tile2.y == tile1.y:
			y1_sign = -1
			x2_sign = -1
			y2_sign = -1
			dom_axis = 'y'
		elif tile2.x < tile1.x and tile2.y == tile1.y:
			x1_sign = -1
			dom_axis = 'y'
		else:
			return
		n1 = tile1.find_node(x1_sign, y1_sign, dom_axis)
		n2 = tile2.find_node(x2_sign, y2_sign, dom_axis)
		n1.add_successor(n2)

	#connects the nodes throughout tiles on the landscape
	def fully_connect(self):
		tiles = self.tile_dict.values()
		checked_d = self.get_true_connection_dict()
		for co1 in checked_d:
			co_list = checked_d[co1]
			tile1 = self.tile_dict[co1]
			for co in co_list:
				t = self.tile_dict[co]
				self.cross_tile_connect_node(tile1, t)

	#assigns IDs to the nodes, done based first on lowest z, then lowest y, then lowest x
	def assign_ID(self):
		num = 0
		all_nodes = []
		just_nodes = []
		for t in self.tile_dict.values():
			for n in t.node_list:
				all_nodes.append(((n.z, n.y, n.x), n))
				just_nodes.append(n)
		if self.interface != None:
			for ns in self.interface.node_dict.values():
				for n in ns:
					all_nodes.append(((n.z, n.y, n.x), n))
					just_nodes.append(n)
		if self.cloud != None:
			for n in self.cloud.node_dict.values():
				all_nodes.append(((n.z, n.y, n.x), n))
				just_nodes.append(n)
		all_nodes = sorted(all_nodes)
		for info in all_nodes:
			n = info[1]
			n.ID = num
			num += 1
		return(just_nodes)

	#generates lists of the simple node and edge class that represent the map
	def get_nodes_and_edges(self):
		self.fully_connect()
		if self.cloud != None:
			self.cloud.generate_nodes()
			self.cloud.connect_own()
			if self.interface != None and not optimal:
				self.interface.generate_nodes()
				self.interface.connect_to_land()
				self.connect_interface_and_cloud()
			if optimal:
				self.cloud.connect_to_land()
		all_nodes = self.assign_ID()
		return(all_nodes)

	def get_mark_list(self):
		mark_x = []
		mark_y = []
		for t in self.tile_dict.values():
			(mx, my) = t.get_mark_pos()
			mark_x.append(mx)
			mark_y.append(my)
		return((mark_x, mark_y))

	#builds interface and cloud
	def generate_interface_and_cloud(self, interface_height, cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density):
		if not optimal:
			self.generate_interface(interface_height)
		self.generate_cloud(cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density)

	#builds interface
	def generate_interface(self, interface_height):
		self.interface = interface(interface_height, self)

	#builds cloud
	def generate_cloud(self, cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density):
		self.cloud = cloud(cloud_height, num_cloud_layers, cloud_layer_dist, cloud_density, self)

	#connects interface to cloud
	def connect_interface_and_cloud(self):
		min_z = self.cloud.height
		for t in self.interface.node_dict:
			i_nodes = self.interface.node_dict[t]
			for i_node in i_nodes:
				c_node_list = self.cloud.tile_node_dict[t]
				for c_node in c_node_list:
					if c_node.z == min_z:
						i_node.add_successor(c_node)
						c_node.add_successor(i_node)

# a single layer to connect the ground to the cloud
class interface:
	def __init__(self, height, landscape):
		self.height = height
		self.landscape = landscape
		self.node_dict = {}

	#generates the interfaces nodes
	def generate_nodes(self):
		for t in self.landscape.tile_dict.values():
			if t.flyable:
				if len(t.exitnodelist) > 0:
					temp_list =[]
					for n in t.node_list:
						if n.category == Category.park or n.category == Category.land:
							x = n.x
							y = n.y
							n2 = node(Category.interface, x, y, self.height)
							temp_list.append(n2)
					self.node_dict[t] = temp_list


	#connects the interface to the ground
	def connect_to_land(self):
		for t in self.landscape.tile_dict.values():
			if t in self.node_dict:
				ns = self.node_dict[t]
				for n1 in ns:
					for n2 in t.node_list:
						if n1.x == n2.x and n1.y == n2.y:
							n1.add_successor(n2)
							n2.add_successor(n1)

#hovering multilayer, multinode per tile node-cloud(density represents how many nodes across a cell)
class cloud:
	def __init__(self, height, num_layers, layer_dist, density, landscape):
		self.height = height
		self.num_layers = num_layers
		self.layer_dist = layer_dist
		self.density = density
		self.node_dict = {}
		self.tile_node_dict = {}
		self.landscape = landscape
		self.multi_connect = []

	#builds nodes
	def generate_nodes(self):
		if not optimal:
			self.non_opt_generate_nodes()
		else:
			self.opt_generate_nodes()

	#generates nodes when non optimal
	def non_opt_generate_nodes(self):
		for t in self.landscape.tile_dict.values():
			if t.flyable:
				base_y = t.y - .5*t.width + 1/float(2*self.density)*t.width
				y_dist = t.width/float(self.density)
				base_x = t.x - .5*t.length + 1/float(2*self.density)*t.length
				x_dist = t.length/float(self.density)
				for z in range(self.num_layers):
					z = z*self.layer_dist + self.height
					for dy in range(self.density):
						y = base_y + dy*y_dist
						for dx in range(self.density):
							x = base_x + dx*x_dist
							n = node(Category.cloud, x, y, z)
							if t not in self.tile_node_dict:
								self.tile_node_dict[t] = [n]
							else:
								self.tile_node_dict[t] += [n]
							self.node_dict[(x,y,z)] = n

	#makes nodes when optimal
	def opt_generate_nodes(self):
		#getting air_cloud_dimensions
		air_way_point_d = air_vel*(time_step)
		self.layer_dist = 2.0*air_way_point_d
		grid_dist = (2.0/float(1 + (2.0)**.5))*air_way_point_d

		tile_length = self.landscape.tile_dict.values()[0].length
		tile_width = self.landscape.tile_dict.values()[0].width

		min_x = -.5*(self.landscape.num_long)*tile_length
		min_y = -.5*(self.landscape.num_wide)*tile_width

		num_min_x = int(min_x / grid_dist)
		num_max_x = int(-1*min_x / grid_dist)
		num_min_y = int(min_y / grid_dist)
		num_max_y = int(-1*min_y / grid_dist)

		current_layer = 0

		while current_layer < self.num_layers:
			z = self.height + current_layer * self.layer_dist
			x_multiple = num_min_x
			while x_multiple <= num_max_x:
				x = x_multiple*grid_dist
				y_multiple = num_min_y
				while y_multiple <= num_max_y:
					y = y_multiple*grid_dist
					for t in self.landscape.tile_dict.values():
						if t.is_contained(x, y):
							if t.flyable:
								n = node(Category.cloud, x, y, z)
								if t not in self.tile_node_dict:
									self.tile_node_dict[t] = [n]
								else:
									self.tile_node_dict[t] += [n]
								self.node_dict[(x,y,z)] = n
								if (x_multiple + y_multiple)%2 == 0:
									self.multi_connect.append(n)
							break
					y_multiple += 1
				x_multiple += 1
			current_layer += 1

	#connects nodes
	def connect_own(self):
		if not optimal:
			self.non_opt_connect_own()
		else:
			self.opt_connect_own()

	#links adjacent (including diagonal) nodes in a two-way edge
	def non_opt_connect_own(self):
		for t in self.tile_node_dict:
			nodes = self.tile_node_dict[t]
			for n1 in nodes:
				for n2 in nodes:
					if n1 != n2:
						x_dist = abs(n1.x - n2.x)
						y_dist = abs(n1.y - n2.y)
						z_dist = abs(n1.z - n2.z)
						if x_dist < (t.length/float(self.density))*1.1:
							if y_dist < (t.width/float(self.density))*1.1:
								if z_dist < self.layer_dist*1.1:
									n1.add_successor(n2)
									n2.add_successor(n1)
			for t2 in self.tile_node_dict:
				if t2 != t:
					if (t2.x - t.x)<t.length*1.1:
						if (t2.y - t.y)<t.width*1.1:
							for n1 in nodes:
								for n2 in self.tile_node_dict[t2]:
									x_dist = abs(n1.x - n2.x)
									y_dist = abs(n1.y - n2.y)
									z_dist = abs(n1.z - n2.z)
									if x_dist < (t.length/float(self.density))*1.1:
										if y_dist < (t.width/float(self.density))*1.1:
											if z_dist < self.layer_dist*1.1:
												n1.add_successor(n2)
												n2.add_successor(n1)
												x_dist = abs(n1.x - n2.x)
												y_dist = abs(n1.y - n2.y)
												z_dist = abs(n1.z - n2.z)
												if x_dist < (t.length/float(self.density))*1.1:
													if y_dist < (t.width/float(self.density))*1.1:
														if z_dist < self.layer_dist*1.1:
															n1.add_successor(n2)
															n2.add_successor(n1)

	#connects nodes when optimal
	def opt_connect_own(self):
		air_way_point_d = air_vel*(time_step)
		grid_dist = (2.0/float(1 + (2.0)**.5))*air_way_point_d
		for n1 in self.node_dict.values():
			(x1, y1, z1) = (n1.x, n1.y, n1.z)
			if n1 in self.multi_connect:
				min_dist = ((2.0)**.5 * grid_dist)*1.1
			else:
				min_dist = grid_dist * 1.1		
			for n2 in self.node_dict.values():
				(x2, y2, z2) = (n2.x, n2.y, n2.z)
				#same layer
				if z2 == z1:
					distance = ((x1 - x2)**2 + (y1 - y2)**2)**.5
					if distance < min_dist:
						n1.add_successor(n2)
				#different layer
				elif abs(z2 - z1) <= self.layer_dist*1.1:
					if x1 == x2 and y1 == y2:
						n1.add_successor(n2)

	#connect cloud to land, used when optimal
	def connect_to_land(self):
		for t in self.landscape.tile_dict.values():
			if t.flyable:
				for n1 in t.node_list:
					if n1.category == Category.park or n1.category == Category.land:
						min_dist = None
						(x1, y1, z1) = (n1.x, n1.y, n1.z)
						for n2 in self.node_dict.values():
							(x2, y2, z2) = (n2.x, n2.y, n2.z)
							dist = ((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)**.5
							if min_dist == None:
								min_dist = dist
								chosen = n2
							elif min_dist > dist:
								min_dist = dist
								chosen = n2
						n1.add_successor(chosen)
						chosen.add_successor(n1)

#estimates an angle, used in connecting the nodes together
#one would have no reason to call this function, but it is necessary for the code to function
def get_rud_angle(dx, dy):
	angle_dict = {(1,0):0,(1,1):45,(0,1):90, (-1,1):135, (-1,0):180, (-1,-1):225, (0, -1):270, (1,-1):315, (0,0):None}
	if dx != 0:
		dx = int(abs(dx)/dx)
	if dy != 0:
		dy = int(abs(dy)/dy)
	return(angle_dict[(dx, dy)])

#using return nodes and edges, displays the information
def node_plot(node_list, edge_list):
	ID_dict = {}
	x_dot_list = []
	y_dot_list = []
	for n in node_list:
		ID_dict[n.ID] = (n.x, n.y, n.z)
		if n.z == 0:
			x_dot_list.append(n.x)
			y_dot_list.append(n.y)
	plt.plot(x_dot_list, y_dot_list, 'bo')
	for e in edge_list:
		ax = plt.axes()
		n1 = ID_dict[e.node1_ID]
		n2 = ID_dict[e.node2_ID]
		if n1[2] == 0 and n2[2] == 0:
			(n1x, n1y) = n1
			(n2x, n2y) = n2
			ax.arrow(n1x, n1y, n2x - n1x, n2y - n1y, head_width = .15, head_length = .35)
	plt.show()

#the next scripts constructs a landscape and then a node/edge map without calling builder
#one can fairly easily use a script to build a landscape and as our landscape will be mostly
#set, this (or something similar), is probably what we will use to construct the landscape

map_dict = {}
for co in map_pre_dict.keys():
	exitnodelist = map_pre_dict[co]
	flyable = True
	elevation = 0
	if 'C' in exitnodelist:
		elevation = helipad_height
	if co in non_fly_list:
		flyable = False
	t = tile(map_tile_size, map_tile_size, exitnodelist, flyable, co[0]*map_tile_size, co[1]*map_tile_size, elevation)
	map_dict[co] = t


final_map = landscape(map_num_long, map_num_wide, map_dict)
final_map.generate_interface_and_cloud(map_interface_height, map_cloud_height, map_num_cloud_layers, map_cloud_layer_dist, map_cloud_density)
return_nodes = final_map.get_nodes_and_edges()
(mark_x, mark_y) = final_map.get_mark_list()

#networkx
G = nx.DiGraph()
for n in return_nodes:
	n.add_graph_node(G)
for n in return_nodes:
	n.add_graph_edges(G)
A = nx.adjacency_matrix(G)
A = nx.to_numpy_matrix(G)
A2 = A.flatten()
A3 = A2.tolist()
A4 = A3[0]
A5 = []
for fl in A4:
	A5.append(int(fl))

num_nodes = len(return_nodes)
x_list = [None]*num_nodes
y_list = [None]*num_nodes
z_list = [None]*num_nodes
category_list = [None]*num_nodes
for n in return_nodes:
	(x_list[n.ID], y_list[n.ID], z_list[n.ID]) = (n.x, n.y, n.z)
	# added .value to convert Enum into int
	category_list[n.ID] = n.category.value
	if n.category == Category.park:
		print(n.ID)

def response(req):
	return MapTalkResponse(category_list, x_list, y_list, z_list, num_nodes, A5, mark_x, mark_y)

def info_sender():
	s = rospy.Service('send_map', MapTalk,response)
	print('mapmaker ready to send info back')
	rospy.spin()

if __name__ == "__main__":
	rospy.init_node('map_maker_server')
	info_sender()