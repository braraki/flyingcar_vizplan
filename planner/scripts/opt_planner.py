#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
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

import thread
from map_maker import map_maker_helper
from planner import planner_helper

from gurobipy import *

#arguments

used_park_IDs = []

cf_num = int(rospy.get_param('/opt_planner/cf_num'))
z_coefficient = float(rospy.get_param('/opt_planner/z_coefficient'))
continuous = bool(rospy.get_param('/opt_planner/continuous'))
land_vel = float(rospy.get_param('/opt_planner/land_vel'))
air_vel = float(rospy.get_param('/opt_planner/air_vel'))
air_buffer_dist = float(rospy.get_param('/opt_planner/air_buffer_dist'))

count = 0
nontime_IDs = 0
planner_timestep = 0.5


loopback_constraints = []
num_loopback_arcs = 0
timestep_constraints = []
timestep_variables = []
constraints_per_timestep = []
variables_per_timestep = 0

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
		self.dist = {}
	def push(self, item, cost):
		self.data.append((cost, item))
		self.dist[item] = cost
	def pop(self):
		self.data.sort()
		min_node = self.data.pop(0)[1]
		self.dist.pop(min_node,None)
		return min_node
	def is_empty(self):
		return len(self.data) == 0
	def in_queue(self,node):
		return node in self.dist
	def decrease_priority(self,node,cost):
		for i,pair in enumerate(self.data):
			old_cost,item = pair
			if item == node:
				self.data[i] = (cost,item)
				break

 
def a_star(successors, start_state, goal_test, heuristic=lambda x: 0):
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
				return parent.path()
			for child_state, cost in successors(parent.state):
				child = SearchNode(child_state, parent, parent.cost+cost)
				if child_state in expanded:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None

def dijkstra(successors, goal_state):
	distances = {}
	steps = {}
	distances[goal_state] = 0
	steps[goal_state] = 0
	agenda = PriorityQueue()
	agenda.push(goal_state, 0)
	
	while not agenda.is_empty():
		parent = agenda.pop()
		children = successors(parent)
		for child, cost in children:
			alt_cost = distances[parent] + cost
			alt_step = steps[parent] + 1
			if child not in distances:
				distances[child] = alt_cost
				agenda.push(child, alt_cost)
				steps[child] = alt_step
			elif alt_cost < distances[child]:
				distances[child] = alt_cost
				agenda.decrease_priority(child,alt_cost)
				steps[child] = alt_step
	return distances, steps

def true_distances(info_dict, adj_array, goal):
	predecessor_matrix = adj_array.transpose()
	def successors(ID1):
		(x1, y1, z1) = info_dict[ID1][0]
		sucs = []
		row = predecessor_matrix[ID1]
		for (ID2, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[ID2][0]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
				node1_category = info_dict[ID1][1]
				vel = land_vel
				# if the ID1 node is an air node, you will definitely travel
				# to node 2 at air velocity
				if map_maker_helper.is_air(node1_category):
					vel = air_vel
				else:
					# if ID1 is land but it is going to an air node then
					# vel will be air_vel
					node2_category = info_dict[ID2][1]
					if map_maker_helper.is_air(node2_category):
						vel = air_vel
				time_to_travel = dist_traveled/vel
				sucs.append((ID2, time_to_travel))
		return sucs
	return dijkstra(successors, goal)

def edge_costs(info_dict, adj_array):
	costs = {}
	battery_costs = {}
	num_IDs = len(info_dict)
	for ID1 in range(num_IDs):
		row = adj_array[ID1]
		for (ID2, value) in enumerate(row):
			if value == 1 or ID1 == ID2:
				costs[(ID1, ID2)] = planner_helper.optimal_cost(info_dict, ID1, ID2, air_vel, land_vel, planner_timestep)
				energy = planner_helper.get_opt_energy(info_dict, ID1, ID2, air_vel, land_vel, planner_timestep)
				battery_voltage_drop = planner_helper.get_voltage(energy)
				battery_costs[(ID1,ID2)] = battery_voltage_drop
	return costs, battery_costs

def optimal_distance(info_dict, adj_array, start, goal):
	(x2, y2, z2) = info_dict[goal][0]
	def successors(id):
		(x1, y1, z1) = info_dict[id][0]
		sucs = []
		row = adj_array[id]
		for (goal, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[goal][0]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
				sucs.append((goal, dist_traveled))
		return(sucs)
	def goal_test(id):
		return  goal == id
	def dist(id):
		(x1, y1, z1) = info_dict[id][0]
		dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
		return(dist)
	optimal_path = a_star(successors, start, goal_test, dist)
	return optimal_path, len(optimal_path)

def optimal_cost_path(info_dict, adj_array, start, goal, true_costs):
	(x2, y2, z2) = info_dict[goal][0]
	def successors(id):
		(x1, y1, z1) = info_dict[id][0]
		sucs = []
		row = adj_array[id]
		for (goal, value) in enumerate(row):
			if value == 1:
				(fx, fy, fz) = info_dict[goal][0]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
				cost = true_costs[(id,goal)]
				sucs.append((goal, cost))
		return(sucs)
	def goal_test(id):
		return  goal == id
	def dist(id):
		(x1, y1, z1) = info_dict[id][0]
		dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
		return(dist)
	optimal_path = a_star(successors, start, goal_test, dist)
	print "Supposedly optimal path: " + str(optimal_path)
	return optimal_path, len(optimal_path)-1

class OptModel:
	def __init__(self, info_dict, adj_array, startend, time_horizon, true_costs, battery_costs, battery_range, kind):
		self.info_dict = info_dict
		self.adj_array = adj_array
		self.startend = startend
		self.time_horizon = time_horizon
		self.true_costs = true_costs
		self.true_battery_costs = battery_costs
		self.kind = kind
		self.num_IDs = len(self.info_dict)
		self.num_time_IDs = (self.time_horizon+1)*self.num_IDs
		self.arcs, self.costs, self.battery_costs = self.convert_graph()
		self.num_arcs = len(self.arcs)
		self.battery_range = battery_range
		self.m, self.flow = self.make_model()

	def optimize(self):
		self.m.optimize()
		return self.m

	def add_time_nodes(self, ID, timestep, arcs, costs,battery_costs):
		current_state = ID+timestep*self.num_IDs
		next_state = ID+(timestep+1)*self.num_IDs
		arcs.append((current_state, next_state))
		for cf in range(cf_num):
			costs[(cf,current_state,next_state)] = self.true_costs[(ID,ID)]
			battery_costs[(cf,current_state,next_state)] = self.true_battery_costs[(ID,ID)]
			if ID == self.startend[cf][1]:
				costs[(cf,current_state,next_state)] = 0

		row = self.adj_array[ID]
		for (ID2, value) in enumerate(row):
			if value == 1 and ID2 != ID:
				next_neighbor = ID2+(timestep+1)*self.num_IDs
				arcs.append((current_state, next_neighbor))
				for cf in range(cf_num):
					costs[(cf,current_state, next_neighbor)] = self.true_costs[(ID,ID2)]
					battery_costs[(cf,current_state, next_neighbor)] = self.true_battery_costs[(ID,ID2)]
					if ID2 == self.startend[cf][1]:
						costs[(cf,current_state,next_neighbor)] = 0
		return arcs, costs, battery_costs

	def convert_graph(self):
		costs = {}
		arcs = tuplelist()
		battery_costs = {}

		# first add loopback arcs
		for start, end in self.startend:
			arcs.append((end+self.time_horizon*self.num_IDs,start))
			print start, end

		for ID in range(self.num_IDs):
			for timestep in range(self.time_horizon):
				arcs, costs, battery_costs = self.add_time_nodes(ID,timestep,arcs,costs, battery_costs)

		print "NUM IDS: " + str(self.num_IDs)
		print "num arcs: " + str(len(arcs))
		return arcs, costs, battery_costs

	def add_time_to_graph(self, arcs, costs, battery_costs, new_time_horizon):
		for ID in range(self.num_IDs):
			for timestep in range(self.time_horizon, new_time_horizon):
				arcs, costs, battery_costs = self.add_time_nodes(ID,timestep,arcs,costs,battery_costs)
		return arcs, costs, battery_costs

	def update(self, new_time_horizon, new_startend):
		new_m = self.remove_loopback_constraints(self.m)
		print "REMOVING CONSTRAINTSSSSSSSSSSSSSSSSSSSSSSSSSSSS"
		new_m = self.remove_constraints(new_m, self.startend)
		print "REMOVED CONSTRAINTS"
		new_m, new_flow, new_arcs = self.remove_loopback_variables(new_m,self.flow,self.arcs)
		new_m.update()

		if new_time_horizon > self.time_horizon:
			old_arc_list_length = len(self.arcs)
			self.arcs, self.costs = self.add_time_to_graph(self.arcs, self.costs, self.battery_costs, new_time_horizon)
			added_arcs = range(old_arc_list_length, len(self.arcs))
			new_m, new_flow = self.add_flow_variables(new_m, new_flow, added_arcs)
			time_node_list = []
			for arc in added_arcs:
				i,j = self.arcs[arc]
				time_node_list.append(i)
				time_node_list.append(j)
			new_m = self.add_capacity_constraints(new_m, new_flow, self.arcs,range(len(new_startend)))
			new_m = self.add_meet_collision_constraints(new_m, new_flow, self.arcs,time_node_list)
			new_m = self.add_flow_conservation_constraints(new_m, new_flow, self.arcs,time_node_list)
			new_m = self.add_head_on_collision_constraints(new_m, new_flow, range(self.num_IDs), range(self.time_horizon,new_time_horizon))
			new_m.update()

		new_m, new_flow, new_arcs = self.add_new_loopback_variables(new_m, new_flow, new_arcs, new_startend)
		new_m.update()
		new_m = self.add_new_loopback_constraints(new_m, new_flow, new_arcs)
		time_node_list = []
		for start, end in new_startend:
			time_node_list.append(start)
			time_node_list.append(end + (new_time_horizon)*self.num_IDs)
		node_list = [x for arc in new_startend for x in arc]
		print "TIME NODE LIST: " + str(time_node_list)
		new_m = self.add_capacity_constraints(new_m, new_flow, new_arcs,range(len(new_startend)))
		new_m = self.add_meet_collision_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m = self.add_flow_conservation_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m.update()

		self.m = new_m
		self.flow = new_flow
		self.time_horizon = new_time_horizon
		self.num_time_IDs = (self.time_horizon+1)*self.num_IDs
		self.num_arcs = len(self.arcs)
		self.startend = new_startend

	def make_model(self):
		m, flow = self.generic_model()
		if self.kind == 'total_distance':
			return self.total_distance_model(m, flow)

	def total_distance_model(self, m, flow):
		# flow through the loopback arc MUST be 1 for its given robot
		for cf in range(cf_num):
			i,j = self.arcs[cf]
			m.addConstr(flow[cf,i,j] == 1, 'loopback_filled_%s' %(cf))

		# the objective function is the sum of every nonloopback arc/robot variable times
		# its distance cost
		objExpr = LinExpr()
		for cf in range(cf_num):
			for i,j in self.arcs[len(self.startend):]:
				objExpr.addTerms(self.costs[cf,i,j],flow[cf,i,j])

		m.setObjective(objExpr,GRB.MINIMIZE)
		m.update()
		return m, flow

	def add_flow_variables(self, m, flow, arc_list):
		# add a flow variable for each robot-arc pair
		# loop through arcs, THEN crazyflies so that you maintain clusters based on timestep
		for arc in arc_list:
			i,j = self.arcs[arc]
			for cf in range(cf_num):
				flow[cf,i,j] = m.addVar(ub=1.0, obj= 0.0, vtype=GRB.BINARY,
					name='flow_%s_%s_%s' % (cf, i, j))

		m.update()
		return m, flow

	def remove_loopback_constraints(self, m):
		for i in range(len(self.startend)):
			f,l = self.arcs[i]
			m.remove(m.getConstrByName('loopback_filled_%s' % (i)))
			for cf in range(cf_num):
				if cf != i:
					m.remove(m.getConstrByName('loopback_cap_%s_%s_%s' % (cf,f,l)))
		return m

	def remove_loopback_variables(self, m, flow, arcs):
		# remove flow variables, arc edges and model variables
		# note that there is one arc edge per loopback edge
		# but there are cf_num flow var and model var per loopback edge
		for i in range(len(self.startend)):
			f,l = arcs.pop(0)
			for cf in range(cf_num):
				m.remove(flow[cf,f,l])
				del flow[(cf,f,l)]
			print "removed loopback arcs " + str(f) + " " + str(l)

		return m, flow, arcs

	def add_new_loopback_variables(self, m, flow, arcs, new_startend):
		for i in range(len(new_startend)):
			start, end = new_startend[i]
			end = end + self.time_horizon*self.num_IDs
			arcs.insert(i,(end,start))
			for cf in range(cf_num):
				flow[cf,end,start] = m.addVar(ub=1.0, obj=0.0, vtype=GRB.BINARY,
					name='flow_%s_%s_%s' % (cf,end,start))

		return m, flow, arcs

	def add_new_loopback_constraints(self, m, flow, arcs):
		for i in range(cf_num):
			f,l = arcs[i]
			for cf in range(cf_num):
				if cf != i:
					m.addConstr(flow[cf,f,l] == 0,
						'loopback_cap_%s_%s_%s' % (cf,f,l))
			m.addConstr(flow[i,f,l] == 1, 'loopback_filled_%s' %(i))
		return m

	def remove_loopback(self):
		new_m = self.remove_loopback_constraints(self.m)
		new_m = self.remove_constraints(new_m, self.startend)
		new_m, new_flow, new_arcs = self.remove_loopback_variables(new_m,self.flow,self.arcs)
		new_m.update()
		self.m = new_m
		self.flow = new_flow
		self.arcs = new_arcs

	def add_loopback(self, new_startend):
		new_m, new_flow, new_arcs = self.add_new_loopback_variables(new_m, new_flow, new_arcs, new_startend)
		new_m.update()
		new_m = self.add_new_loopback_constraints(new_m, new_flow, new_arcs)
		time_node_list = []
		for start, end in new_startend:
			time_node_list.append(start)
			time_node_list.append(end + (self.time_horizon-1)*self.num_IDs)
		node_list = [x for arc in new_startend for x in arc]
		print "TIME NODE LIST: " + str(time_node_list)
		new_m = self.add_capacity_constraints(new_m, new_flow, new_arcs,range(len(new_startend)))
		new_m = self.add_meet_collision_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m = self.add_flow_conservation_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m.update()
		self.m = new_m
		self.flow = new_flow
		self.arcs = new_arcs
		self.startend = new_startend

	def add_loopback_capacity_constraints(self, m, flow):
		# add capacity constraint on each loopback arc
		# aka, only robot i can pass through loopback arc i
		# CRAZYFLIE NUM MUST CORRESPOND TO LOOPBACK ARC NUM
		for i in range(len(self.startend)):
			s, e = self.arcs[i]
			for cf in range(cf_num):
				if cf != i:
					#print cf, s, e
					m.addConstr(flow[cf,s,e] == 0,
						'loopback_cap_%s_%s_%s' % (cf,s,e))
		return m

	def remove_constraints(self, m, startend):
		m = self.remove_capacity_constraints(m, range(len(startend)))
		time_node_list = []
		for start, end in startend:
			time_node_list.append(start)
			time_node_list.append(end + (self.time_horizon)*self.num_IDs)
		node_list = list(set([x for arc in startend for x in arc]))
		m = self.remove_meet_collision_constraints(m, time_node_list)
		m = self.remove_flow_conservation_constraints(m, time_node_list)
		#print node_list
		#m = self.remove_head_on_collision_constraints(m, node_list)
		return m

	def remove_capacity_constraints(self, m, arc_list):
		for arc in arc_list:
			i,j = self.arcs[arc]
			m.remove(m.getConstrByName('cap_%s_%s' % (i,j)))
		return m

	def remove_meet_collision_constraints(self, m, time_node_list):
		for node in time_node_list:
			v_out = self.arcs.select(node,'*')
			if v_out != []:
				m.remove(m.getConstrByName('meet_%s' % (node)))
		return m

	def update_loopback(self, new_startend):
		new_m = self.remove_loopback_constraints(self.m)
		new_m = self.remove_constraints(new_m, self.startend)
		new_m, new_flow, new_arcs = self.remove_loopback_variables(new_m,self.flow,self.arcs)
		new_m, new_flow, new_arcs = self.add_new_loopback_variables(new_m, new_flow, new_arcs, new_startend)
		new_m.update()
		new_m = self.add_new_loopback_constraints(new_m, new_flow, new_arcs)
		time_node_list = []
		for start, end in new_startend:
			time_node_list.append(start)
			time_node_list.append(end + self.time_horizon*self.num_IDs)
		node_list = [x for arc in new_startend for x in arc]
		new_m = self.add_capacity_constraints(new_m, new_flow, new_arcs,range(len(new_startend)))
		new_m = self.add_meet_collision_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m = self.add_flow_conservation_constraints(new_m, new_flow, new_arcs,time_node_list)
		new_m.update()
		self.m = new_m
		self.flow = new_flow
		self.arcs = new_arcs
		self.startend = new_startend

	def remove_flow_conservation_constraints(self, m, time_node_list):
		for cf in range(cf_num):
			for node in time_node_list:
				m.remove(m.getConstrByName('node_%s_%s' % (cf, node)))
		return m

	def remove_head_on_collision_constraints(self, m, node_list):
		for ID1 in node_list:
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			row = self.adj_array[ID1]
			for (ID2, value) in enumerate(row):
				# ID2 > ID1 because we don't want to consider when ID1 == ID2 and
				# we've already considered the case when ID1 > ID2
				if value == 1 and ID2 > ID1:
					id2_row = self.adj_array[ID2]
					# this is to check that ID1 and ID2 both have edges to each other
					if id2_row[ID1] == 1:
						for timestep in range(self.time_horizon):
							u_t0 = ID1+timestep*self.num_IDs
							v_t0 = ID2 + timestep*self.num_IDs
							#for cf in range(cf_num):
							m.remove(m.getConstrByName('head_on_%s_%s' % (u_t0, v_t0)))
		return m

	def add_capacity_constraints(self, m, flow, arcs,arc_list):
		# add capacity constraint on each arc
		# one robot max per arc
		# not added to loopback arcs anymore. not necessary. IT IS NECESSARY
		for arc in arc_list:
			i, j = arcs[arc]
			m.addConstr(quicksum(flow[cf,i,j] for cf in range(cf_num)) <= 1.0,
				name='cap_%s_%s' % (i,j))
		return m

	def add_meet_collision_constraints(self, m, flow, arcs,time_node_list):
		for v in time_node_list:
			v_out = arcs.select(v,'*')
			flow_list = []
			for i,o in v_out:
				for cf in range(cf_num):
					flow_list.append((cf,i,o))
			m.addConstr(quicksum(flow[cf,i,o] for cf,i,o in flow_list) <= 1,
				'meet_%s' % (v))
		return m

	def add_flow_conservation_constraints(self, m, flow, arcs,time_node_list):
		for cf in range(cf_num):
			for node in time_node_list:
				m.addConstr(quicksum(flow[cf,i,j] for i,j in arcs.select('*',node)) ==
					quicksum(flow[cf,j,k] for j,k in arcs.select(node,'*')),
						'node_%s_%s' % (cf,node))
		return m

	def add_head_on_collision_constraints(self, m, flow, node_list, time_range):
		for ID1 in node_list:
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			row = self.adj_array[ID1]
			for (ID2, value) in enumerate(row):
				# ID2 > ID1 because we don't want to consider when ID1 == ID2 and
				# we've already considered the case when ID1 > ID2
				if value == 1 and ID2 > ID1:
					id2_row = self.adj_array[ID2]
					# this is to check that ID1 and ID2 both have edges to each other
					if id2_row[ID1] == 1:
						for timestep in time_range:
							u_t0 = ID1+timestep*self.num_IDs
							u_t1 = ID1 + (timestep+1)*self.num_IDs
							v_t0 = ID2 + timestep*self.num_IDs
							v_t1 = ID2 + (timestep+1)*self.num_IDs
							#for cf in range(cf_num):
							m.addConstr(quicksum(flow[cf,u_t0,v_t1] for cf in range(cf_num)) + 
								quicksum(flow[cf,v_t0,u_t1] for cf in range(cf_num)) <= 1,
								'head_on_%s_%s' % (u_t0, v_t0))
		return m

	def add_battery_constraints(self, m, flow, arcs):
		for cf in range(cf_num):
			objExpr = LinExpr()
			for i,j in arcs[len(self.startend):]:
				objExpr.addTerms(self.battery_costs[cf,i,j],flow[cf,i,j])
			m.addConstr(objExpr,GRB.LESS_EQUAL, self.battery_range[cf], 'battery_%s' % (cf))
		return m

	def generic_model(self):
		print str(time.time()) + " about to make model"
		m = Model('netflow')
		flow = {}

		m, flow = self.add_flow_variables(m, flow, range(self.num_arcs))
		print str(time.time()) + " added variables to model"

		m = self.add_loopback_capacity_constraints(m, flow)
		m = self.add_capacity_constraints(m, flow, self.arcs, range(self.num_arcs))
		print str(time.time()) + " added capacity constraints"

		m = self.add_meet_collision_constraints(m, flow, self.arcs, range(self.num_time_IDs))
		print str(time.time()) + " added meet collison constraints"

		m = self.add_flow_conservation_constraints(m, flow, self.arcs, range(self.num_time_IDs))
		print str(time.time()) + " added flow conservation constraints"

		m = self.add_head_on_collision_constraints(m, flow, range(self.num_IDs), range(self.time_horizon))
		print str(time.time()) + " added head-on collision constraints"

		m = self.add_battery_constraints(m, flow, self.arcs)
		print str(time.time()) + " added battery constraints"

		m.update()
		return m, flow

	def print_solution(self):
		if self.m.status == GRB.Status.OPTIMAL:
			solution = self.m.getAttr('x', self.flow)
			print "TIME HORIZON: %d" % (self.time_horizon)
			for h in range(cf_num):
				print('\nOptimal flows for %s:' % h)
				for i,j in self.arcs:
					if solution[h,i,j] > 0:
						print('%s -> %s: %g' % (i, j, solution[h,i,j]))

	def update_batteries(self):
		paths = {}
		solution = self.m.getAttr('x',self.flow)
		for cf in range(cf_num):
			battery_drop = 0
			path = tuplelist()
			for i,j in self.arcs[len(self.startend):]:
				if solution[cf,i,j] > 0:
					battery_drop += self.battery_costs[cf,i,j]
			self.battery_range[cf] = self.battery_range[cf] - battery_drop
			print "cf %s battery is now %s" % (cf, self.battery_range[cf]) 
		return self.battery_range

	def get_paths(self):
		paths = {}
		times = {}
		solution = self.m.getAttr('x',self.flow)
		for cf in range(cf_num):
			path = tuplelist()
			for i,j in self.arcs:
				if solution[cf,i,j] > 0:
					path.append((i,j))
			first_node = path.pop(0)[1]
			#print path
			first_timestep = 0
			ordered_path = self.order_path(path,first_node)
			time_adjusted_path = self.time_adjust(ordered_path)
			paths[cf] = time_adjusted_path
			current_time = time.time()
			times[cf] = [current_time + planner_timestep*x for x in range(0,len(paths[cf]))]
		return paths,times

	def order_path(self,path,node):
		if path:
			edge = path.select(node,'*')[0]
			next_node = edge[1]
			path.remove(edge)
			return [node] + self.order_path(path,next_node)
		else:
			return [node]

	def time_adjust(self,path):
		new_path = []
		for timestep,node in enumerate(path):
			new_node = node - timestep*self.num_IDs
			new_path.append(new_node)
		return new_path


def id_to_timestep(arc_id):
	arc_id -= num_loopback_arcs
	timestep = arc_id/variables_per_timestep
	return timestep

def makespan_model(m, flow, arcs, costs, startend, num_IDs):

	objExpr = LinExpr()
	for i in range(len(startend)):
		s,e = arcs[i]
		cf_ID = i
		objExpr.addTerms(1.0,flow[cf_ID,s,e])

	m.setObjective(objExpr,GRB.MAXIMIZE)

	return m, flow

def minmax_distance_model(m, flow, arcs, costs, startend, num_IDs):

	# this is the max distance, which we will want to minimize
	x_max = m.addVar(vtype=GRB.INTEGER,name="x_max")

	m.update()

	# flow through the loopback arc MUST be 1 for its given robot
	for cf in range(cf_num):
		i,j = arcs[cf]
		m.addConstr(flow[cf,i,j] == 1, 'loopback_filled_%s' %(cf))

	# we want to constrain x_max to be larger than the largest distance
	for cf in range(cf_num):
		# note that arcs[len(startend):] represents the NON loopback arcs
		m.addConstr(quicksum(costs[cf,i,j]*flow[cf,i,j] for i,j in arcs[len(startend):]) <= x_max,
					'max_dist_for_%s' % (cf))

	m.setObjective(x_max,GRB.MINIMIZE)

	return m, flow

def valid(node, num_IDs):
	return node >= 0 and node < num_IDs

class full_system:
	def __init__(self, info_dict, adj_array):
		self.adj_array = adj_array
		self.info_dict = info_dict
		self.num_IDs = len(self.info_dict)
		self.paths = []
		self.times = []
		self.model_type = 'total_distance'
		self.planning_time = 0
		self.time_horizon = 0
		self.battery_range = []
		for cf in range(cf_num):
			self.battery_range.append(4)
		self.true_costs, self.battery_costs = edge_costs(self.info_dict, self.adj_array)
		self.pubTime = rospy.Publisher('~time_path_topic',HiPathTime, queue_size=10)
		self.runner()

	# get the start/end IDs, the optimal paths, and the min time horizon
	def get_initial_path_info(self):
		startend = tuplelist()
		optimal_paths = []
		optimal_steps = []
		min_time_horizon = 0
		for cf_ID in range(cf_num):
			#sys = system(self.adj_array, self.info_dict, cf_ID, self.pubTime)
			#self.system_list.append(sys)
			(ID1, ID2) = self.request_situation(cf_ID)
			startend.append((ID1,ID2))
			print startend[cf_ID]
			optimal_path, optimal_path_steps = optimal_cost_path(self.info_dict,self.adj_array,ID1,ID2,self.true_costs)
			optimal_paths.append(optimal_path)
			optimal_steps.append(optimal_path_steps)
			if optimal_steps[cf_ID] > min_time_horizon:
				min_time_horizon = optimal_steps[cf_ID]
			print "min horizon %d for cf %d" % (min_time_horizon, cf_ID)

		return startend, optimal_paths, min_time_horizon

	def runner(self):
		global nontime_IDs
		nontime_IDs = self.num_IDs
		m, flow, arcs, costs, old_startend, old_time_horizon = self.publish_new_paths()

		if continuous:
			self.continuous_paths(m, flow, arcs, costs, old_startend, old_time_horizon)

	def publish_new_paths(self, m=None, flow=None, arcs=None, costs=None, old_startend=None, old_time_horizon = None):
		planning_start_time = time.time()
		startend, optimal_paths, min_time_horizon = self.get_initial_path_info()
		optimal_time_paths = self.paths_to_time_paths(optimal_paths)
		print str(time.time()) + " about to convert graph"

		#min_time_horizon -= 1

		if m == None: #then this is your first time planning
			model = OptModel(self.info_dict, self.adj_array, startend, min_time_horizon, self.true_costs, self.battery_costs, self.battery_range, self.model_type)
			#model.update_loopback(startend)
		else:
			model = OptModel(self.info_dict, self.adj_array, startend, min_time_horizon, self.true_costs, self.battery_costs, self.battery_range, self.model_type)
			#model.update_loopback(startend)

		m = model.optimize()

		print str(time.time()) + " optimization finished"

		self.time_horizon = min_time_horizon
		while m.status != GRB.Status.OPTIMAL and self.time_horizon < 2*min_time_horizon*cf_num and not rospy.is_shutdown():
			#old_time_horizon = self.time_horizon
			self.time_horizon = self.time_horizon + 1
			#self.update_model(m, flow, startend, self.time_horizon, old_startend, old_time_horizon, arcs, costs,self.true_costs)
			model = OptModel(self.info_dict, self.adj_array, startend, self.time_horizon, self.true_costs, self.battery_costs, self.battery_range, self.model_type)
			#model.update(self.time_horizon, startend)
			#model.update_loopback(startend)
			m = model.optimize()

		if m.status == GRB.Status.OPTIMAL:
			old_startend = startend
			old_time_horizon = self.time_horizon
			planning_end_time = time.time()
			self.planning_time = planning_start_time - planning_end_time

			model.print_solution()
			self.battery_range = model.update_batteries()
			self.paths,self.times = model.get_paths()

			for cf in range(cf_num):
				print "PUBLISHING PATH"
				print cf, self.paths[cf]
				#print times[cf]
				self.pubTime.publish(cf_num,cf,self.paths[cf],self.times[cf],self.planning_time)

			return m, flow, arcs, costs, old_startend, old_time_horizon

	def publish_old_paths(self):
		for cf in range(cf_num):
			if self.paths[cf] != None and self.paths[cf] != []:
				self.pubTime.publish(cf_num, cf, self.paths[cf], self.times[cf], self.planning_time)

	def continuous_paths(self, m, flow, arcs, costs, old_startend, old_time_horizon):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.is_finished():
				m, flow, arcs, costs, old_startend, old_time_horizon = self.publish_new_paths(m, flow, arcs, costs, old_startend, old_time_horizon)
			else:
				self.publish_old_paths()
			r.sleep()

	def is_finished(self):
		t = time.time()
		last_time = self.times[0][len(self.times[0]) - 1]
		return(t > last_time)

	def paths_to_time_paths(self,paths):
		time_paths = []
		for path in paths:
			time_path = []
			for timestep, node in enumerate(path):
				new_node = node + timestep*self.num_IDs
				time_path.append(new_node)
			time_paths.append(time_path)
		return time_paths

	def request_situation(self,cf_ID):
			global count
			print(count)
			count += 1
			#print('situation asking')
			rospy.wait_for_service('send_situation')
			try:
				#print('calling')
				func = rospy.ServiceProxy('send_situation', situation)
				resp = func(cf_ID)
				return((resp.start_ID, resp.end_ID))
			except rospy.ServiceException, e:
				t = 1
				#print("service call failed")

def waiter(info_dict, A):
	def start(data):
		print('started!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
		fs = full_system(info_dict, A)

	#wait for setup to tell you to go
	rospy.Subscriber('~Starter', Bool, start)


if __name__ == "__main__":
	rospy.init_node('opt_planner', anonymous = True)
	(info_dict, A) = map_maker_helper.map_maker_client('send_complex_map')
	Category = map_maker_helper.Category
	waiter(info_dict, A)
	#fs = full_system(info_dict, A)
	rospy.spin()
