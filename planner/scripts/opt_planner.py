#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from map_maker.srv import *
from map_maker.msg import *
from planner.srv import *

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
loopback_constraints = []
num_loopback_arcs = 0
timestep_constraints = []
timestep_variables = []
constraints_per_timestep = []
variables_per_timestep = []

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
				if gen_adj_array_info_dict.is_air(node1_category):
					vel = air_vel
				else:
					# if ID1 is land but it is going to an air node then
					# vel will be air_vel
					node2_category = info_dict[ID2][1]
					if gen_adj_array_info_dict.is_air(node2_category):
						vel = air_vel
				time_to_travel = dist_traveled/vel
				sucs.append((ID2, time_to_travel))
		return sucs
	return dijkstra(successors, goal)

def edge_costs(info_dict, adj_array):
	costs = {}
	num_IDs = len(info_dict)
	for ID1 in range(num_IDs):
		row = adj_array[ID1]
		for (ID2, value) in enumerate(row):
			if value == 1 or ID1 == ID2:
				costs[(ID1, ID2)] = planner_helper.optimal_cost(info_dict, ID1, ID2, air_vel, land_vel, planner_timestep)
	return costs

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

def convert_graph(old_graph, old_adj, startend, true_costs, time_horizon):
	og = old_graph
	oa = old_adj
	th = time_horizon
	num_IDs = len(old_graph)

	# new adj array
	# the format is this: each node ID is represented by
	# ID, ID+num_IDs, ID+2*num_IDs, .... ID+time_horizon*num_IDs
	# aka ID+(timestep)*num_IDs
	# note that you can retrieve the original ID of one of the
	# new states: old_ID = new_ID % num_IDs
	#na = np.zeros((num_IDs*(th+1),num_IDs*(th+1)))
	# new dict assigns a cost to each edge
	#costs = {}
	dists = {}
	arcs = tuplelist()

	# the first arcs are "loop back" arcs
	# that connect each goal node to the
	# corresponding start node
	# note that the 'end' node occurs
	# at the last time step
	global num_loopback_arcs
	for start, end in startend:
		arcs.append((end+th*num_IDs,start))
		num_loopback_arcs += 1
		print end, start

	global variables_per_timestep
	# add arcs in TIMESTEP chunks, not node chunks
	# this way when you loop through the arcs, you are looping through
	# one TIMESTEP at a time, so based off of the arc number you can
	# easily tell which timestep you are in
	for timestep in range(th):
		constraints_per_timestep.append([])
		variables_per_timestep.append(0)
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
		for ID in range(num_IDs):
			current_state = ID+timestep*num_IDs
			next_state = ID+(timestep+1)*num_IDs
			#na[(current_state, next_state)] = 1
			arcs.append((current_state, next_state))
			variables_per_timestep[timestep] += 1
			# costs are structured so that each robot/edge combo has
			# a unique cost. This should allow us to adjust cost based
			# on robot-specific data such as battery voltage
			for cf in range(cf_num):
				cost_of_waiting = 0.1
				#costs[(cf,current_state,next_state)] = cost_of_waiting
				dists[(cf,current_state,next_state)] = true_costs[(ID,ID)]
				if ID == startend[cf][1]:
					dists[(cf,current_state,next_state)] = 0

			# need to find successors of ID (sID) and connect
			# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
			old_row = oa[ID]
			for (ID2, value) in enumerate(old_row):
				if value == 1 and ID2 != ID:
					# ID2 is a successor
					next_neighbor = ID2+(timestep+1)*num_IDs
					#na[(current_state, next_neighbor)] = 1
					arcs.append((current_state, next_neighbor))
					variables_per_timestep[timestep] += 1

					(x1,y1,z1) = old_graph[ID][0]
					(x2,y2,z2) = old_graph[ID2][0]

					distance = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5

					for cf in range(cf_num):
						cost_of_travel = 0.2
						#costs[(cf,current_state, next_neighbor)] = cost_of_travel
						dists[(cf,current_state, next_neighbor)] = true_costs[(ID,ID2)]
						if ID2 == startend[cf][1]:
							dists[(cf,current_state,next_neighbor)] = 0
	
	print "NUM IDS: " + str(num_IDs)
	print "num time arcs: " + str(len(arcs))
	print variables_per_timestep
	#print arcs
	#return na, costs, arcs, dists
	return arcs, dists

def make_model(arcs, costs, startend, num_IDs, kind, optimal_paths,th,adj_array):

	m, flow = generic_model(arcs,startend,num_IDs,optimal_paths,th,adj_array)

	if kind == 'makespan':
		return makespan_model(m, flow, arcs, costs, startend, num_IDs)
	elif kind == 'minmax_distance':
		return minmax_distance_model(m, flow, arcs, costs, startend, num_IDs)
	elif kind == 'total_distance':
		return total_distance_model(m, flow, arcs, costs, startend, num_IDs)

def id_to_timestep(arc_id):
	arc_id -= num_loopback_arcs
	timestep = arc_id/variables_per_timestep[0]
	return timestep

def generic_model(arcs,startend,num_IDs, optimal_paths, th, adj_array):
	print str(time.time()) + " about to make model"
	m = Model('netflow')

	# add a flow variable for each robot-arc pair
	flow = {}
	# loop through arcs, THEN crazyflies so that you maintain clusters based on timestep
	for i,j in arcs:
		for cf in range(cf_num):
			flow[cf,i,j] = m.addVar(ub=1.0, obj= 0.0, vtype=GRB.BINARY,
				name='flow_%s_%s_%s' % (cf, i, j))

	m.update()

	print str(time.time()) + " added variables to model"

	#for cf in range(cf_num):
	#	initial_path = optimal_paths[cf]
	#	for timestep in range(len(initial_path)-1):
	#		node = initial_path[timestep]
	#		next_node = initial_path[timestep+1]
			#flow[cf,node,next_node].start = 1.0

	global constraints_per_timestep
	constraint_num = 0

	# add capacity constraint on each loopback arc
	# aka, only robot i can pass through loopback arc i
	# CRAZYFLIE NUM MUST CORRESPOND TO LOOPBACK ARC NUM
	for i in range(len(startend)):
		s, e = arcs[i]
		for cf in range(cf_num):
			if cf != i:
				#print cf, s, e
				m.addConstr(flow[cf,s,e] == 0,
					'loopback_cap_%s_%s_%s' % (cf,s,e))
				constraint_num += 1

	print str(time.time()) + " added loopback constraints"

	# add capacity constraint on each arc
	# one robot max per arc
	# not added to loopback arcs anymore. not necessary
	#for i, j in arcs:

	# add meet capacity and meet collision constraints
	for v in range(num_IDs):
		i, j = arcs[v]
		m.addConstr(quicksum(flow[cf,i,j] for cf in range(cf_num)) <= 1.0,
			name='cap_%s_%s' % (i,j))

		v_out = arcs.select(v,'*')
		flow_list = []
		for i,o in v_out:
			for cf in range(cf_num):
				flow_list.append((cf,i,o))
		m.addConstr(quicksum(flow[cf,i,o] for cf,i,o in flow_list) <= 1,
			'meet_%s' % v)
		constraint_num += 2
		timestep = id_to_timestep(v)
		constraints_per_timestep[timestep] += [constraint_num-1, constraint_num]

	print str(time.time()) + " added meet collison constraints"

	# flow conservation constraints
	for cf in range(cf_num):
		for node in range(num_IDs):
			m.addConstr(quicksum(flow[cf,i,j] for i,j in arcs.select('*',node)) ==
				quicksum(flow[cf,j,k] for j,k in arcs.select(node,'*')),
					'node_%s_%s' % (cf,node))
			constraint_num += 1
			timestep = id_to_timestep(node)
			constraints_per_timestep[timestep] += [constraint_num]


	print str(time.time()) + " added flow conservation constraints"

	# add head-on collision constraint
	# already_checked = []
	# for node in range(num_IDs):
	# 	out = arcs.select(node,'*')
	# 	for u_t0, v_t1 in out:
	# 		v_t0 = v_t1 - nontime_IDs
	# 		u_t1 = u_t0 + nontime_IDs
	# 		if valid(v_t0, num_IDs) and valid(u_t1, num_IDs) and u_t0 != v_t0:
	# 			v_out = arcs.select(v_t0,'*')
	# 			for x, y in v_out:
	# 				if y == u_t1:
	# 					edge1 = (u_t0, v_t1)
	# 					edge2 = (v_t0, u_t1)
	# 					if edge1 not in already_checked:
	# 						already_checked.append(edge1)
	# 						already_checked.append(edge2)
	# 						m.addConstr(quicksum(flow[cf,u_t0,v_t1] for cf in range(cf_num)) +
	# 							quicksum(flow[cf,v_t0,u_t1] for cf in range(cf_num)) <= 1,
	# 							'head_on_%s_%s' % (u_t0, v_t0))

	for ID1 in range(nontime_IDs):
	# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
		row = adj_array[ID1]
		for (ID2, value) in enumerate(row):
			# ID2 > ID1 because we don't want to consider when ID1 == ID2 and
			# we've already considered the case when ID1 > ID2
			if value == 1 and ID2 > ID1:
				id2_row = adj_array[ID2]
				# this is to check that ID1 and ID2 both have edges to each other
				if id2_row[ID1] == 1:
					for timestep in range(th):
						u_t0 = ID1+timestep*nontime_IDs
						u_t1 = ID1 + (timestep+1)*nontime_IDs
						v_t0 = ID2 + timestep*nontime_IDs
						v_t1 = ID2 + (timestep+1)*nontime_IDs
						#for cf in range(cf_num):
						m.addConstr(quicksum(flow[cf,u_t0,v_t1] for cf in range(cf_num)) + 
							quicksum(flow[cf,v_t0,u_t1] for cf in range(cf_num)) <= 1,
							'head_on_%s_%s' % (u_t0, v_t0))
						constraint_num += 1
						constraints_per_timestep[timestep] += [constraint_num]

	print str(time.time()) + " added head-on collision constraints"

	print constraints_per_timestep[0]
	print variables_per_timestep

	return m, flow

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

def total_distance_model(m, flow, arcs, costs, startend, num_IDs):

	# flow through the loopback arc MUST be 1 for its given robot
	for cf in range(cf_num):
		i,j = arcs[cf]
		m.addConstr(flow[cf,i,j] == 1, 'loopback_filled_%s' %(cf))

	# the objective function is the sum of every nonloopback arc/robot variable times
	# its distance cost
	objExpr = LinExpr()
	for cf in range(cf_num):
		for i,j in arcs[len(startend):]:
			objExpr.addTerms(costs[cf,i,j],flow[cf,i,j])

	m.setObjective(objExpr,GRB.MINIMIZE)

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
		self.true_costs = {}
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
			optimal_path, optimal_path_steps = optimal_distance(self.info_dict,self.adj_array,ID1,ID2)
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

	def update_model(self, m, flow, startend, time_horizon, old_startend, old_time_horizon, arcs, costs):

		# remove old loopback arc constraints and loopback arcs, and loopback flow variables
		# and replace with new constraints and arcs and variables

		for i in range(len(old_startend)):
			s, e = arcs.pop(0)
			if self.model_type == 'total_distance':
				m.remove(m.getConstrByName('loopback_filled_%s' % (i)))
			m.remove(m.getConstrByName('cap_%s_%s' % (s, e)))
			for cf in range(cf_num):
				if cf != i:
					m.remove(m.getConstrByName('loopback_cap_%s_%s_%s' % (cf, s, e)))
					m.remove(flow.pop((cf,s,e)))
					#arcs.pop(i)
		for i in range(len(startend)):
			start = startend[i][0]
			end = startend[i][1] + (time_horizon)*nontime_IDs
			arcs.insert(i, (end,start))
			for cf in range(cf_num):
				flow[cf,end,start] = m.addVar(ub=1.0, obj=0.0, vtype = GRB.BINARY,
					name='flow_%s_%s_%s' % (cf, end,start))

		m.update()

		for i in range(len(startend)):
			start = startend[i][0]
			end = startend[i][1] + (time_horizon)*nontime_IDs
			if self.model_type == 'total_distance':
				m.addConstr(flow[cf,end,start] == 1, 'loopback_filled_%s' %(i))
			m.addConstr(quicksum(flow[cf,end,start] for cf in range(cf_num)) <= 1.0,
				name='cap_%s_%s' % (end,start))
			for cf in range(cf_num):
				if cf != i:
					m.addConstr(flow[cf,end,start] == 0,
						'loopback_cap_%s_%s_%s' % (cf, end,start))

		m.update()

		m, flow, arcs, costs = self.update_graph(m,flow,arcs,costs,startend,time_horizon,old_startend,old_time_horizon)

		m.update()

		return m, flow,arcs,costs

	def update_graph(self,m,flow,arcs,costs,startend,time_horizon,old_startend,old_time_horizon):

		if time_horizon > old_time_horizon:
			print "adding nodes"
			costs = self.update_graph_update_old_startend_costs(costs,old_startend,old_time_horizon,self.true_costs)
			m, flow, arcs, costs = self.update_graph_add_nodes(m,flow,arcs,costs,startend,time_horizon,old_time_horizon,self.true_costs)
		elif time_horizon < old_time_horizon:
			print "removing nodes"
			m, flow, arcs, costs = self.update_graph_remove_nodes(m,flow,arcs,costs,startend,time_horizon,old_startend,old_time_horizon,self.true_costs)

		objExpr = LinExpr()
		for cf in range(cf_num):
			for i,j in arcs[len(startend):]:
				objExpr.addTerms(costs[cf,i,j],flow[cf,i,j])

		m.setObjective(objExpr,GRB.MINIMIZE)

		m.update()

		return m, flow, arcs, costs

	def update_graph_update_old_startend_costs(self,costs,old_startend,old_time_horizon):
		for cf in range(len(old_startend)):
			i, j = old_startend[cf]
			start = i
			end_before = j + nontime_IDs*(old_time_horizon-1)
			end = j + nontime_IDs*old_time_horizon
			# cover the 'waiting at the goal' state
			if costs[cf,end_before,end] == 0:
				costs[(cf,end_before,end)] = self.true_costs[(j,j)]

			predecessor_matrix = self.adj_array.transpose()
			predecessors = predecessor_matrix[j]
			for (ID,value) in enumerate(predecessors):
				if value == 1 and ID != j:
					end_pred = ID + (old_time_horizon-1)*nontime_IDs
					if costs[cf,end_pred,end] == 0:
						costs[cf,end_pred,end] = self.true_costs[(ID,j)]

		return costs
			
	def update_graph_remove_nodes(self,m,flow,arcs,costs,startend,time_horizon,old_startend,old_time_horizon):
		removed_arcs = tuplelist()
		removed_variables = []
		for ID in range(nontime_IDs):
			# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			for timestep in range(time_horizon,old_time_horizon):
				current_state = ID+timestep*nontime_IDs
				next_state = ID+(timestep+1)*nontime_IDs
				#na[(current_state, next_state)] = 1
				arcs.remove((current_state, next_state))
				removed_arcs.append((current_state, next_state))

				# costs are structured so that each robot/edge combo has
				# a unique cost. This should allow us to adjust cost based
				# on robot-specific data such as battery voltage
				for cf in range(cf_num):
					del costs[(cf,current_state,next_state)]
					m.remove(flow.pop((cf,current_state,next_state)))
					removed_variables.append((cf,current_state,next_state))

				# need to find successors of ID (sID) and connect
				# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
				old_row = self.adj_array[ID]
				for (ID2, value) in enumerate(old_row):
					if value == 1 and ID2 != ID:
						# ID2 is a successor
						next_neighbor = ID2+(timestep+1)*nontime_IDs
						#na[(current_state, next_neighbor)] = 1
						arcs.remove((current_state, next_neighbor))
						removed_arcs.append((current_state, next_neighbor))

						for cf in range(cf_num):
							del costs[(cf,current_state, next_neighbor)]
							m.remove(flow.pop((cf,current_state,next_neighbor)))
							removed_variables.append((cf,current_state,next_neighbor))

		m = self.remove_constraints(m, flow, removed_variables, removed_arcs,old_time_horizon,time_horizon)

		return m, flow, arcs, costs

	def remove_constraints(self,m,flow,removed_variables, removed_arcs,old_time_horizon,time_horizon):
		for i, j in removed_arcs:
			m.remove(m.getConstrByName('cap_%s_%s' % (i,j)))

		# flow conservation constraints
		for cf in range(cf_num):
			for node,j in removed_arcs:
				try:
					m.remove(m.getConstrByName('node_%s_%s' %(cf,node)))
				except Exception as inst:
					print(type(inst))

		for ID1 in range(nontime_IDs):
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			row = self.adj_array[ID1]
			for (ID2, value) in enumerate(row):
				# ID2 > ID1 because we don't want to consider when ID1 == ID2 and
				# we've already considered the case when ID1 > ID2
				if value == 1 and ID2 > ID1:
					id2_row = self.adj_array[ID2]
					# this is to check that ID1 and ID2 both have edges to each other
					if id2_row[ID1] == 1:
						for timestep in range(time_horizon,old_time_horizon):
							u_t0 = ID1+timestep*nontime_IDs
							v_t0 = ID2 + timestep*nontime_IDs
							m.remove(m.getConstrByName('head_on_%s_%s' % (u_t0,v_t0)))

		# add meet collision constraint
		for v, j in removed_arcs:
			try:
				m.remove(m.getConstrByName('meet_%s' % (v)))
			except Exception as inst:
				print(type(inst))

		m.update()

		return m

	def update_graph_add_nodes(self,m,flow,arcs,costs,startend,time_horizon,old_time_horizon):
		added_variables = []
		added_arcs = tuplelist()
		for ID in range(nontime_IDs):
			# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			for timestep in range(old_time_horizon,time_horizon):
				current_state = ID+timestep*nontime_IDs
				next_state = ID+(timestep+1)*nontime_IDs
				#na[(current_state, next_state)] = 1
				arcs.append((current_state, next_state))
				added_arcs.append((current_state, next_state))
				# costs are structured so that each robot/edge combo has
				# a unique cost. This should allow us to adjust cost based
				# on robot-specific data such as battery voltage
				for cf in range(cf_num):
					costs[(cf,current_state,next_state)] = self.true_costs[(ID,ID)]
					flow[cf,current_state,next_state] = m.addVar(ub=1.0, obj= 0.0, vtype=GRB.BINARY,
						name='flow_%s_%s_%s' % (cf, current_state, next_state))
					added_variables.append((cf,current_state,next_state))

					if ID == startend[cf][1]:
						costs[(cf,current_state,next_state)] = 0

				# need to find successors of ID (sID) and connect
				# ID+timestep*num_IDs to sID+(timestep+1)*num_IDs
				old_row = self.adj_array[ID]
				for (ID2, value) in enumerate(old_row):
					if value == 1 and ID2 != ID:
						# ID2 is a successor
						next_neighbor = ID2+(timestep+1)*nontime_IDs
						#na[(current_state, next_neighbor)] = 1
						arcs.append((current_state, next_neighbor))
						added_arcs.append((current_state, next_neighbor))

						for cf in range(cf_num):
							flow[cf,current_state,next_neighbor] = m.addVar(ub=1.0, obj= 0.0, vtype=GRB.BINARY,
								name='flow_%s_%s_%s' % (cf, current_state, next_neighbor))
							added_variables.append((cf,current_state,next_neighbor))
							costs[(cf,current_state, next_neighbor)] = self.true_costs[(ID,ID2)]
							if ID2 == startend[cf][1]:
								costs[(cf,current_state,next_neighbor)] = 0
		m.update()
		m = self.add_constraints(m, flow, added_variables, added_arcs,old_time_horizon,time_horizon)
		return m, flow, arcs, costs

	def add_constraints(self,m,flow,added_variables, added_arcs,old_time_horizon,time_horizon):
		for i, j in added_arcs:
			m.addConstr(quicksum(flow[cf,i,j] for cf in range(cf_num)) <= 1.0,
			name='cap_%s_%s' % (i,j))

		# flow conservation constraints
		for cf in range(cf_num):
			for node in range(old_time_horizon*nontime_IDs,time_horizon*nontime_IDs):
				m.addConstr(quicksum(flow[cf,i,j] for i,j in added_arcs.select('*',node)) ==
					quicksum(flow[cf,j,k] for j,k in added_arcs.select(node,'*')),
						'node_%s_%s' % (cf,node))

		for ID1 in range(nontime_IDs):
		# need to connect ID+timestep*num_IDs to ID+(timestep+1)*num_IDs
			row = self.adj_array[ID1]
			for (ID2, value) in enumerate(row):
				# ID2 > ID1 because we don't want to consider when ID1 == ID2 and
				# we've already considered the case when ID1 > ID2
				if value == 1 and ID2 > ID1:
					id2_row = self.adj_array[ID2]
					# this is to check that ID1 and ID2 both have edges to each other
					if id2_row[ID1] == 1:
						for timestep in range(old_time_horizon,time_horizon):
							u_t0 = ID1+timestep*nontime_IDs
							u_t1 = ID1 + (timestep+1)*nontime_IDs
							v_t0 = ID2 + timestep*nontime_IDs
							v_t1 = ID2 + (timestep+1)*nontime_IDs
							for cf in range(cf_num):
								m.addConstr(quicksum(flow[cf,u_t0,v_t1] for cf in range(cf_num)) + 
									quicksum(flow[cf,v_t0,u_t1] for cf in range(cf_num)) <= 1,
									'head_on_%s_%s' % (u_t0, v_t0))

		# add meet collision constraint
		for v in range(old_time_horizon*nontime_IDs,time_horizon*nontime_IDs):
			v_out = added_arcs.select(v,'*')
			flow_list = []
			for i,o in v_out:
				for cf in range(cf_num):
					flow_list.append((cf,i,o))
			m.addConstr(quicksum(flow[cf,i,o] for cf,i,o in flow_list) <= 1)

		m.update()

		return m

	def publish_new_paths(self, m=None, flow=None, arcs=None, costs=None, old_startend=None, old_time_horizon = None):
		planning_start_time = time.time()
		startend, optimal_paths, min_time_horizon = self.get_initial_path_info()
		optimal_time_paths = self.paths_to_time_paths(optimal_paths)
		print str(time.time()) + " about to convert graph"

		if m == None: #then this is your first time planning
			self.true_costs = edge_costs(self.info_dict, self.adj_array)
			arcs, costs = convert_graph(self.info_dict,A,startend,self.true_costs,min_time_horizon)
			print str(time.time()) + " converted graph; about to make model"
			m, flow = make_model(arcs, costs, startend, self.num_IDs*(min_time_horizon+1),self.model_type,optimal_time_paths,min_time_horizon,self.adj_array)
		else:
			self.true_costs = edge_costs(self.info_dict, self.adj_array)
			arcs, costs = convert_graph(self.info_dict,A,startend,self.true_costs,min_time_horizon)
			print str(time.time()) + " converted graph; about to make model"
			m, flow = make_model(arcs, costs, startend, self.num_IDs*(min_time_horizon+1),self.model_type,optimal_time_paths,min_time_horizon,self.adj_array)
			#m, flow, arcs, costs = self.update_model(m, flow, startend, min_time_horizon, old_startend, old_time_horizon, arcs, costs,self.true_costs)

		m.optimize()

		print str(time.time()) + " optimization finished"

		self.time_horizon = min_time_horizon
		while m.status != GRB.Status.OPTIMAL and self.time_horizon < 2*min_time_horizon*cf_num and not rospy.is_shutdown():
			#old_time_horizon = self.time_horizon
			#self.time_horizon = self.time_horizon + 1
			#self.update_model(m, flow, startend, self.time_horizon, old_startend, old_time_horizon, arcs, costs,self.true_costs)
			arcs, costs = convert_graph(self.info_dict,A,startend,self.true_costs,self.time_horizon)
			m, flow = make_model(arcs, costs, startend, self.num_IDs*(self.time_horizon+1),self.model_type,optimal_time_paths,self.time_horizon,self.adj_array)
			m.optimize()

		if m.status == GRB.Status.OPTIMAL:
			old_startend = startend
			old_time_horizon = self.time_horizon
			planning_end_time = time.time()
			self.planning_time = planning_start_time - planning_end_time
			solution = m.getAttr('x', flow)

			self.print_solution(m, solution, flow, arcs)
			self.paths,self.times = self.extract_paths(solution, arcs)

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
		while not rospy.is_shutdown():
			if self.is_finished():
				m, flow, arcs, costs, old_startend, old_time_horizon = self.publish_new_paths(m, flow, arcs, costs, old_startend, old_time_horizon)
			else:
				self.publish_old_paths()
			time.sleep(0.1)

	def is_finished(self):
		t = time.time()
		last_time = self.times[0][len(self.times[0]) - 1]
		return(t > last_time)

	def print_solution(self, m, solution, flow, arcs):
		print "TIME HORIZON: %d" % (self.time_horizon)
		for h in range(cf_num):
			print('\nOptimal flows for %s:' % h)
			for i,j in arcs:
				if solution[h,i,j] > 0:
					print('%s -> %s: %g' % (i, j, solution[h,i,j]))

	def extract_paths(self,solution,arcs):
		paths = {}
		times = {}
		for cf in range(cf_num):
			path = tuplelist()
			for i,j in arcs:
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
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	waiter(info_dict, A)
	#fs = full_system(info_dict, A)
	rospy.spin()
