#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from map_maker.srv import *
from map_maker.msg import *
from planner.srv import *
from planner.msg import *
from std_msgs.msg import String

import time

from map_maker import map_maker_helper
from planner import planner_helper

cf_num = int(rospy.get_param('/si_planner/cf_num'))
z_coefficient = float(rospy.get_param('/si_planner/z_coefficient'))
continuous = bool(rospy.get_param('/si_planner/continuous'))
land_vel = float(rospy.get_param('/si_planner/land_vel'))
air_vel = float(rospy.get_param('/si_planner/air_vel'))
takeoff_vel = float(rospy.get_param('/si_planner/takeoff_vel'))
landing_vel = float(rospy.get_param('/si_planner/landing_vel'))
air_buffer_dist = float(rospy.get_param('/si_planner/air_buffer_dist'))
buffer_z_frac = float(rospy.get_param('/si_planner/buffer_z_frac'))
show_reserved = bool(rospy.get_param('/si_planner/show_reserved'))

si_dict = {}

air_buffer_dict = {}

count = 0

##search

##this will search through the dictionary returned from a landscape in the 
##get_true_connection_dict function

#current_time = 0.0
space_time = .75
planning_time = 2

best_heur = True


class SearchNode:
	def __init__(self, state, parent, time, voltage, cost=0, interval=None):
		self.state = state
		self.parent = parent
		self.time = time
		self.voltage = voltage
		self.cost = cost
		self.interval = interval

	def path(self):
		if self.parent == None:
			return [(self.state, self.time)]
		else:
			return self.parent.path() + [(self.state, self.time)]


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
	def decrease_priority(self, item, cost):
		for i,pair in enumerate(self.data):
			old_cost,old_item = pair
			if item == old_item:
				self.data[i] = (cost,item)
				break
 
def a_star(info_dict, successors, start_state, start_voltage, goal_test, heuristic=lambda x: 0):
	start_time = time.time()+planning_time
	if goal_test(start_state, start_time):
		return ([start_state], start_voltage)
	start_node = SearchNode(start_state, None, start_time , start_voltage, 0, find_start_interval(start_state, start_time))
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	cost_expanded = {}
	time_expanded = {}
	kept_expanded = {}
	reps = 0
	while not agenda.is_empty():
		reps += 1
		#print(reps)
		parent = agenda.pop()
		cont = False
		
		if (parent.state, parent.interval) not in cost_expanded:
			cost_expanded[(parent.state, parent.interval)] = parent.cost
			cont = True
		elif cost_expanded[(parent.state, parent.interval)] > parent.cost:
			cost_expanded[(parent.state, parent.interval)] = parent.cost
			cont = True
		if (parent.state, parent.interval) not in time_expanded:
			time_expanded[(parent.state, parent.interval)] = parent.time
			cont = True
		elif time_expanded[(parent.state, parent.interval)] > parent.time:
			time_expanded[(parent.state, parent.interval)] = parent.time
			cont = True
		'''
		if (parent.state, parent.interval) not in kept_expanded:
			kept_expanded[(parent.state, parent.interval)] = [(parent.time, parent.cost)]
			cont = True
		else:
			kepts = kept_expanded[(parent.state, parent.interval)]
			keep = True
			for (t, c) in kepts:
				if t < parent.time:
					elapsed_time = parent.time - t
					used_energy = planner_helper.get_wait_energy(info_dict, parent.state, elapsed_time)
					projected_cost = c + planner_helper.get_cost(used_energy, elapsed_time)
					if projected_cost < c:
						keep = False
						break
			if keep:
				kept_expanded[(parent.state, parent.interval)] += [(parent.time, parent.cost)]
				cont = True
		'''
		if cont:
			if goal_test(parent.state, parent.time):
				print(parent.path())
				return (parent.path(), parent.voltage)
			for child_state, t, v, cost, interval in successors(parent.state, parent.time, parent.voltage, parent.interval):
				ID = child_state

				keep = False
				if (child_state, interval) not in cost_expanded:
					keep = True
				elif cost_expanded[(child_state, interval)] > cost:
					keep = True
				if (child_state, interval) not in time_expanded:
					keep = True
				elif time_expanded[(child_state, interval)] > t:
					keep = True
				'''
				keep = True
				if (child_state, interval) in kept_expanded:
					kepts = kept_expanded[(child_state, interval)]
					for (kt, kc) in kepts:
						if kt < t:
							elapsed_time = t - kt
							used_energy = planner_helper.get_wait_energy(info_dict, child_state, elapsed_time)
							projected_cost = kc + planner_helper.get_cost(used_energy, elapsed_time)
							if projected_cost < cost:
								keep = False
								break
				'''
				if keep:
					child = SearchNode(child_state, parent, t, v, parent.cost+cost, interval)
					agenda.push(child, child.cost+heuristic(child_state))



	print('reps')
	print(reps)
	print('start')
	print((start_state, start_time))
	print('first successors')
	#s1 = (successors(start_state, start_time))
	#print(s1)
	#for (spot, t, dt) in s1:
	#	s2 = (successors(spot, t))
	#	print(s2)
	#	for (spot2, t2, dt2) in s2:
	#		print(successors(spot2, t2))
	return None

def find_start_interval(state, time):
	my_intervals = si_dict[state]
	my_interval = None
	for i in my_intervals:
		if i[0] - .1 <= time <= i[1]:
			#print(i)
			return(i)
	print('interval not found')
	print('AAAAAAAAAAAAAAAAAAA')

def dijkstra(successors, goal_state):
	costs = {}
	costs[goal_state] = 0
	agenda = PriorityQueue()
	agenda.push(goal_state, 0)
	
	while not agenda.is_empty():
		parent = agenda.pop()
		children = successors(parent)
		for child, cost in children:
			alt_cost = costs[parent] + cost
			if child not in costs:
				costs[child] = alt_cost
				agenda.push(child, alt_cost)
			elif alt_cost < costs[child]:
				costs[child] = alt_cost
				agenda.decrease_priority(child,alt_cost)
	return costs

def true_costs(info_dict, adj_array, goal):
	predecessor_matrix = adj_array.transpose()
	def successors(ID2):
		#((x1, y1, z1),c1) = info_dict[ID1]
		sucs = []
		row = predecessor_matrix[ID2]
		for (ID1, value) in enumerate(row):
			if value == 1:
				'''
				((fx, fy, fz), fc) = info_dict[ID2]
				dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
				vel = land_vel
				if c1 != Category.land or c1 != Category.park or c1 != Category.waypoint or c2 != Category.land or c2 != Category.park or c2 != Category.waypoint:
					vel = air_vel
				time_passed = dist_traveled / float(vel)
				'''
				#time_passed = planner_helper.get_cost(info_dict, ID1, goal, air_vel, land_vel)
				t = planner_helper.get_time(info_dict, ID1, ID2, air_vel, land_vel, takeoff_vel, landing_vel)
				e = planner_helper.get_energy(info_dict, ID1, ID2, t, air_vel, land_vel, takeoff_vel, landing_vel)
				c = planner_helper.get_cost(e, t)
				sucs.append((ID1, c))
		return sucs
	return dijkstra(successors, goal)



def get_reserved_IDs():
	t = time.time()
	reserved = []
	for ID in si_dict:
		for interval in si_dict[ID]:
			found = False
			if interval[0] < t < interval[1]:
				found = True
				break
		if not found:
			reserved.append(ID)
	return(reserved)




#single crazyflie
class flie:
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
		self.record_planning_time = None
		self.voltage = 1000000000
		self.running = True
		self.info_pub = rospy.Publisher('/info_topic',String, queue_size=20)

	def generate_random_path(self):
		(ID1, ID2) = self.request_situation()
		self.end_pos = self.info_dict[ID2][0]
		p_info = self.find_path(ID1, ID2)
		if p_info != None:
			(p, self.planning_time) = p_info
			p2 = self.edit_path(p)
			return(p2)
		
	def request_situation(self):
		global count
		print(count)
		count += 1
		#print('situation asking')
		rospy.wait_for_service('send_situation')
		try:
			#print('calling')
			func = rospy.ServiceProxy('send_situation', situation)
			resp = func(self.cf_ID)
			return((resp.start_ID, resp.end_ID))
		except rospy.ServiceException, e:
			t = 1
			#print("service call failed")

	def edit_path(self, p):
		new_path = []
		for index in range(len(p) - 1):
			(ID1, t1) = p[index]
			(ID2, t2) = p[index+1]
			(x1, y1, z1) = self.info_dict[ID1][0]
			c1 = self.info_dict[ID1][1]
			(x2, y2, z2) = self.info_dict[ID2][0]
			c2 = self.info_dict[ID2][1]
			dist = ((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)**.5
			v = land_vel
			if map_maker_helper.is_air(c1) or map_maker_helper.is_air(c2):
				if z1 - .01 > z2:
					v = landing_vel
				elif z2 - .01 > z1:
					v = takeoff_vel
				else:
					v = air_vel
			expected_time = dist/float(v)
			true_time = t2 - t1
			new_path.append((ID1, t1))
			if true_time - expected_time > .0002:
				new_path.append((ID1, t2 - expected_time))
				print('waited')
		new_path.append(p[len(p) - 1])
		return(new_path)

	def find_path(self, ID1, end_ID):
		#print('find path')
		start_planning_time = time.time()
		(x2, y2, z2) = self.info_dict[end_ID][0]
		if best_heur:
			cost_dict = true_costs(self.info_dict, self.adj_array, end_ID)

		def successors(state, time, voltage, my_interval):
			#print(" ")
			#print('successors')
			#print((state, time))
			#time = state[1]
			(x1, y1, z1) = self.info_dict[state][0]
			c1 = self.info_dict[state][1]
			sucs = []
			row = self.adj_array[state]
			for (ID2, value) in enumerate(row):
				if value == 1:
					if ID2 != state:
						(fx, fy, fz) = self.info_dict[ID2][0]
						c2 = self.info_dict[ID2][1]
						'''
						dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
						v = land_vel
						if map_maker_helper.is_air(c1) or map_maker_helper.is_air(c2):
							if z1 - .01 > z2:
								v = landing_vel
							elif z2 - .01 > z1:
								v = takeoff_vel
							else:
								v = air_vel
						time_passed = dist_traveled/float(v)
						'''
						time_passed = planner_helper.get_time(self.info_dict, state, ID2, air_vel, land_vel, takeoff_vel, landing_vel)
						current_time = time + time_passed
						safe_intervals = si_dict[ID2]
						for interval in safe_intervals:
							if interval[0] + space_time < current_time < interval[1] - space_time:
								suc_state = ID2

								energy = planner_helper.get_energy(self.info_dict, state, suc_state, time_passed, air_vel, land_vel, takeoff_vel, landing_vel)
								cost = planner_helper.get_cost(energy, time_passed)
								voltage_drop = planner_helper.get_voltage(energy)
								final_voltage = voltage - voltage_drop

								if final_voltage > 0:
									if c2 != Category.park:
										sucs.append((suc_state, current_time, final_voltage, cost, interval))
									elif goal_test(suc_state, current_time):
										sucs.append((suc_state, current_time, final_voltage, cost, interval))
								#elif interval[1] < my_interval[1] and interval[0] < interval[1] - 2*space_time:
							#if state overlaps
							elif interval[0] < my_interval[1]:
								suc_state = ID2
								arrival_time = max(current_time, interval[0] + space_time)
								#check that leaving is allowed
								if my_interval[0] < arrival_time < my_interval[1]:
									#if we can arrive in time
									if interval[0] <= arrival_time < interval[1] - space_time:

										time_passed = arrival_time - time
										energy = planner_helper.get_energy(self.info_dict, state, suc_state, time_passed, air_vel, land_vel, takeoff_vel, landing_vel)
										cost = planner_helper.get_cost(energy, time_passed)
										voltage_drop = planner_helper.get_voltage(energy)
										final_voltage = voltage - voltage_drop

										if final_voltage > 0:	
											if c2 != Category.park:
												sucs.append((suc_state, arrival_time, final_voltage, cost, interval))
											elif goal_test(suc_state, arrival_time):
												sucs.append((suc_state, arrival_time, final_voltage, cost, interval))
			#print(sucs)
			return(sucs)
			
		def goal_test(state, t):
			#print(id)
			if end_ID == state:
				safe_intervals = si_dict[state]
				for interval in safe_intervals:
					if interval[0] + space_time< t < interval[1] - (space_time + planning_time):
						return  True
			return(False)

		def heur(state):
			(x1, y1, z1) = self.info_dict[state][0]
			dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
			time_heur = dist/float(air_vel)
			return(time_heur)

		def true_cost_heur(state):
			return(cost_dict[state])

		if best_heur:
			p_info = a_star(self.info_dict, successors, ID1, self.voltage, goal_test, true_cost_heur)
		else:
			p_info= a_star(self.info_dict, successors, ID1, self.voltage, goal_test, heur)
		if p_info!= None:
			(path, self.voltage) = p_info
			total_planning_time = time.time() - start_planning_time
			(path_cost, path_distance, path_time) = self.analyze_path(path)
			self.info_pub.publish(str(total_planning_time) + '\t' + str(path_cost) + '\t' + str(path_distance) + '\t' + str(path_time))
			print('Time to plan: '+str(total_planning_time))
			print('Current voltage: '+str(self.voltage))
			return(path, total_planning_time)
		else:
			self.kill(ID1)

	def analyze_path(self,path):
		print "DSLJFSDL:KDJF:SLDKFJLSD:KJF:SDLKJF:SDFJSD:"
		print path
		start_node = path[0][0]
		end_node = path[-1][0]
		end_ID = end_node
		cost_dict = true_costs(self.info_dict, self.adj_array, end_ID)
		path_time = path[-1][1] - path[0][1]
		prev_x = 0
		prev_y = 0
		prev_z = 0
		distance = 0
		path_cost = 0
		for index, node in enumerate(path):
			ID = node[0]
			path_cost += cost_dict[ID] 
			infoz = self.info_dict[ID]
			(x,y,z) = infoz[0]
			if index > 0:
				distance += ((z-prev_z)**2 + (y-prev_y)**2 + (x-prev_x)**2)**0.5
			prev_x = x
			prev_y = y
			prev_z = z
		return path_cost, distance, path_time

	#kills path, cf will stop moving, sends almost empty path
	def kill(self, end_ID):
		self.running = False
		self.pubTime.publish(cf_num, self.cf_ID, [end_ID, end_ID], [time.time(), time.time()+1], 0)


	def is_finished(self):
		t = time.time()
		last_time = self.times[len(self.times) - 1]
		return(t > last_time)


	def publish_new_path(self):
		#print('publish new path')
		info = self.generate_random_path()
		if info == None:
			print('not good')
			#self.publish_new_path()
		else:
			self.p = []
			self.times = []
			for state in info:
				self.p.append(state[0])
				self.times.append(state[1])
			self.update_si_dict()
			#print(self.p)
			#print(self.times)
			if self.p != None and self.p != []:
				self.pubTime.publish(cf_num, self.cf_ID, self.p, self.times, self.planning_time)
				#print('published')

	def publish_old_path(self):
		if self.p != None and self.p != []:
			self.pubTime.publish(cf_num, self.cf_ID, self.p, self.times, self.planning_time)

	def update_si_dict(self):
		#print('update si dict')
		global si_dict
		current_time = time.time()
		purge_time = current_time - 10
		#print('path: '+str(self.cf_ID))
		#print(" ")
		for index in range(len(self.p)):
			last = False
			first = False
			if index == len(self.p) - 1:
				last = True
			else:
				next_t = self.times[index + 1]
				next_ID = self.p[index + 1]
			if index == 0:
				first = True
			else:
				last_t = self.times[index - 1]
				last_ID = self.p[index - 1]
			ID = self.p[index]
			t = self.times[index]
			buffer_IDs = air_buffer_dict[ID]
			#print(t)
			for ID in buffer_IDs:
				intervals = si_dict[ID]
				new_intervals = []
				#print(intervals)
				for interval in intervals:
					if interval[1] > purge_time:
						if interval[0] < purge_time:
							start_time = purge_time
						else:
							start_time = interval[0]
						if start_time < t < interval[1]:
							low_split = t - space_time
							high_split = t + space_time
							if last:
								#print('last')
								high_split = t + (planning_time - .02)
							else:
								high_split = max(next_t, t + space_time)
							if first:
								low_split = low_split
							else:
								low_split = min(t - space_time, last_t)
							if low_split > start_time:
								new_intervals.append((start_time, low_split))
							if high_split < interval[1]:
								new_intervals.append((high_split, interval[1]))
						else:
							new_intervals.append((start_time, interval[1]))
				si_dict[ID] = new_intervals
				#print(t)
				#print(new_intervals)
				#print(" ")
			#print(si_dict)

class full_system:
	def __init__(self, adj_array, info_dict):
		self.adj_array = adj_array
		self.info_dict = info_dict
		print "NUMBER OF NODES: " + str(len(self.info_dict))
		self.flie_list = []
		self.pubTime = rospy.Publisher('~time_path_topic',HiPathTime, queue_size=10)
		self.pubRes = rospy.Publisher('~reserved_IDs_topic', reserved_IDs, queue_size=10)
		self.runner()

	def analyse_adj_array(self):
		for (ID1, row) in enumerate(self.adj_array):
			suc = []
			if self.info_dict[ID1][1]== Category.waypoint:
				for (ID2, value) in enumerate(row):
					if value == 1:
						suc.append(self.info_dict[ID2])
				print(self.info_dict[ID1])
				print(suc)
				print(" ")

	def runner(self):
		for ID in range(cf_num):
			f = flie(self.adj_array, self.info_dict, ID, self.pubTime)
			self.flie_list.append(f)
		for f in self.flie_list:
			f.publish_new_path()
		self.double_check()

	#publishes constant old paths
	#publishes new path when it is necessary
	#is really doing what runner is supposed to do
	def double_check(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			for f in self.flie_list:
				if f.running:
					if f.is_finished() and continuous:
						f.publish_new_path()
					else:
						f.publish_old_path()
			self.publish_reserved_IDs()
			rate.sleep()

	def publish_reserved_IDs(self):
		if show_reserved:
			res_list = get_reserved_IDs()
			self.pubRes.publish(res_list)


def fill_air_buffer_dict(info_dict):
	global air_buffer_dict
	for ID in info_dict:
		c = info_dict[ID][1]
		if c != Category.cloud and c != Category.interface and c != Category.air_waypoint:
			air_buffer_dict[ID] = [ID]
		else:
			(x1, y1, z1) = info_dict[ID][0]
			buffered = [ID]
			for ID2 in info_dict:
				if ID2 != ID:
					c2 = info_dict[ID2][1]
					if c2 == Category.cloud or c2 == Category.interface or c2 == Category.air_waypoint:
						(x2, y2, z2) = info_dict[ID2][0]
						dist = ((x2 - x1)**2 + (y2 - y1)**2 + buffer_z_frac * (z2 - z1)**2)**.5
						if dist <= air_buffer_dist:
							buffered.append(ID2)
			air_buffer_dict[ID] = buffered
	#print(air_buffer_dict)

#waits for setup to tell it to go
def waiter(info_dict, A):
	def start(data):
		print('started!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
		fs = full_system(A, info_dict)

	#wait for setup to tell you to go
	rospy.Subscriber('~Starter', Bool, start)

	rospy.spin()



if __name__ == "__main__":
	print('test')
	rospy.init_node('highlighter', anonymous = True)
	(info_dict, A) = map_maker_helper.map_maker_client('send_complex_map')
	Category = map_maker_helper.Category
	fill_air_buffer_dict(info_dict)
	starting_time = time.time()
	for ID in info_dict:
		si_dict[ID] = [(starting_time, starting_time*1000)]
	waiter(info_dict, A)