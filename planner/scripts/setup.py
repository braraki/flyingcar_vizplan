#!/usr/bin/env python

import rospy
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

from map_maker import gen_adj_array_info_dict


import sys, select, termios, tty

cf_num = int(rospy.get_param('/setup/cf_num'))
centered = bool(rospy.get_param('/setup/centered'))




set_start = False
start_list = []


park_dict = {}



def set_IDs():
	starting_IDs = []
	for cf in range(cf_num):
		starting_IDs.append(set_ID(starting_IDs, cf))
	return(starting_IDs)




def set_ID(used, cf_num):
	if set_start:
		if cf_num <= len(start_list) - 1:
			return(start_list[cf_num])
	elif centered:
		min_dist = None
		chosen = None
		for ID in park_dict:
			if ID not in used:
				dist = least_distance(park_dict[ID], used)
				if min_dist == None:
					min_dist = dist
					chosen = ID
				elif dist < min_dist:
					min_dist = dist
					chosen = ID
		return(chosen)
	else:
		unfound = True
		while unfound:
			ID = random.choice(park_dict.keys())
			if ID not in used:
				return(ID)



def least_distance(point, used):
	(x, y, z) = point
	dist = (x**2 +y**2 + z**2)**.5
	for ID2 in used:
		(x2, y2, z2) = park_dict[ID2]
		new_d = ((x2 - x)**2 + (y2 - y)**2 + (z2 - z)**2)**.5
		#dist -= new_d
	return(dist)

def setter():
	starting_IDs = set_IDs()
	print(starting_IDs)
	x_list = []
	y_list = []
	z_list = []
	for ID in starting_IDs:
		x_list.append(park_dict[ID][0])
		y_list.append(park_dict[ID][1])
		z_list.append(park_dict[ID][2])
	PosPub = rospy.Publisher('~Start_SimPos_topic', SimPos, queue_size = 10)
	IDPub = rospy.Publisher('~StartingID_topic', setup_IDs, queue_size = 10)
	StartPub = rospy.Publisher('~Starter', Bool, queue_size = 10)
	#waits so that subscriber will recieve message
	time.sleep(1)
	print('Are you ready to begin, if so type Y: ')
	go = 'N'
	while go != 'Y' and go != 'y':
		if not rospy.is_shutdown():
			PosPub.publish(x_list, y_list, z_list)
			IDPub.publish(starting_IDs)
		else:
			break
		#go = raw_input('Are you ready to begin, if so type Y: ')
		go = get_key()
	StartPub.publish(True)



def get_key():
	tty.setraw(sys.stdin.fileno())
	r_list, _, _ = select.select([sys.stdin], [], [], .1)
	if r_list:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return(key)




if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	#print('test')
	rospy.init_node('setup', anonymous = True)
	(info_dict, A) = gen_adj_array_info_dict.map_maker_client('send_complex_map')
	Category = gen_adj_array_info_dict.Category
	for ID in info_dict:
		c = info_dict[ID][1]
		if c == Category.park:
			park_dict[ID] = info_dict[ID][0]
	setter()
