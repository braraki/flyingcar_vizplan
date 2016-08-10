#!/usr/bin/env python

import sys
import io
import os

size = int(sys.argv[1])
block_size = int(sys.argv[2])

class city:
	def __init__(self, size, block_size):
		self.size = size
		self.block_size = block_size
		self.num_long = size
		self.num_wide = size
		self.non_fly_list = []
		self.map_pre_dict = {}
		for x in range(self.num_long):
			for y in range(self.num_wide):
				chosen = []
				spot = str((x,y))
				if x%(self.block_size+1) == 0:
					if y > 0:
						chosen.append('S')
					if y < (self.num_wide - 1):
						chosen.append('N')
				if y%(self.block_size+1) == 0:
					if x > 0:
						chosen.append('W')
					if x < (self.num_long - 1):
						chosen.append('E')
				self.map_pre_dict[spot] = chosen
		for spot in self.map_pre_dict:
			if self.map_pre_dict[spot] == []:
				self.non_fly_list.append(spot)

	def make_yaml(self):
		directory = (os.path.dirname(os.path.realpath(__file__)))
		name = directory+'/city_s'+str(self.size)+'_bs'+str(self.block_size)+'.yaml'
		with io.FileIO(name, "w") as file:
			file.write('map_num_long: '+str(self.num_long)+'\n')
			file.write('map_num_wide: '+str(self.num_wide)+'\n')
			file.write('non_fly_list: '+str(self.non_fly_list)+'\n')
			file.write('map_pre_dict: '+str(self.map_pre_dict)+'\n')






c = city(size, block_size)
c.make_yaml()


