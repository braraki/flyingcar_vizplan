#!/usr/bin/env python

from map_maker import map_maker_helper

Category = map_maker_helper.Category

mark_x = []
mark_y = []

ENERGY_WEIGHT = 0.2
TIME_WEIGHT = 1 - ENERGY_WEIGHT

cf_mass = .038

def get_time(info_dict, ID1, ID2, air_vel, land_vel, takeoff_vel = None, landing_vel = None):
	if takeoff_vel == None:
		takeoff_vel = air_vel
	if landing_vel == None:
		landing_vel = air_vel
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(fc):
		if z1 - .01 > fz:
			v = landing_vel
		elif fz - .01 > z1:
			v = takeoff_vel
		else:
			v = air_vel
	else:
		v = land_vel
	time_passed = dist_traveled / float(v)
	return(time_passed)

def optimal_cost(info_dict, ID1, ID2, air_vel, land_vel, timestep):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = dist_traveled/timestep
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(fc):
		vel = air_vel
	wait_energy = get_wait_energy(info_dict, ID1, timestep)
	move_energy = get_move_energy_opt(info_dict, ID1, ID2, timestep, air_vel, land_vel)

	return(TIME_WEIGHT*timestep + ENERGY_WEIGHT*(wait_energy+move_energy))

def get_energy(info_dict, ID1, ID2, travel_time, air_vel, land_vel, takeoff_vel = None, landing_vel = None):
	if takeoff_vel == None:
		takeoff_vel = air_vel
	if landing_vel == None:
		landing_vel = air_vel
	((x1, y1, z1),c1) = info_dict[ID1]
	((x2, y2, z2), c2) = info_dict[ID2]
	dist_traveled = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**.5
	vel = land_vel
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(c2):
		if z1 - .01 > z2:
			vel = landing_vel
		elif z2 - .01 > z1:
			vel = takeoff_vel
		else:
			vel = air_vel
	expected_time = dist_traveled/float(vel)
	if expected_time < travel_time:
		wait_energy = get_wait_energy(info_dict, ID1, travel_time - expected_time)
		move_energy = get_move_energy(info_dict, ID1, ID2, expected_time)
	else:
		wait_energy = 0
		move_energy = get_move_energy(info_dict, ID1, ID2, travel_time)
	return(wait_energy + move_energy)

def get_opt_energy(info_dict, ID1, ID2, air_vel, land_vel, timestep):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	vel = land_vel
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(fc):
		vel = air_vel
	expected_time = dist_traveled/float(vel)
	if ID1 == ID2:
		wait_energy = get_wait_energy(info_dict, ID1, timestep)
		move_energy = get_move_energy_opt(info_dict, ID1, ID2, timestep, air_vel, land_vel)
	else:
		wait_energy = 0
		move_energy = get_move_energy_opt(info_dict, ID1, ID2, timestep, air_vel, land_vel)
	return(wait_energy + move_energy)

def get_wait_energy(info_dict, ID1, wait_time):
	((x1, y1, z1),c1) = info_dict[ID1]
	if map_maker_helper.is_air(c1):
		wait_energy_expenditure = 8
	else:
		wait_energy_expenditure = 0.1
	return(wait_time * wait_energy_expenditure)


#I dont know what this is
def get_move_energy_opt(info_dict, ID1, ID2, move_time, air_vel, land_vel):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(fc):
		const_energy_expenditure = 7.8
	else:
		const_energy_expenditure = .6
	TE = const_energy_expenditure*dist_traveled
	PE = max((fz-z1), 0)*cf_mass*9.8
	return(TE + PE)

def get_move_energy(info_dict, ID1, ID2, move_time):
	((x1, y1, z1),c1) = info_dict[ID1]
	((fx, fy, fz), fc) = info_dict[ID2]
	dist_traveled = ((x1-fx)**2 + (y1-fy)**2 + (z1-fz)**2)**.5
	if map_maker_helper.is_air(c1) or map_maker_helper.is_air(fc):
		const_energy_expenditure = 7.8
	else:
		const_energy_expenditure = .6
	TE = const_energy_expenditure*move_time
	PE = max((fz-z1), 0)*cf_mass*9.8
	return(TE + PE)

def get_cost(energy, time):
	energy_coefficient = .25
	time_coefficient = 1 - energy_coefficient
	cost = energy*energy_coefficient + time*time_coefficient
	return(cost)

def get_voltage(energy):
	#battery is 240mAh
	return(energy)

def current_draw(power, distance, velocity):
	current = power/3.7
	current_draw = current*distance/velocity
	return current_draw