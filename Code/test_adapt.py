from hexpy.controllers.kinematic import Controller, reshape
from hexpy.simulator import Simulator
from MBOA import MBOA
import numpy as np
import time

sim = 1.4456
real = 1.3125


def evaluate(x, duration=5.0, visualiser=False, summarise_descriptor=True, collisions=False):
	body_height, velocity, leg_params = reshape(x)
	# controller will return an error if parameters are not feasible
	try:
		controller = Controller(leg_params, body_height=body_height, velocity=velocity, crab_angle=-np.pi/6)
	except:
		return 0, np.zeros(6)
	# initialise simulator
	simulator = Simulator('/urdf/hex.urdf', controller, visualiser, collisions, locked_joints=[])
	# initialise fitness and descriptor
	fitness, descriptor = 0, np.full((6, 0), False)
	# simulator returns error if collision occurs
	for t in np.arange(0, duration, step=simulator.dt):
		start_time = time.perf_counter()
		try:
			simulator.step()
		except RuntimeError as collision:
			fitness = 0
			break
		fitness = simulator.base_pos()[0]
		descriptor = np.append(descriptor, simulator.supporting_legs().reshape(-1,1), axis=1)

		end_time = time.perf_counter()
		elapsed_time = end_time - start_time

		if visualiser:
			time.sleep(max(1/240-elapsed_time, 0))
	# summarise descriptor
	if summarise_descriptor:
		descriptor = np.sum(descriptor, axis=1) / np.size(descriptor, axis=1)
		descriptor = np.nan_to_num(descriptor, nan=0.0, posinf=0.0, neginf=0.0)
	simulator.terminate()

	return fitness, descriptor


if __name__ == "__main__":
		
	num_its = []
	best_indexes = []
	best_perfs = []

	# need to redefine the evaluate function each time to include the failed leg
	num_it, best_index, best_perf = MBOA("./experiments/map_2.dat", "./centroids_40000_6.dat", evaluate, max_iter=40)
	num_its.append(num_it)
	best_indexes.append(best_index)
	best_perfs.append(best_perf)

	# print('Failed Leg: %d' % failed_leg_1)
	print(num_it, best_index, best_perf)
