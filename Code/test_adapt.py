from hexapod.controllers.kinematic import Controller, reshape
from hexapod.simulator import Simulator
from MBOA import MBOA
import numpy as np

sim = 1.4456
real = 1.3125


if __name__ == "__main__":
		
	num_its = []
	best_indexes = []
	best_perfs = []

	for failed_legs in [[1],[2],[3],[4],[5],[6]]:
		for map_n in range(1,6):

			def evaluate(x, duration=5.0, visualiser=False, collisions=False):
				body_height, velocity, leg_params = reshape(x)
				# controller will return an error if parameters are not feasible
				try:
					controller = Controller(leg_params, body_height=body_height, velocity=velocity, crab_angle=-np.pi/6)
				except:
					return 0, np.zeros(6)
				# initialise simulator
				simulator = Simulator('/urdf/hexapod.urdf', controller, visualiser, collisions, failed_legs=failed_legs)
				# initialise fitness and descriptor
				fitness, contacts = 0, np.full((6, 0), False)
				# simulator returns error if collision occurs
				for t in np.arange(0, duration, step=simulator.dt):
					try:
						simulator.step()
					except RuntimeError as collision:
						fitness = 0
						break
					fitness = simulator.base_pos()[0]
					contacts = np.append(contacts, simulator.supporting_legs().reshape(-1,1), axis=1)
				if summarise_descriptor:
					descriptor = np.sum(contacts, axis=1) / np.size(contacts, axis=1)
					descriptor = np.nan_to_num(descriptor, nan=0.0, posinf=0.0, neginf=0.0)
				simulator.terminate()
				return fitness, descriptor

			num_it, best_index, best_perf = MBOA("/maps/niches_20000/map_%d.dat" % map_n, "/centroids/centroids_40000_6.dat", evaluate, max_iter=40)
	
	num_its.append(num_it)
	best_indexes.append(best_index)
	best_perfs.append(best_perf)

	# print('Failed Leg: %d' % failed_leg_1)
	print(num_it, best_index, best_perf)
