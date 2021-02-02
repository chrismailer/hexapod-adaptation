from hexapod.controllers.kinematic import Controller, reshape
from hexapod.simulator import Simulator
from adapt.MBOA import MBOA
import numpy as np

# evaluate function
def evaluate_gait(x, duration=5.0):
	global failed_legs
	body_height, velocity, leg_params = reshape(x)
	try:
		controller = Controller(leg_params, body_height=body_height, velocity=velocity, crab_angle=-np.pi/6)
	except:
		return 0
	simulator = Simulator(controller, visualiser=False, collision_fatal=False, failed_legs=failed_legs)
	fitness = 0
	for t in np.arange(0, duration, step=simulator.dt):
		try:
			simulator.step()
		except RuntimeError as error:
			fitness = 0
			break
		fitness = simulator.base_pos()[0]
	simulator.terminate()
	return fitness


# experiment
n_maps = 10
niches = 20000
rhos = np.arange(0.1, 0.825, 0.025)
failures = [[1,3], [2,4], [3,5], [4,6], [5,1], [6, 2]]
failed_legs = failures[0]

print(rhos)

perfs = np.zeros((n_maps*len(failures), len(rhos)))
iters = np.zeros((n_maps*len(failures), len(rhos)))

for col_index, rho in enumerate(rhos):
	print("\nTesting ρ =", rho)
	for map_n in range(1, n_maps+1):
		print("\n\tTesting map", map_n, end=': ')
		for leg_index, legs in enumerate(failures):
			print(legs, end=' ')
			failed_legs = legs
			num_it, best_index, best_perf, new_map = MBOA(f"./maps/niches_{niches}/map_{map_n}.dat", f"./centroids/centroids_{niches}_6.dat", evaluate_gait, max_iter=20, rho=rho, print_output=False)
			row_index = len(failures) * (map_n - 1) + leg_index
			perfs[row_index, col_index] = best_perf
			iters[row_index, col_index] = num_it
			print(num_it, end=' ')

np.savetxt(f"./experiments/rho/perfs_{niches}.dat", perfs)
np.savetxt(f"./experiments/rho/iters_{niches}.dat", iters, '%d')
