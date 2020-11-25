from hexpy.controllers.kinematic import Controller, reshape
from hexpy.simulator import Simulator
from copy import copy
import numpy as np
import GPy as GPy
import time


# load the CVT voronoi centroids from input archive
def load_centroids(filename):
	points = np.loadtxt(filename)
	return points

# load the generated map
def load_map(filename, dim=6, dim_ctrl=32):
	# print("Loading ",filename)
	data = np.loadtxt(filename)
	fit = data[:, 0]
	desc = data[:, 1:dim+1]
	x = data[:, 2*dim+1:]
	return fit, desc, x

# aquisition function for the bayesian optimization
def UCB(mu_map, kappa, sigma_map):
	GP = []
	for i in range(0, len(mu_map)):
		GP.append(mu_map[i] + kappa*sigma_map[i])
	return np.argmax(GP)


def MBOA(map_filename, centroids_filename, eval, max_iter):

	alpha = 0.90
	kappa = 0.05
	rho = 0.4
	variance_noise_square = 0.001

	dim_x = 6

	num_it = 0
	real_perfs, tested_indexes = [-1],[]
	X, Y = np.empty((0, dim_x)), np.empty((0,1))

	# load map and centroids
	centroids = load_centroids(centroids_filename)
	fits, descs, ctrls = load_map(map_filename, centroids.shape[1], 32)

	n_fits, n_descs, n_ctrls = np.array(fits), np.array(descs), np.array(ctrls)

	n_fits_real = copy(np.array(n_fits))
	fits_saved = copy(n_fits)

	started = False

	while((max(real_perfs) < alpha*max(n_fits_real)) and (num_it <= max_iter)):

		if started:
			#define GP kernel
			kernel = GPy.kern.Matern52(dim_x, lengthscale=rho, ARD=False) + GPy.kern.White(dim_x, np.sqrt(variance_noise_square))
			#define Gp which is here the difference between map perf and real perf
			m = GPy.models.GPRegression(X, Y, kernel)
			#predict means and variances for the difference btwn map perf and real perf
			means, variances = m.predict(n_descs)
			#Add the predicted difference to the map found in simulation
			for j in range(0, len(n_fits_real)):
				n_fits_real[j] = means[j] + fits_saved[j]

			#apply acquisition function to get next index to test
			index_to_test = UCB(n_fits_real, kappa, variances)
		else:
			index_to_test = np.argmax(n_fits)
			started = True
			real_perfs = []

		print("Testing index:", index_to_test)
		print("Expected performance:", n_fits_real[index_to_test])
		#if the behavior to test has already been tested, don't test it again
		if(index_to_test in tested_indexes):
			print("Behaviour already tested")
			break
		else:
			ctrl_to_test = n_ctrls[index_to_test]
			tested_indexes.append(index_to_test)
			
			# eval the performance
			real_perf, _ = eval(ctrl_to_test)
		
		num_it += 1

		# add descriptor and real performance
		X = np.append(X, n_descs[[index_to_test],:], axis=0)
		Y = np.append(Y, (np.array(real_perf)-fits_saved[index_to_test]).reshape((1,1)), axis=0)

		#store
		real_perfs.append(real_perf)

		# combine updated fitness values and reconstruct map
		new_map = np.loadtxt(map_filename)
		new_map[:,0] = n_fits_real

		
	o = np.argmax(real_perfs)
	best_index = tested_indexes[o]
	best_perf = real_perfs[o]

	return num_it, best_index, best_perf, new_map


if __name__ == "__main__":

	for i in range(3,7):
		
		num_its = []
		best_indexes = []
		best_perfs = []

		failed_leg_1 = i
		failed_leg_2 = (i % 6) + 1

		print("Testing legs", failed_leg_1, "and", failed_leg_2)

		for map_num in range(1,11):
			# need to redefine the evaluate function each time to include the failed leg
			def evaluate(x, duration=5.0, visualiser=False, summarise_descriptor=True, collisions=False):
				body_height, velocity, leg_params = reshape(x)
				# controller will return an error if parameters are not feasible
				try:
					controller = Controller(leg_params, body_height=body_height, velocity=velocity, crab_angle=-np.pi/6)
				except:
					return 0, np.zeros(6)
				# initialise simulator
				locked_joints = list(range((failed_leg_1-1)*3,failed_leg_1*3)) + list(range((failed_leg_2-1)*3,failed_leg_2*3))
				simulator = Simulator('/urdf/hex.urdf', controller, visualiser, collisions, locked_joints=locked_joints)
				# initialise fitness and descriptor
				fitness, descriptor = 0, np.full((6, 0), False)
				# simulator returns error if collision occurs
				for t in np.arange(0, duration, step=simulator.dt):
					start_time = time.perf_counter()
					try:
						simulator.step()
					except RuntimeError as error:
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


			num_it, best_index, best_perf, new_map = MBOA("./experiments/map_%d.dat" % map_num, "./centroids_40000_6.dat", evaluate, max_iter=40)
			num_its.append(num_it)
			best_indexes.append(best_index)
			best_perfs.append(best_perf)

		# print('Failed Leg: %d' % failed_leg_1)
		print(*best_perfs, sep=' ')

