from hexpy.controllers.kinematic import Controller, reshape
from plots.footfall import plot_footfall
from hexpy.simulator import Simulator
import numpy as np
import time

# np.set_printoptions(precision=4)
# np.warnings.filterwarnings('error', category=np.VisibleDeprecationWarning)

# function responsible for testing the gait parameters and returning the descriptor and performance
# pass in a 32 length array of parmeters between 0.0 and 1.0
def simulate_gait(x, duration=5.0, visualiser=True, summarise_descriptor=True, collisions=False, failed_legs=[]):
	# controller will return an error if parameters are not feasible
	body_height, velocity, leg_params = reshape(x)
	try:
		controller = Controller(leg_params, body_height=body_height, velocity=velocity, crab_angle=-np.pi/6)
	except:
		return 0, np.zeros(6)
	# initialise simulator
	simulator = Simulator(urdf='./hexpy/urdf/hexapod.urdf', controller=controller, follow=True, visualiser=visualiser, collision_fatal=collisions, failed_legs=failed_legs, camera_position=[0, 0.14, 0], camera_distance=0.7, camera_yaw=20, camera_pitch=-30)
	# initialise reward and descriptor
	reward = 0
	contact_sequence = np.full((6, 0), False)

	# simulator returns error if collision occurs
	for t in np.arange(0, duration, step=simulator.dt):
		start_time = time.perf_counter()
		try:
			simulator.step()
		except RuntimeError as error:
			print(error)
			reward = 0
			break

		reward = simulator.base_pos()[0]
		contact_sequence = np.append(contact_sequence, simulator.supporting_legs().reshape(-1,1), axis=1)

		end_time = time.perf_counter()
		elapsed = end_time - start_time

		if visualiser:
			time.sleep(max(simulator.dt-elapsed, 0))

	# summarise descriptor
	descriptor = np.sum(contact_sequence, axis=1) / np.size(contact_sequence, axis=1)
	# plot footfall diagram
	# plot_footfall(contact_sequence)

	simulator.terminate()

	return reward, descriptor



# place to test out gaits

if __name__ == "__main__":

	log = np.loadtxt('./maps/map_6.dat')
	row_index = np.argmax(log, axis=0)[0]
	row_index = np.random.randint(0, log.shape[0])
	row_index = 26269

	fitness = log[row_index, 0]
	desc = log[row_index, 1:7]
	centroid = log[row_index, 7:13]
	params = log[row_index, 13:]

	print(row_index)

	fitness, descriptor = simulate_gait(params, duration=5.0, summarise_descriptor=False, collisions=False, visualiser=True)
	# plot(descriptor)
