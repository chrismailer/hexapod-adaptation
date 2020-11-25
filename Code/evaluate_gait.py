from hexpy.controllers.kinematic import Controller, reshape
from hexpy.simulator import Simulator
import numpy as np


# function responsible for testing the gait parameters and returning the descriptor and performance
# pass in a 32 length array of parmeters between 0.0 and 1.0
def evaluate(x, duration=5.0):
	body_height, velocity, leg_params = reshape(x)
	# controller will return an error if parameters are not feasible
	try:
		controller = Controller(body_height=0.14, velocity=0.3, period=1.0, crab_angle=-np.pi/6)
	except:
		return 0, np.zeros(6)
	# initialise simulator
	simulator = Simulator('./hexapod/urdf/hex.urdf', controller, visualiser=True, collision_fatal=True)
	# initialise reward and descriptor
	reward = 0
	descriptor = np.full((6, 0), False)
	# simulator returns error if collision occurs
	for t in np.arange(0, duration, step=simulator.dt):
		try:
			simulator.step()
		except RuntimeError as error:
			reward = 0
			break
		reward = simulator.base_pos()[0] # distance travelled along x axis
		descriptor = np.append(descriptor, simulator.supporting_legs().reshape(-1,1), axis=1)
	# summarise descriptor
	descriptor = np.nan_to_num(np.sum(descriptor, axis=1) / np.size(descriptor, axis=1), nan=0.0, posinf=0.0, neginf=0.0)
	simulator.terminate()

	return reward, descriptor



if __name__ == "__main__":
	from hexapod.controllers.kinematic import tripod_gait
	tripod_gait = [0.7, 0.3,0.5, 0.5, 0.3, 0.5, 0.0, 0.5, 0.5, 0.3, 0.5, 0.5, 0.5, 0.5, 0.3, 0.5, 0.0, 0.5, 0.5, 0.3, 0.5, 0.5, 0.5, 0.5, 0.3, 0.5, 0.0, 0.5, 0.5, 0.3, 0.5, 0.5]

	reward, descriptor = evaluate(tripod_gait, duration=5.0)

	print('reward=', reward)
	print('descriptor=', descriptor)
