import numpy as np


# radius, offset, step_height, phase, duty_cycle, period
tripod_gait = [	0.7, 0.3,
				0.5, 0.5, 0.3, 0.0, 0.5, 0.0, # leg 0
				0.5, 0.5, 0.3, 0.5, 0.5, 0.0, # leg 1
				0.5, 0.5, 0.3, 0.0, 0.5, 0.0, # leg 2
				0.5, 0.5, 0.3, 0.5, 0.5, 0.0, # leg 3
				0.5, 0.5, 0.3, 0.0, 0.5, 0.0, # leg 4
				0.5, 0.5, 0.3, 0.5, 0.5, 0.0] # leg 5


wave_gait = [	0.7, 0.4,
				0.6, 0.5, 0.1, 0/6, 5/6, 1.0, # leg 0
				0.6, 0.5, 0.1, 1/6, 5/6, 1.0, # leg 1
				0.6, 0.5, 0.1, 2/6, 5/6, 1.0, # leg 2
				0.6, 0.5, 0.1, 3/6, 5/6, 1.0, # leg 3
				0.6, 0.5, 0.1, 4/6, 5/6, 1.0, # leg 4
				0.6, 0.5, 0.1, 5/6, 5/6, 1.0] # leg 5


quadruped_gait = [	0.7, 0.4,
				0.6, 0.5, 0.1, 0/3, 2/3, 1.0, # leg 0
				0.6, 0.5, 0.1, 1/3, 2/3, 1.0, # leg 1
				0.6, 0.5, 0.1, 2/3, 2/3, 1.0, # leg 2
				0.6, 0.5, 0.1, 0/3, 2/3, 1.0, # leg 3
				0.6, 0.5, 0.1, 1/3, 2/3, 1.0, # leg 4
				0.6, 0.5, 0.1, 2/3, 2/3, 1.0] # leg 5


class Controller:
	def __init__(self, leg_params, crab_angle=0.0, body_height=0.14, velocity=0.1, dt=1/240):
		# link lengths
		self.l_1 = 0.05317
		self.l_2 = 0.10188
		self.l_3 = 0.14735

		self.dt = dt
		self.velocity = velocity
		self.crab_angle = crab_angle
		self.body_height = body_height

		self.angles = []
		self.speeds = []

		leg_params = np.array(leg_params).reshape(6, 6)

		self.periods = leg_params[:,5]

		# create trajectory for each leg
		for leg_index in range(0, 6):
			leg_positions, leg_velocities = self.__leg_traj(leg_index, leg_params[leg_index])
			joint_angles, joint_speeds = self.__inverse_kinematics(leg_positions, leg_velocities)
			# check that the position is achieveble
			achieved_positions = self.forward_kinematics(joint_angles)
			valid = np.all(np.isclose(leg_positions, achieved_positions))
			if not valid:
				raise RuntimeError("Desired foot trajectory not achieveable")

			self.angles.extend(np.ndarray.tolist(joint_angles))
			self.speeds.extend(np.ndarray.tolist(joint_speeds))



	def joint_angles(self, t):
		angles = []
		for i, angle in enumerate(self.angles):
			period = self.periods[int(i/3)]
			k = int(((t % period) / period) * len(angle))
			angles.append(angle[k])

		return angles


	def joint_speeds(self, t):
		speeds = []
		for i, speed in enumerate(self.speeds):
			period = self.periods[int(i/3)]
			k = int(((t % period) / period) * len(speed))
			speeds.append(speed[k])

		return speeds


	# Returns x, y, z trajectory path points for a leg in the leg coordinate space
	def __leg_traj(self, leg_index, leg_params):
		leg_angle = (np.pi / 3.0) * (leg_index)
		radius, offset, step_height, phase, duty_factor, period = leg_params

		stride = self.velocity * duty_factor * period

		# key points in path
		mid = np.zeros(3)
		mid[0] = radius * np.cos(offset)
		mid[1] = radius * np.sin(offset)
		mid[2] = -self.body_height + 0.014 + step_height

		start = np.zeros(3)
		start[0] = mid[0] + (stride / 2) * np.cos(-leg_angle + self.crab_angle)
		start[1] = mid[1] + (stride / 2) * np.sin(-leg_angle + self.crab_angle)
		start[2] = -self.body_height + 0.014

		end = np.zeros(3)
		end[0] = mid[0] - (stride / 2) * np.cos(-leg_angle + self.crab_angle)
		end[1] = mid[1] - (stride / 2) * np.sin(-leg_angle + self.crab_angle)
		end[2] = -self.body_height + 0.014

		array_dim = period / self.dt

		# compute support path
		support_dim = int(np.around(array_dim * duty_factor))
		support_positions, support_velocities = self.__support_traj(start, end, support_dim)

		# compute swing path
		swing_dim = int(np.around(array_dim * (1.0 - duty_factor)))
		swing_positions, swing_velocities = self.__swing_traj(end, mid, start, swing_dim)

		# combine support and swing paths
		positions = np.append(support_positions, swing_positions, axis=1)
		velocities = np.append(support_velocities, swing_velocities, axis=1)

		# shift points according to phase
		phase_shift = int(np.around(phase * array_dim))
		positions = np.roll(positions, phase_shift, axis=1)
		velocities = np.roll(velocities, phase_shift, axis=1)

		return positions, velocities


	def __support_traj(self, start, end, num):
		# Straight line from start to end point
		positions = np.linspace(start, end, num, axis=1)
		# the servo velocity is the same for each position
		duration = num * self.dt
		# duration can sometimes be 0
		with np.errstate(divide='ignore', invalid='ignore'):
			velocity = ((end - start) / duration).reshape(3,1)
		# velocities = np.nan_to_num(velocities, nan=0.0, posinf=0.0, neginf=0.0)
		velocities = np.tile(velocity, num)

		return positions, velocities



	# Generates a smooth trajectory from start point to end point with a 6th order polynomial
	def __swing_traj(self, start, via, end, num):
		t = np.ones((7, num))
		tf = num * self.dt
		time = np.linspace(0, tf, num)
		# time = np.linspace(0, tf, num)

		for i in range(7):
			t[i,:] = np.power(time, i)

		a_0 = start
		a_1 = np.zeros(3)
		a_2 = np.zeros(3)
		a_3 = (2 / (tf ** 3)) * (32 * (via - start) - 11 * (end - start))
		a_4 = -(3 / (tf ** 4)) * (64 * (via - start) - 27 * (end - start))
		a_5 = (3 / (tf ** 5)) * (64 * (via - start) - 30 * (end - start))
		a_6 = -(32 / (tf ** 6)) * (2 * (via - start) - (end - start))

		positions = np.stack([a_0, a_1, a_2, a_3, a_4, a_5, a_6], axis=-1).dot(t)
		velocities = np.stack([a_1, 2*a_2, 3*a_3, 4*a_4, 5*a_5, 6*a_6, np.zeros(3)], axis=-1).dot(t)

		return positions, velocities


	# Inverse kinematics to calculate joint angles to reach provided points
	def __inverse_kinematics(self, foot_position, foot_speed):
		l_1, l_2, l_3 = self.l_1, self.l_2, self.l_3

		x, y, z = foot_position
		dx, dy, dz = foot_speed

		theta_1 = np.arctan2(y, x)

		c_1, s_1 = np.cos(theta_1), np.sin(theta_1)
		c_3 = ((x - l_1 * c_1)**2 + (y - l_1 * s_1)**2 + z**2 - l_2**2 - l_3**2) / (2 * l_2 * l_3)
		s_3 = -np.sqrt(np.maximum(1 - c_3**2, 0)) # maximum ensures not negative

		theta_2 = np.arctan2(z, (np.sqrt((x - l_1 * c_1)**2 + (y - l_1 * s_1)**2))) - np.arctan2((l_3 * s_3), (l_2 + l_3 * c_3))
		theta_3 = np.arctan2(s_3, c_3)

		c_2, s_2 = np.cos(theta_2), np.sin(theta_2)
		c_23 = np.cos(theta_2 + theta_3)

		with np.errstate(divide='ignore'):
			theta_dot_1 = (dy*c_1 - dx*s_1) / (l_1 + l_3*c_23 + l_2*c_2)
			theta_dot_2 = (1/l_2)*(dz*c_2 - dx*c_1*s_2 - dy*s_1*s_2 + (c_3 / s_3)*(dz*s_2 + dx*c_1*c_2 + dy*c_2*s_1))
			theta_dot_3 = -(1/l_2)*(dz*c_2 - dx*c_1*s_2 - dy*s_1*s_2 + ((l_2 + l_3*c_3)/(l_3*s_3))*(dz*s_2 + dx*c_1*c_2 + dy*c_2*s_1))

		theta_dot_1 = np.nan_to_num(theta_dot_1, nan=0.0, posinf=0.0, neginf=0.0)
		theta_dot_2 = np.nan_to_num(theta_dot_2, nan=0.0, posinf=0.0, neginf=0.0)
		theta_dot_3 = np.nan_to_num(theta_dot_3, nan=0.0, posinf=0.0, neginf=0.0)

		joint_angles = np.array([theta_1, theta_2, theta_3])
		joint_speeds = np.array([theta_dot_1, theta_dot_2, theta_dot_3])

		return joint_angles, joint_speeds


	def __joint_velocities(self, foot_speed, joint_angles):
		l_1, l_2, l_3 = self.l_1, self.l_2, self.l_3

		dx, dy, dz = foot_speed
		theta_1, theta_2, theta_3 = joint_angles

		c_1, s_1 = np.cos(theta_1), np.sin(theta_1)
		c_2, s_2 = np.cos(theta_2), np.sin(theta_2)
		c_3, s_3 = np.cos(theta_3), np.sin(theta_3)
		c_23 = np.cos(theta_2 + theta_3)

		theta_dot_1 = (dy*c_1 - dx*s_1) / (l_1 + l_3*c_23 + l_2*c_2)
		theta_dot_2 = (1/l_2)*(dz*c_2 - dx*c_1*s_2 - dy*s_1*s_2 + (c_3 / s_3)*(dz*s_2 + dx*c_1*c_2 + dy*c_2*s_1))
		theta_dot_3 = -(1/l_2)*(dz*c_2 - dx*c_1*s_2 - dy*s_1*s_2 + ((l_2 + l_3*c_3)/(l_3*s_3))*(dz*s_2 + dx*c_1*c_2 + dy*c_2*s_1))

		joint_speeds = np.array([theta_dot_1, theta_dot_2, theta_dot_3])

		return joint_speeds


	def forward_kinematics(self, joint_angles):
		l_1, l_2, l_3 = self.l_1, self.l_2, self.l_3
		theta_1, theta_2, theta_3 = joint_angles
		
		# Compute point from joint angles
		x = np.cos(theta_1) * (l_1 + l_3 * np.cos(theta_2 + theta_3) + l_2 * np.cos(theta_2))
		y = np.sin(theta_1) * (l_1 + l_3 * np.cos(theta_2 + theta_3) + l_2 * np.cos(theta_2))
		z = l_3 * np.sin(theta_2 + theta_3) + l_2 * np.sin(theta_2)

		return np.array([x, y, z])


if __name__ == '__main__':
	# Radius, offset, stride, step, phase, duty_cycle
	leg_params = [
	0.1, 0, 0.1, 0.0, 0.5, 0.5, # leg 0
	0.1, 0, 0.1, 0.5, 0.5, 1.0, # leg 1
	0.1, 0, 0.1, 0.0, 0.5, 1.0, # leg 2
	0.1, 0, 0.1, 0.5, 0.5, 1.0, # leg 3
	0.1, 0, 0.1, 0.0, 0.5, 1.0, # leg 4
	0.1, 0, 0.1, 0.5, 0.5, 1.0] # leg 5

	controller = Controller(leg_params)

	print(controller.joint_angles(t=0.505))
	
