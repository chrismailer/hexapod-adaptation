import pybullet_utils.bullet_client as bc
import pybullet as p
import pybullet_data
import numpy as np
import os


class Simulator:
	"""This class is a wrapper for simulating the RARL Hexapod with the PyBullet physics engine

	Note
	----
	This class is configured to automatically start a PyBullet instance on an available CPU core

    Attributes
    ----------    	
    urdf : str
    	Filename of URDF model of hexapod. 
    controller : :obj:'Controller'
    	Positional controller object responsible for providing joint angles and velocities at given time
    visualiser_enabled : bool
    	Whether to display a GUI
    collision_fatal : bool
    	Whether collisions between links raise an exception
    locked_joints : :obj:`list` of :obj:`int`
    	A list of joint numbers which should be fixed in their retracted positions
    failed_joints : :obj:`list` of :obj:`int`
    	A list of joint numbers which should simulated as failed

    """
	def __init__(self, urdf, controller, visualiser=False, collision_fatal=True, locked_joints=[], failed_joints=[]):
		self.t = 0 #: float: Current time of the simulator
		self.dt = 1/240  #: float: Timestep of simulator. Default is 1/240s for PyBullet.
		self.gravity = -9.81 #: float: Magnitude of gravity vector in the positive z direction
		self.controller = controller
		self.visualiser_enabled = visualiser
		self.collision_fatal = collision_fatal
		self.locked_joints = locked_joints
		self.failed_joints = failed_joints

		self.camera_position = [0, 0, self.controller.body_height] #: list of float: GUI camera focus position in cartesian coordinates
		self.camera_distance = 0.5 #: float: GUI camera distance from camera position
		self.camera_yaw = -20 #: float: GUI camera yaw in degrees
		self.camera_pitch = -30 #: float: GUI camera pitch in degrees
		
		if self.visualiser_enabled:
			self.client = bc.BulletClient(connection_mode=p.GUI)
			self.client.resetDebugVisualizerCamera(cameraDistance=self.camera_distance, cameraYaw=self.camera_yaw, cameraPitch=self.camera_pitch, cameraTargetPosition=self.camera_position)
			self.client.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
			self.client.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, False)
			self.client.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, False)
		else:
			self.client = bc.BulletClient(connection_mode=p.DIRECT)

		self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
		self.client.setGravity(0, 0, self.gravity)
		self.client.setRealTimeSimulation(0) # simulation needs to be explicitly stepped

		# profiling
		# self.logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "timings.json")

		# Add ground plane and set lateral friction coefficient
		self.groundId = self.client.loadURDF("plane.urdf") #: int: Body ID of ground
		self.client.changeDynamics(self.groundId, -1, lateralFriction=1.0)

		# Add hexapod URDF
		position = [0, 0, self.controller.body_height]
		orientation = self.client.getQuaternionFromEuler([0, 0, -controller.crab_angle])
		filepath = os.path.abspath(os.path.dirname(__file__)) + urdf
		self.hexId = self.client.loadURDF(filepath, position, orientation, flags=(p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION)) #: int: Body ID of hexapod robot

		self.joints = self.__get_joints(self.hexId) #: list of int: List of joint indeces
		self.links = self.__get_links(self.joints) #: list of int: List of link indeces

		self.__init_joints(self.controller, self.joints, self.locked_joints)
		self.__init_links(self.links)


	# set joints to their initial positions
	def __init_joints(self, controller, joints, locked_joints):
		joint_angles = controller.joint_angles(t=0)
		for index, joint in enumerate(joints):
			joint_angle = joint_angles[index]
			joint_index, lower_limit, upper_limit, max_torque, max_speed = joint
			# not guaranteed that joint is present
			if joint_index is None: continue

			# if joint is locked, set it to fixed angle
			if index in locked_joints:
				joint_speed = 0
				if joint_index in np.arange(18)[0::3]:
					joint_angle = np.radians(0)
				elif joint_index in joints[1::3]:
					joint_angle = np.radians(90)
				elif joint_index in joints[2::3]:
					joint_angle = np.radians(-165)

			# set joints to their starting position
			self.client.resetJointState(self.hexId, joint_index, targetValue=joint_angle)
			
			if index in locked_joints: continue

			# ensure actuator behaves as unpowered servo
			# assign small friction force to joint to simulate servo friction
			self.client.setJointMotorControl2(self.hexId, joint_index, p.VELOCITY_CONTROL, force=0.1)


	def __init_links(self, links):
		tibia_links = links[:, 2]
		for link_index in tibia_links:
			# assign friction to feet to ensure they are not dragged
			self.client.changeDynamics(self.hexId, link_index, lateralFriction=1.0)

		# remove collisions between femur and base as this was preventing full coxa range of motion
		femur_links = links[:, 1]
		for link_index in femur_links:
			self.client.setCollisionFilterPair(self.hexId, self.hexId, linkIndexA=-1, linkIndexB=link_index, enableCollision=0)

	# Fetches and stores the joint index and joint information in the expected order
	def __get_joints(self, robotId):
		# A lot of the joint information in the URDF is not used in setJointMotorControl and needs to be manually applied
		joint_names = [b'joint_1_1', b'joint_1_2', b'joint_1_3', b'joint_2_1', b'joint_2_2', b'joint_2_3', b'joint_3_1', b'joint_3_2', b'joint_3_3', b'joint_4_1', b'joint_4_2', b'joint_4_3', b'joint_5_1', b'joint_5_2', b'joint_5_3', b'joint_6_1', b'joint_6_2', b'joint_6_3']
		joints = np.full((len(joint_names), 5), None)
		for joint_index in range(self.client.getNumJoints(robotId)):
			info = self.client.getJointInfo(robotId, joint_index)
			try:
				index = joint_names.index(info[1])
				# [ joint_index, lower_limit, upper_limit, max_torque, max_velocity ]
				joints[index] = [info[0], info[8], info[9], info[10], info[11]]
			except ValueError:
				print("Unexpected joint name in URDF")
		return joints


	def __get_links(self, joints):
		# In pybullet the linkIndex is the jointIndex
		link_indices = self.joints[:,0]
		links = link_indices.reshape(6,3)
		return links


	def terminate(self):
		"""Closes PyBullet physics engine and frees up system resources

        Note
        ----
        Prints the PyBullet error if termination failed

        """
		try:
			# p.stopStateLogging(self.logId)
			self.client.disconnect()
		except p.error as e:
			print("Termination of simulation failed:", e)


	def step(self):
		# using setJointMotorControl2 (slightly slower but allows setting of max velocity)
		joint_angles = self.controller.joint_angles(t=self.t)
		joint_speeds = self.controller.joint_speeds(t=self.t)

		for index, joint in enumerate(self.joints):
			# get joint properties
			joint_index, lower_limit, upper_limit, max_torque, max_speed = joint
			
			# skip if joint is not present of if failed
			if (joint_index is None) or (index in self.locked_joints) or (index in self.failed_joints): continue

			joint_angle = joint_angles[index]
			joint_speed = joint_speeds[index]
			
			# ensure controller does not attempt to exceed joint limits
			joint_angle = min(max(lower_limit, joint_angle), upper_limit)
			joint_speed = min(max(-max_speed, joint_speed), +max_speed)

			# max velocity in URDF isn't used and needs to be assigned
			self.client.setJointMotorControl2(self.hexId, joint_index, p.POSITION_CONTROL, targetPosition=joint_angle, targetVelocity=joint_speed, force=max_torque, maxVelocity=max_speed)


		if self.collision_fatal:
			if self.__link_collision() or self.__ground_collision():
				raise RuntimeError("Link collision during simulation")

		# follow robot with camera
		if self.visualiser_enabled:
			self.client.resetDebugVisualizerCamera(cameraDistance=self.camera_distance, cameraYaw=self.camera_yaw, cameraPitch=self.camera_pitch, cameraTargetPosition=self.base_pos())

		self.client.stepSimulation()

		self.t += self.dt

	
	def supporting_legs(self):
		"""Determines the supporting legs for the hexapod

		Note
		----
		A leg is considered to be supporting if it is in constact with the ground.
		This method is used in plotting the gait sequence diagram.

        Returns
        ----
        list of bool: A list of booleans where the index is the leg number and where 'True' represents in contact with the ground.

        """
		tibia_links = self.links[:, 2]
		# Get contact points between hex and ground for the last stepSimulation call
		contact_points = np.array(self.client.getContactPoints(self.hexId, self.groundId), dtype=object)
		# get links for contact between ground and hexapod
		try:
			contact_links = contact_points[:, 3]
		except IndexError as e:
			contact_links = np.array([])
		supporting_legs = np.isin(tibia_links, contact_links)

		return supporting_legs


	# Check for collision between links in robot
	def __link_collision(self):
		# Get contact points between hex links
		# Collision between child and parent in URDF is disabled by default so shouldn't need to ignore this
		contact_points = np.asarray(self.client.getContactPoints(self.hexId, self.hexId), dtype=object)
		# also need to add in collision detection between robot and ground excluding tibia links
		return contact_points.size > 0


	def __ground_collision(self):
		tibia_links = self.links[:, 2]
		# Get contact points between hex and ground for the last stepSimulation call
		contact_points = np.array(self.client.getContactPoints(self.hexId, self.groundId), dtype=object)
		# get links for contact between ground and hexapod
		try:
			contact_links = contact_points[:, 3]
		except IndexError as e:
			contact_links = np.array([])
		# filter out tibia contacts
		contact_links = contact_links[~np.isin(contact_links, tibia_links)]

		# returns a boolean array of whether tibia is in contact with the ground or not
		return contact_links.size > 0


	# returns the base position without the orientation
	def base_pos(self):
		"""Returns the position of the hexapod base

		Note
		----
		The base orientation is not returned

        Returns
        ----
        list of float: The position of the hexapod base in cartesian coordinates

        """
		return self.client.getBasePositionAndOrientation(self.hexId)[0]

if __name__ == "__main__":
	help(Simulator)