import time
from simulator import Simulator
from controllers.kinematic import Controller, tripod_gait

controller = Controller(tripod_gait, body_height=0.15, velocity=0.3, crab_angle=0)
simulator = Simulator(controller, follow=False, visualiser=True, collision_fatal=False)

while True:
	simulator.step()
	time.sleep(simulator.dt)
