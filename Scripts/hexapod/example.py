from simulator import Simulator
from controllers.stabilised import Controller, tripod_gait

controller = Controller(tripod_gait, body_height=0.12, velocity=0.4, crab_angle=-1.57)
simulator = Simulator(controller, urdf='/urdf/hexapod_simplified.urdf', follow=True, visualiser=True, collision_fatal=False, failed_legs=[])

while True:
	simulator.step()
