from hexapod.controllers.kinematic import Controller, reshape
from hexpy.simulator import Simulator
import map_elites.cvt as cvt_map_elites
import numpy as np


# function responsible for testing the gait parameters and returning the descriptor and performance
# pass in a 32 length array of parmeters between 0.0 and 1.0
def evaluate_gait(x):
    body_height, velocity, leg_params = reshape(x)
    # controller will return an error if parameters result in a leg singularity
    try:
        controller = Controller(leg_params, body_height=body_height, velocity=velocity, period=1.0, crab_angle=-np.pi/6)
    except:
        return 0, np.zeros(6)
    # initialise simulator
    simulator = Simulator(controller=controller, visualiser=True, collision_fatal=True)
    reward = 0
    descriptor = np.full((6, 0), False)
    for t in np.arange(0, 5.0, step=simulator.dt):
        try:
            simulator.step()
        except RuntimeError as collision:
            reward = 0
            break
        reward = simulator.base_pos()[0] # distance travelled along x axis
        descriptor = np.append(descriptor, simulator.supporting_legs().reshape(-1,1), axis=1)
    # summarise descriptor
    descriptor = np.nan_to_num(np.sum(descriptor, axis=1) / np.size(descriptor, axis=1), nan=0.0, posinf=0.0, neginf=0.0)
    simulator.terminate()

    return reward, descriptor


if __name__ == '__main__':
    params = \
        {
            # more of this -> higher-quality CVT (400000)
            "cvt_samples": 15e6,
            # we evaluate in batches to parallelise
            "batch_size": 2390,
            # proportion of niches to be filled before starting (400)
            "random_init": 0.01,
            # batch for random initialization
            "random_init_batch": 2390,
            # when to write results (one generation = one batch)
            "dump_period": 1e6,
            # do we use several cores?
            "parallel": True,
            # do we cache the result of CVT and reuse?
            "cvt_use_cache": True,
            # min/max of parameters
            "min": 0,
            "max": 1,
        }

    archive = cvt_map_elites.compute(6, 32, evaluate_gait, n_niches=40000, max_evals=40e6, log_file=open('log.dat', 'w'), params=params)

