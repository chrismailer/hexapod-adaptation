import numpy as np
from plots.map import plot_map
from MBOA import MBOA

np.set_printoptions(precision=3, suppress=True)

# real_perf = np.loadtxt("./real_experiments/failure_1&4_4.dat")

# index = 0

# def eval(x):
# 	global index
# 	perf = real_perf[index, 1]
# 	print("Real performance:", perf)
# 	index += 1
# 	return perf, []


# if __name__ == "__main__":
# 	num_it, best_index, best_perf, new_map = MBOA("./experiments/map_4.dat", "./centroids_40000_6.dat", eval, max_iter=40)
# 	plot_map(new_map, 2)


for map_n in range(1,11):
	MAP = np.loadtxt('./maps/map_%d.dat' % map_n)
	max_fitness = round(np.max(MAP[:,0]) / 5, 3)
	n_gaits = MAP.shape[0]
	print(max_fitness, n_gaits)