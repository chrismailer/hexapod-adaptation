import numpy as np
import matplotlib
import matplotlib.pyplot as plt


def plot_map(data, map_num):
	# matplotlib.use("pgf")
	matplotlib.rcParams.update({
	    "pgf.texsystem": "pdflatex",
	    'font.family': 'serif',
	    'text.usetex': True,
	    'pgf.rcfonts': False,
	})

	fig, ax = plt.subplots()
	fig.set_size_inches(w=4.7747, h=3.5)

	A = np.zeros((125, 125))

	max_index = np.argmax(data[:,0])

	for index, row in enumerate(data):
		fitness = row[0] / 5
		desc = row[1:7]

		dim = 5

		x = int((np.ceil(desc[0]*dim)-1)*(dim**2) + (np.ceil(desc[1]*dim)-1)*dim + (np.ceil(desc[2]*dim)-1))
		y = int((np.ceil(desc[5]*dim)-1)*(dim**2) + (np.ceil(desc[4]*dim)-1)*dim + (np.ceil(desc[3]*dim)-1))

		if (fitness > A[x,y]):
			A[x,y] = fitness


	# show max value
	x, y = np.unravel_index(np.argmax(A), np.array(A).shape)
	plt.scatter((y/124)*100, (x/124)*100, color='red', s=4, marker='s')

	# plt.imshow(A, interpolation='none', origin='lower', vmin=0, vmax=0.5, extent=(0, 100, 0, 100))
	plt.imshow(A, interpolation='none', origin='lower', extent=(0, 100, 0, 100), cmap='inferno', vmin=0, vmax=0.5)
	clb = plt.colorbar()
	clb.ax.set_ylabel('Fitness ($m/s$)')

	# ax.set_title('Descriptor-Performance Map %d' % map_num)
	ax.set_title('Descriptor-Performance Map')
	# ax.set_title('Adapted Descriptor-Performance Map')
	# ax.set_xticks(np.linspace(0,100,26), minor=True)
	# ax.set_yticks(np.linspace(0,100,26), minor=True)
	# ax.xaxis.set_minor_formatter('{x:d}')
	# plt.tick_params(axis='both', which='both', bottom=False, top=False, labelbottom=False, left=False)
	ax.axis('off')

	# plt.ylabel('Right side descriptor (\%)')
	# plt.xlabel('Left side descriptor (\%)')
	# plt.grid(which='both', color='k')

	fig.tight_layout()

	# plt.savefig('histogram.pgf')

	plt.show()


if __name__ == "__main__":
	map_num = 1
	data = np.loadtxt('../maps/map_%d.dat' % map_num)
	# data = np.loadtxt('../real_experiments/adapted_map_3.dat')
	plot_map(data, map_num)
