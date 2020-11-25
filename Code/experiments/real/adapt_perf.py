import numpy as np
from scipy import stats
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

# load simulated results
# scenario1 = np.loadtxt('../experiments/adapt_perf_1.dat').flatten() / 5
# scenario2 = np.loadtxt('../experiments/adapt_perf_2_2.dat').flatten() / 5
# scenario3 = np.loadtxt('../experiments/adapt_perf_2_1.dat').flatten() / 5
# scenario4 = np.loadtxt('../experiments/adapt_perf_2_0.dat').flatten() / 5

# load control results
control1 = np.loadtxt('control_experiment.dat') / 5
control2 = np.loadtxt('../experiments/tripod_failure_1.dat').flatten()
control3 = np.loadtxt('../experiments/tripod_failure_2_2.dat').flatten()
control = np.array([control1, control2, control3])

# load real experiment data
real1 = np.max(np.loadtxt('experiment_1.dat')[:,1]) / 5
real2 = np.max(np.loadtxt('experiment_2.dat')[:,1]) / 5
real3 = np.max(np.loadtxt('experiment_3.dat')[:,1]) / 5
real4 = np.max(np.loadtxt('experiment_4.dat')[:,1]) / 5
real5 = np.max(np.loadtxt('experiment_5.dat')[:,1]) / 5
real6 = np.max(np.loadtxt('experiment_6.dat')[:,1]) / 5
real = np.array([[1,real1], [1,real2], [2,real3], [2,real4], [3,real5], [3,real6]])

normal_tripod_mean = np.mean(np.loadtxt('../experiments/tripod_no_failure.dat')) / 5

t_statistic, p_value = stats.ttest_ind(real.flatten(), np.hstack(control))
print(p_value)

# plotting graph
fig, ax = plt.subplots()
fig.set_size_inches(w=4.7747, h=3.5)
ax.yaxis.grid(True)
ax.set_axisbelow(True)
ax.set_title('Adapted Performance')

# ax.axhline(failed_tripod_mean, color='tab:red', linestyle='--', label='Failed Tripod Gait')
# ax.axhspan(failed_tripod_max, failed_tripod_min, color='tab:red', alpha=0.25)

bplot2 = ax.boxplot(control, notch=True, showfliers=True, labels=['None', 'Scenario 1', 'Scenario 2'], widths=0.2, showmeans=True, patch_artist=True)

# ax.plot(data[:,0], data[:,1], marker='o', linestyle='', color='tab:red')

scatter = ax.scatter(real[:,0], real[:,1], marker='x')

for i, point in enumerate(real):
	if i+1 == 3:
		plt.text(point[0]+0.05, point[1], str(i+1), verticalalignment='bottom')
	else:
		plt.text(point[0]+0.05, point[1], str(i+1), verticalalignment='top')

for patch in bplot2['boxes']:
	patch.set_facecolor('tab:orange')

plt.ylabel('Performance ($m/s$)')
plt.xlabel('Failure')
# plt.xticks(np.arange(3), ['None', 'Scenario 1', 'Scenario 2'])
plt.ylim(0, 0.5)
plt.legend((scatter, bplot2["boxes"][0]), ('Adaptation', 'Tripod'), loc='upper right')

fig.tight_layout()

plt.savefig('../../Final Report/figures/adapted_perf_real.pdf')

# plt.show()
