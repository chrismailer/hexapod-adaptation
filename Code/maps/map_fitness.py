import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

fig, ax = plt.subplots()
fig.set_size_inches(w=4.7747, h=3.5)

mean_fitness = np.empty((16737,10))
max_fitness = np.empty((16737,10))
n_evals = []

for n in range(1,11):
	log = np.loadtxt('./niches_40000/log_%d.dat' % n)
	n_evals = log[:,0] / 1e6
	max_fitness[:,n-1] = log[:,2] / 5
	mean_fitness[:,n-1] = log[:,3] / 5
	# np.append(max_fitness, max_fit, axis=1)
max_fitness[max_fitness == 0] = np.nan

mean_fitness_20k = np.empty((16737,1))
max_fitness_20k = np.empty((16737,1))
n_evals_20k = []

log_20k = np.loadtxt('./niches_20000/log_1.dat')
n_evals_20k = log_20k[:,0] / 1e6
max_fitness_20k[:,0] = log_20k[:,2] / 5
mean_fitness_20k[:,0] = log_20k[:,3] / 5
max_fitness_20k[max_fitness_20k == 0] = np.nan

fit_mean = np.nanmean(max_fitness, axis=1)
fit_max = np.max(max_fitness, axis=1)
minim = np.min(max_fitness, axis=1)
# maxim = np.percentile(max_fitness, 75, axis=1)
# minim = np.percentile(max_fitness, 25, axis=1)

ax.plot(n_evals, max_fitness_20k, label='20000 Max', color='tab:orange')
ax.plot(n_evals, mean_fitness_20k, label='20000 Mean', color='tab:orange', linestyle='--')

ax.plot(n_evals, np.mean(max_fitness, axis=1), label='40000 Max', color='tab:blue')
ax.fill_between(n_evals, np.min(max_fitness, axis=1), np.max(max_fitness, axis=1), alpha=0.2, color='tab:blue')

ax.plot(n_evals, np.mean(mean_fitness, axis=1), label='40000 Mean', color='tab:blue', linestyle='--')
ax.fill_between(n_evals, np.min(mean_fitness, axis=1), np.max(mean_fitness, axis=1), alpha=0.2, color='tab:blue')

ax.set_title('Map Fitness')
# ax.legend()

plt.ylabel('Gait performance ($m/s$)')
plt.xlabel('Evaluations ($million$)')
plt.ylim((0,0.6))
plt.xlim((0,40))
plt.grid(True, which='major', axis='y')
plt.legend(loc='lower right')

fig.tight_layout()

# plt.savefig('../../Final Report/figures/map_fitness_progression.pdf')

plt.show()
