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

n_maps = 10

fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
fig.set_size_inches(w=8.0, h=3.0)

mean_fit_40 = np.empty((16737,n_maps))
max_fit_40 = np.empty((16737,n_maps))

mean_fit_20 = np.empty((16737,n_maps))
max_fit_20 = np.empty((16737,n_maps))

niches_20 = np.empty((16737,n_maps))
niches_40 = np.empty((16737,n_maps))

n_evals = []


for n in range(1, n_maps+1):
	log_20 = np.loadtxt('./niches_20000/log_%d.dat' % n)
	log_40 = np.loadtxt('./niches_40000/log_%d.dat' % n)
	max_fit_20[:,n-1] = log_20[:,2] / 5
	max_fit_40[:,n-1] = log_40[:,2] / 5
	mean_fit_20[:,n-1] = log_20[:,3] / 5
	mean_fit_40[:,n-1] = log_40[:,3] / 5
	n_evals = log_20[:,0] / 1e6
	# np.append(max_fitness, max_fit, axis=1)
	niches_20[:, n-1] = log_20[:,1] / 200
	if n > 10:
		niches_40[:, n-1] = log_40[:,1] / 400
	else:
		niches_40[:, n-1] = log_40[:,1] / 400 - 13.7 # accounts for error where failed gaits were still put into a niche

max_fit_20[max_fit_20 == 0] = np.nan
max_fit_40[max_fit_40 == 0] = np.nan


# max fitness
ax1.grid(True, which='major', axis='y')
ax1.set_xlabel('Evaluations ($million$)')
ax1.set_ylabel('Gait fitness ($m/s$)')
ax1.set_xlim((0,40))
ax1.set_ylim((0,0.55))
ax1.axhline(0.5, color='tab:red', linestyle='-.', label='maximum')
ax1.text(0.7, 0.5, 'maximal', horizontalalignment='left', verticalalignment='bottom')

ax1.set_title('Maximum Fitness')
ax1.plot(n_evals, np.nanmean(max_fit_20, axis=1), label='20000', color='tab:orange')
ax1.fill_between(n_evals, np.min(max_fit_20, axis=1), np.max(max_fit_20, axis=1), alpha=0.4, color='tab:orange')

ax1.plot(n_evals, np.nanmean(max_fit_40, axis=1), label='40000', color='tab:blue')
ax1.fill_between(n_evals, np.min(max_fit_40, axis=1), np.max(max_fit_40, axis=1), alpha=0.4, color='tab:blue')

ax1.legend(loc='lower right')


# mean fitness
ax2.grid(True, which='major', axis='y')
ax2.set_xlabel('Evaluations ($million$)')
ax2.set_ylabel('Gait fitness ($m/s$)')
ax2.set_xlim((0,40))
ax2.set_ylim((0,0.55))
ax2.axhline(0.5, color='tab:red', linestyle='-.', label='maximum')
ax2.text(0.7, 0.5, 'maximal', horizontalalignment='left', verticalalignment='bottom')

ax2.set_title('Average Fitness')
ax2.plot(n_evals, np.nanmean(mean_fit_20, axis=1), label='20000', color='tab:orange')
ax2.fill_between(n_evals, np.min(mean_fit_20, axis=1), np.max(mean_fit_20, axis=1), alpha=0.2, color='tab:orange')

ax2.plot(n_evals, np.nanmean(mean_fit_40, axis=1), label='40000', color='tab:blue')
ax2.fill_between(n_evals, np.min(mean_fit_40, axis=1), np.max(mean_fit_40, axis=1), alpha=0.2, color='tab:blue')

ax2.legend(loc='lower right')


# map infill
ax3.grid(True, which='major', axis='y')
ax3.set_xlabel('Evaluations ($million$)')
ax3.set_ylabel('Coverage ($\%$)')
ax3.set_xlim((0,40))
ax3.set_ylim((0,100))

ax3.set_title('Coverage')
ax3.plot(n_evals, np.nanmean(niches_20, axis=1), label='20000', color='tab:orange')
ax3.fill_between(n_evals, np.min(niches_20, axis=1), np.max(niches_20, axis=1), alpha=0.4, color='tab:orange')

ax3.plot(n_evals, np.nanmean(niches_40, axis=1), label='40000', color='tab:blue')
ax3.fill_between(n_evals, np.min(niches_40, axis=1), np.max(niches_40, axis=1), alpha=0.4, color='tab:blue')

ax3.legend(loc='lower right')


# plt.ylim((0,0.6))


fig.tight_layout()

plt.show()
