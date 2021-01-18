import matplotlib.pyplot as plt
import matplotlib
import numpy as np

# matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

sim_data_20 = []
sim_data_40 = []

for scenario in range(1,4):
	data_20 = list(np.loadtxt(f'./20000_niches/trials_{scenario}.dat').flatten())
	data_40 = list(np.loadtxt(f'./40000_niches/trials_{scenario}.dat').flatten())
	sim_data_20.append(data_20)
	sim_data_40.append(data_40)


ticks = ['Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4']

def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)

fig, ax = plt.subplots()
fig.set_size_inches(w=4.7747, h=3.5)
ax.yaxis.grid(True)
ax.set_title('Number of Adaptation Trials')

bpl = plt.boxplot(sim_data_40, positions=np.array(range(len(sim_data_40)))*2.0-0.4, widths=0.4, showfliers=True, notch=True, showmeans=True, patch_artist=True)
bpr = plt.boxplot(sim_data_20, positions=np.array(range(len(sim_data_20)))*2.0+0.4, widths=0.4, showfliers=True, notch=True, showmeans=True, patch_artist=True)
set_box_color(bpl, 'tab:blue')
set_box_color(bpr, 'tab:orange')

# draw temporary red and blue lines and use them to create a legend
plt.plot([], c='tab:blue', label='40000')
plt.plot([], c='tab:orange', label='20000')
plt.legend()

plt.xticks(range(0, len(ticks) * 2, 2), ticks)
plt.xlim(-2, len(ticks)*2)
plt.ylim(0, 40)
plt.tight_layout()

plt.show()