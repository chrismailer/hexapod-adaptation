import numpy as np

def load_map(filename, dim=6, dim_ctrl=32):
    # print("Loading ",filename)
    data = np.loadtxt(filename)
    fit = data[:, 0] / 5.0
    desc = data[:, 1:dim+1]
    x = data[:, 2*dim+1:]
    return fit, desc, x

centroids_10 = np.loadtxt('../../centroids/centroids_10000_6.dat')
centroids_20 = np.loadtxt('../../centroids/centroids_20000_6.dat')
centroids_40 = np.loadtxt('../../centroids/centroids_40000_6.dat')
centroids_60 = np.loadtxt('../../centroids/centroids_60000_6.dat')

n_maps = 10
niches = ['10k', '20k', '40k', '60k']

global_performance = np.zeros((n_maps, len(niches)))
global_reliability = np.zeros((n_maps, len(niches)))
precision = np.zeros((n_maps, len(niches)))
coverage = np.zeros((n_maps, len(niches)))

cells_10 = np.full((centroids_10.shape[0], n_maps), np.NaN)
cells_20 = np.full((centroids_20.shape[0], n_maps), np.NaN)
cells_40 = np.full((centroids_40.shape[0], n_maps), np.NaN)
cells_60 = np.full((centroids_60.shape[0], n_maps), np.NaN)

print('This can take a while', end='')

for col, niche_size in enumerate(niches):
    for map_n in range(1, n_maps+1):
        fits, descs, x = load_map(f'./{niche_size}/map_{map_n}.dat')
        global_performance[map_n-1, col] = max(fits) / 0.5
        coverage[map_n-1, col] = fits[fits > 0].shape[0] / int(niche_size[:2])
        print('.', end='')
        for fit, desc in zip(fits, descs):
            if col == 0:
                row_index = np.where((centroids_10 == desc).all(axis=1))
                cells_10[row_index, map_n-1] = fit
            elif col == 1:
                row_index = np.where((centroids_20 == desc).all(axis=1))
                cells_20[row_index, map_n-1] = fit
            elif col == 2:
                row_index = np.where((centroids_40 == desc).all(axis=1))
                cells_40[row_index, map_n-1] = fit
            elif col == 3:
                row_index = np.where((centroids_60 == desc).all(axis=1))
                cells_60[row_index, map_n-1] = fit

# remove rows which are only NaN
# ignore niches which couldn't be filled
cells_10 = cells_10[~np.isnan(cells_10).all(axis=1)]
cells_20 = cells_20[~np.isnan(cells_20).all(axis=1)]
cells_40 = cells_40[~np.isnan(cells_40).all(axis=1)]
cells_60 = cells_60[~np.isnan(cells_60).all(axis=1)]

cells_10 = cells_10[(cells_10 > 0).any(axis=1)]
cells_20 = cells_20[(cells_20 > 0).any(axis=1)]
cells_40 = cells_40[(cells_40 > 0).any(axis=1)]
cells_60 = cells_60[(cells_60 > 0).any(axis=1)]

M_10 = np.nanmax(cells_10, axis=1)[:, np.newaxis]
M_20 = np.nanmax(cells_20, axis=1)[:, np.newaxis]
M_40 = np.nanmax(cells_40, axis=1)[:, np.newaxis]
M_60 = np.nanmax(cells_60, axis=1)[:, np.newaxis]

global_reliability[:,0] = np.nansum(np.divide(np.nan_to_num(cells_10), M_10), axis=0) / M_10.shape[0] # includes niches which weren't filled
global_reliability[:,1] = np.nansum(np.divide(np.nan_to_num(cells_20), M_20), axis=0) / M_20.shape[0]
global_reliability[:,2] = np.nansum(np.divide(np.nan_to_num(cells_40), M_40), axis=0) / M_40.shape[0] # includes niches which weren't filled
global_reliability[:,3] = np.nansum(np.divide(np.nan_to_num(cells_60), M_60), axis=0) / M_60.shape[0]

precision[:,0] = np.nanmean(np.divide(cells_10, M_10), axis=0)
precision[:,1] = np.nanmean(np.divide(cells_20, M_20), axis=0)
precision[:,2] = np.nanmean(np.divide(cells_40, M_40), axis=0)
precision[:,3] = np.nanmean(np.divide(cells_60, M_60), axis=0)

# save data to file
header = ','.join([str(elem) for elem in niches])
np.savetxt('global_performance.csv', global_performance, delimiter=',', header=header)
np.savetxt('global_reliability.csv', global_reliability, delimiter=',', header=header)
np.savetxt('precision.csv', precision, delimiter=',', header=header)
np.savetxt('coverage.csv', coverage, delimiter=',', header=header)
