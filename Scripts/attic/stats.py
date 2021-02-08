# used to compute statistics for 
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt


# load experiment data
scenario1 = np.loadtxt('./experiments/adapt_perf_1.dat').flatten() / 5
scenario2 = np.loadtxt('./experiments/adapt_perf_2_2.dat').flatten() / 5
scenario3 = np.loadtxt('./experiments/adapt_perf_2_1.dat').flatten() / 5
scenario4 = np.loadtxt('./experiments/adapt_perf_2_0.dat').flatten() / 5

normal_tripod_mean = np.mean(np.loadtxt('./experiments/tripod_no_failure.dat')) / 5
failed_tripod = np.loadtxt('./experiments/tripod_failure_1.dat') / 5

# plt.hist(scenario4)
# plt.show()

# t_statistic, p_value = stats.ttest_ind(scenario1, scenario2, equal_var=False)
t_statistic, p_value = stats.ttest_1samp(scenario4, np.mean(failed_tripod))
# t_statistic, p_value = stats.ttest_ind(scenario1, failed_tripod, equal_var=False)

print(p_value * 100)