import scipy.io as sio
from observation_model import EstimatePoses
from EKF import ExtendedKalmanFilter
import numpy as np
from tqdm import tqdm

filenames = [
        '../data/studentdata0.mat',
        '../data/studentdata1.mat',
        '../data/studentdata2.mat',
        '../data/studentdata3.mat',
        '../data/studentdata4.mat',
        '../data/studentdata5.mat',
        '../data/studentdata6.mat',
        '../data/studentdata7.mat'
]


observation_model = EstimatePoses()
# observation_model.plot_data_series('../data/studentdata1.mat')

print("Estimating Covariance Matrix in the observation model ........ ")

R = np.zeros((6, 6))

for filename in tqdm(filenames,total = len(filenames)):
     R_estimate = observation_model.estimate_covariances(filename)
     R += R_estimate

# Calculate the average R
R = R / len(filenames)

# Intializing Q process noise 
Q = np.eye(15) * 1e-6

filename = '../data/studentdata1.mat'

ekf = ExtendedKalmanFilter(filename, Q, R)
student_number = 1
filtered_positions, filtered_orientations = ekf.run_filter(student_number, observation_model.estimate_poses)
observation_model.visualize_filter_results(filename,filtered_positions)














    






