import numpy as np
import scipy.io
import sympy as sp
from tqdm import tqdm

from observation_model import EstimatePoses

class ExtendedKalmanFilter:
    def __init__(self, mat_file, Q, R):
        self.mat_file = mat_file
        self.Q = Q
        self.R = R
        self.filtered_positions = []
        self.filtered_orientations = []

    def symbolic_jacobian_process_model(self):
        """
        Symbolic computation of the Jacobian of the process model.
        """
        p1, p2, p3, phi, theta, psi, p1_dot, p2_dot, p3_dot, bg1, bg2, bg3, ba1, ba2, ba3, dt = sp.symbols(
            'p1 p2 p3 phi theta psi p1_dot p2_dot p3_dot bg1 bg2 bg3 ba1 ba2 ba3 dt')

        # Define the matrix elements
        G_q = sp.Matrix([
            [sp.cos(theta), 0, -sp.cos(phi) * sp.sin(theta)],
            [0, 1, sp.sin(phi)],
            [sp.sin(theta), 0, sp.cos(phi) * sp.cos(theta)]
        ])

        # Compute the inverse of G_q
        G_q_inv = G_q.inv()

        # Write R_q as a 3x3 matrix just like G_q
        R_q = sp.Matrix([
            [sp.cos(psi) * sp.cos(theta) - sp.sin(phi) * sp.sin(theta) * sp.sin(psi), -sp.cos(phi) * sp.sin(psi),
            sp.cos(psi) * sp.sin(theta) + sp.cos(theta) * sp.sin(phi) * sp.sin(psi)],
            [sp.cos(psi) * sp.sin(phi) * sp.sin(theta) + sp.cos(theta) * sp.sin(psi), sp.cos(phi) * sp.cos(psi),
            sp.sin(psi) * sp.sin(theta) - sp.cos(psi) * sp.cos(theta) * sp.sin(phi)],
            [-sp.cos(phi) * sp.sin(theta), sp.sin(phi), sp.cos(phi) * sp.cos(theta)]
        ])

        # Define the state vector x = [p, q, p_dot, bg, ba]
        x = sp.Matrix([p1, p2, p3, phi, theta, psi, p1_dot, p2_dot, p3_dot, bg1, bg2, bg3, ba1, ba2, ba3])

        # Create a new matrix including only p_dot1, p_dot2, p_dot3
        p_dot = sp.Matrix([p1_dot, p2_dot, p3_dot])

        # Define the input vector
        wx, wy, wz, vx, vy, vz = sp.symbols('wx wy wz vx vy vz')
        uw = sp.Matrix([wx, wy, wz])
        ua = sp.Matrix([vx, vy, vz])

        # Define the gravity vector and the bias noises
        g = sp.Matrix([0, 0, -9.81])
        nbg = sp.Matrix([0, 0, 0])
        nba = sp.Matrix([0, 0, 0])

        # Define the x_dot equation x_dot
        x_dot = sp.Matrix([p_dot, G_q_inv * uw, g + R_q * ua, nbg, nba])

        # Compute the Jacobian of x_dot with respect to x to obtain A
        A = x_dot.jacobian(x)

        # Construct the F matrix
        F = sp.eye(15) + A * dt

        # Propogating the state
        x_hat = x + x_dot * dt

        return F, x_hat

    def process_model_step(self, x, dtime, u):
        """
        Implementation of the process model step.
        """
        p1, p2, p3, phi, theta, psi, p1_dot, p2_dot, p3_dot, bg1, bg2, bg3, ba1, ba2, ba3, dt = sp.symbols(
            'p1 p2 p3 phi theta psi p1_dot p2_dot p3_dot bg1 bg2 bg3 ba1 ba2 ba3 dt')
        wx, wy, wz, vx, vy, vz = sp.symbols('wx wy wz vx vy vz')
        F_sym, x_hat_sym = self.symbolic_jacobian_process_model()
        #substituting the vlaue of u and x and dt in the symbolic equation
        F = F_sym.subs(
            {p1: x[0, 0], p2: x[1, 0], p3: x[2, 0], phi: x[3, 0], theta: x[4, 0], psi: x[5, 0], p1_dot: x[6, 0],
            p2_dot: x[7, 0], p3_dot: x[8, 0], bg1: x[9, 0], bg2: x[10, 0], bg3: x[11, 0], ba1: x[12, 0],
            ba2: x[13, 0], ba3: x[14, 0], wx: u[0, 0], wy: u[1, 0], wz: u[2, 0], vx: u[3, 0], vy: u[4, 0],
            vz: u[5, 0], dt: dtime})
        x_hat = x_hat_sym.subs(
            {p1: x[0, 0], p2: x[1, 0], p3: x[2, 0], phi: x[3, 0], theta: x[4, 0], psi: x[5, 0], p1_dot: x[6, 0],
            p2_dot: x[7, 0], p3_dot: x[8, 0], bg1: x[9, 0], bg2: x[10, 0], bg3: x[11, 0], ba1: x[12, 0],
            ba2: x[13, 0], ba3: x[14, 0], wx: u[0, 0], wy: u[1, 0], wz: u[2, 0], vx: u[3, 0], vy: u[4, 0],
            vz: u[5, 0], dt: dtime})
        F = np.array(F).reshape(15, 15)
        x_hat = np.array(x_hat).reshape(15, 1)

        return F, x_hat
    
    def jacobian_measurement_mode(self):
        """
        Compute the Jacobian of the measurement model.
        :return: H matrix
        """
        H = np.zeros((6, 15))

        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)

        return H


    def predict(self, x, P, u, dt):
        """
        Predict the state and covariance forward in time.
        :param u: control input (IMU readings)
        :param dt: time step
        :param P: earlier covariance estimations
        """
        # Update state with process model
        F, x_hat = self.process_model_step(x, dt, u)

        # Predict covariance
        P = F @ P @ F.T + self.Q

        return x_hat, P
    
    def update(self, x_hat, data, P,estimate_pose):
        """
        Update the state estimate using observed measurements.
        :param z: measurement vector
        :param estimate_pose: function handle for estimate poses from observation model.
        """
        # Compute measurement matrix H
        H = self.jacobian_measurement_mode()
        P = P.astype(np.float64)
        H = H.astype(np.float64)
        R = self.R.astype(np.float64)

        # Compute Kalman gain
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)

        # Update state estimate
        orientation, position, timestamp = estimate_pose(data)
        z = np.concatenate((position.reshape(-1, 1), orientation.reshape(-1, 1)))
        y = z - H @ x_hat

        x_hat += K @ y

        P = (np.eye(15) - K @ H) @ P

        return x_hat, P
    
    def run_filter(self, student_number,estimate_pose):
        """
        Estimate pose at each time stamp.
        """
        i = 0
        mat_data = scipy.io.loadmat(self.mat_file, simplify_cells=True)
        # Extract motion capture data
        vicon = mat_data['vicon']
        ground_truth_timestamps = mat_data['time']
        vicon_data = np.array(vicon).T

        # Extract ground truth positions and orientations
        # This is to get an initial estimate of the drone. 
        ground_truth_position = vicon_data[0, :3]
        ground_truth_orientation = vicon_data[0, 3:6]
        ground_truth_velocity = vicon_data[0, 6:9]

        x_hat = np.hstack((ground_truth_position, ground_truth_orientation, ground_truth_velocity, [0, 0, 0],
                            [0, 0, 0])).reshape(15, 1)

        #Defining the P matrix as diagonal matrix with covariance value as 0.015. Initial Process Noise Value
        P = np.eye(15) * 0.01

        print("Doing Iteration for:", self.mat_file)

        for data_sample in tqdm(mat_data['data'],total=len(mat_data['data'])):
            # End condition for the for loop
            if i == len(mat_data['data'])-1:
                break
            # Calculating the dt
            dt = mat_data['data'][i + 1]['t'] - mat_data['data'][i]['t']

            # The data for the student 0 is different than others contering that here
            if student_number==0:
                omega = np.array(data_sample['drpy']).reshape(-1, 1)

            else :
                omega = np.array(data_sample['omg']).reshape(-1, 1)

            acc = np.array(data_sample['acc']).reshape(-1, 1)

            # Creating a u verctor(6,1)
            u = np.concatenate((omega, acc))

            tag_ids = data_sample['id']

            if isinstance(tag_ids, int):
                tag_ids = np.array([tag_ids])
            if len(tag_ids) == 0:
                x_hat, P = self.predict(x_hat, P, u, dt)
            else:
                x_hat, P = self.predict(x_hat, P, u, dt)
                x_hat, P = self.update(x_hat, data_sample, P,estimate_pose)

            self.filtered_positions.append([x_hat[0, 0], x_hat[1, 0], x_hat[2, 0]])
            self.filtered_orientations.append([x_hat[3, 0], x_hat[4, 0], x_hat[5, 0]])

            i += 1

        return self.filtered_positions, self.filtered_orientations
    

  

