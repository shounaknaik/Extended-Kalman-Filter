import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import scipy.io as sio
import matplotlib.pyplot as plt
import ipdb; 


class EstimatePoses:

    def __init__(self):
        
        #vicon format:[𝑥, 𝑦, 𝑧, 𝜙, 𝜙, 𝜓, 𝑣𝑥 , 𝑣𝑦 , 𝑣𝑧 , 𝜔𝑥 , 𝜔𝑦 , 𝜔𝑧 ]

        # Get ID tag to world coordinate map
        tag_ids = np.array([[0, 12, 24, 36, 48, 60, 72, 84, 96],
                    [1, 13, 25, 37, 49, 61, 73, 85, 97],
                    [2, 14, 26, 38, 50, 62, 74, 86, 98],
                    [3, 15, 27, 39, 51, 63, 75, 87, 99],
                    [4, 16, 28, 40, 52, 64, 76, 88, 100],
                    [5, 17, 29, 41, 53, 65, 77, 89, 101],
                    [6, 18, 30, 42, 54, 66, 78, 90, 102],
                    [7, 19, 31, 43, 55, 67, 79, 91, 103],
                    [8, 20, 32, 44, 56, 68, 80, 92, 104],
                    [9, 21, 33, 45, 57, 69, 81, 93, 105],
                    [10, 22, 34, 46, 58, 70, 82, 94, 106],
                    [11, 23, 35, 47, 59, 71, 83, 95, 107]])
        
        self.tag_size = 0.152  # meters (side length of each tag)
        standard_spacing = 0.152  # meters (spacing between tags)
        extra_spacing = 0.178  # meters (spacing between columns 3-4 and 6-7)

        self.id_wc_map = {}

        rows, columns = 12,9

        for r in range(rows):
            for c in range(columns):
                # find top left corner
                id = tag_ids[r][c]
                x = r *(self.tag_size + standard_spacing)
                y = c * (self.tag_size + standard_spacing)

                if c == 3 or c == 6:
                    y+= (extra_spacing - standard_spacing)

                z = 0

                list_of_points = [(x,y,z)]
                list_of_points.append((x+self.tag_size,y,0))
                list_of_points.append((x+self.tag_size,y+self.tag_size,0))
                list_of_points.append((x,y+self.tag_size,0))

                self.id_wc_map[id] = list_of_points

        self.intrinsic_matrix = np.array([[314.1779 , 0 , 199.4848],
                              [0 , 314.2218 , 113.7838],
                              [0, 0, 1]])
        self.distortion_coefficients = np.array([-0.438607, 0.248625, 0.00072, -0.000476, -0.0911])
        self.c2i_translation = np.array([-0.04, 0.0, -0.03])
    


    def estimate_poses(self,data):
        """
        Uses PnP to estimate poses
        Here data is data from one timestamp
        """

        p1 = data['p4']
        p2 = data['p1']
        p3 = data['p2']
        p4 = data['p3']

        tag_ids = data['id']

        # There are some plain ints in data and the p arrays are a list. 
        # The 𝑖th column in each of these fields corresponds to the 𝑖th tag in the ID field. The
        # values are expressed in image (pixel) coordinates.
        if isinstance(tag_ids, int):
            tag_ids=np.array([tag_ids])
            p1=p1.reshape(2,1)
            p2=p2.reshape(2,1)
            p3=p3.reshape(2,1)
            p4=p4.reshape(2,1)

        if len(tag_ids) == 0:
            pos = np.nan
            ori = np.nan
            timestamp = np.nan
            return pos , ori , timestamp

        image_points = []
        num_image_points = p1.shape[1] # column wise arrangement
        for i in range(num_image_points):
            image_points.append((p1[0, i], p1[1, i]))
            image_points.append((p2[0, i], p2[1, i]))
            image_points.append((p3[0, i], p3[1, i]))
            image_points.append((p4[0, i], p4[1, i]))
        
        assert num_image_points == len(tag_ids), "3D - 2D correspondences not equal"

        all_world_points = []
        for tag in tag_ids:
            world_points_li = self.id_wc_map[tag]
            all_world_points.extend(world_points_li)
        


        ## Solve PnP ##
        # Gets us camera to world transformation
        bool_val, rvec, tvec = cv2.solvePnP(np.array(all_world_points), np.array(image_points), self.intrinsic_matrix, self.distortion_coefficients)
        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        T_c2w = np.hstack([R,tvec.reshape(3,1)])
        T_c2w = np.vstack((T_c2w,[0,0,0,1]))
        T_w2c = np.linalg.inv(T_c2w)

        # Figuring out the rotation between Camera and IMU. 
        z_rot = np.array([[np.cos(np.pi / 4), -np.sin(np.pi / 4), 0], 
                               [np.sin(np.pi / 4), np.cos(np.pi / 4), 0],
                               [0, 0, 1],])
        
        x_rot = np.array([[1, 0, 0],
                               [0, -1, 0],
                               [0, 0, -1],])

        rotation_c2i = x_rot@z_rot

        T_c2i = np.hstack([rotation_c2i,self.c2i_translation.reshape(3,1)])
        T_c2i = np.vstack((T_c2i,[0,0,0,1]))

        #Finding the transformation from imu to world.
        # Vicon gives world to imu poses
        T_w2i = T_w2c @ T_c2i

        estimated_position = T_w2i[:3, 3]
    
        roll, pitch, yaw = Rotation.from_matrix(T_w2i[:3, :3]).as_euler('xyz')

        estimated_orientation = np.array([roll,pitch,yaw])
        timestamp = data['t']
        

        return estimated_orientation, estimated_position, timestamp

    def plot_data_series(self,filename):

        mat_contents = sio.loadmat(filename,simplify_cells = True )

        ## Get Ground truth ##
        #Data needs to be transposed from (12,n) to (n,12)
        gt_vicon = mat_contents['vicon'].T

        ## Get Estimates
        estimated_positions = []
        estimated_orientations = []
        # Looping over each timestep and estimating pose
        for data in mat_contents["data"]:
            estimated_orientation, estimated_position, timestamp = self.estimate_poses(data)
            if not np.isnan(estimated_orientation).any() and not np.isnan(estimated_position).any():
                estimated_orientations.append(estimated_orientation) 
                estimated_positions.append(estimated_position)

        estimated_positions = np.array(estimated_positions)
        estimated_orientations = np.array(estimated_orientations)

        gt_positions = gt_vicon[:,:3]

        # Plot trajectory
        # # Plot 3D trajectory
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(estimated_positions[:, 0], estimated_positions[:, 1], estimated_positions[:, 2], label='Estimated', color='blue')
        ax.scatter(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], label='Ground Truth', color='green')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.set_title(f"Trajectory Plot: Ground Truth vs Estimates")
        plt.tight_layout()
        

        plt.show()



    def visualize_filter_results(self,mat_file,filtered_positions):
        # Load .mat file
        mat_data = sio.loadmat(mat_file, simplify_cells=True)

        # Extract motion capture data
        vicon = mat_data['vicon']
        ground_truth_timestamps = mat_data['time']
        vicon_data = np.array(vicon).T

        # Extract ground truth positions
        ground_truth_position = vicon_data[:, :3]
        ground_truth_position = np.array(ground_truth_position)

        # Extract observation model data
        estimated_positions = []

        # Assuming estimate_pose is a function that returns position, orientation, and timestamp
        for data_sample in mat_data['data']:
            orientation, position, timestamp = self.estimate_poses(data_sample)
            if not np.isnan(position).any() and not np.isnan(orientation).any() and not np.isnan(timestamp).any():
                estimated_positions.append(position)
                # estimated_orientations.append(orientation)

        # Convert lists to numpy arrays
        estimated_positions = np.array(estimated_positions)
        filtered_positions = np.array(filtered_positions)

        # Plot ground truth, estimated, and filtered positions
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot ground truth position
        ax.scatter(ground_truth_position[:, 0], ground_truth_position[:, 1], ground_truth_position[:, 2], label='Ground Truth', color='blue')

        # Plot estimated position
        ax.scatter(estimated_positions[:, 0], estimated_positions[:, 1], estimated_positions[:, 2], label='Observation Model Result', color='green')

        # Plot filtered position
        ax.scatter(filtered_positions[:, 0], filtered_positions[:, 1], filtered_positions[:, 2], label='EKF Filtered', color='red')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Ground Truth vs Estimated vs Filtered Position ({mat_file})')
        ax.legend()
        plt.show()



    def interpolate_ground_truth_state(self,mat_contents,drone_timestamp):

        data_gt = mat_contents['vicon'].T
        time_gt = mat_contents['time']

        # Find the first ground truth time that is past that timestamp
        data_point1 = None
        data_point2 = None
        for idx in range(1, len(time_gt)):  # Start at index 1 to safely access index - 1
            if time_gt[idx] > drone_timestamp:
                data_point1 = data_gt[idx - 1]
                data_point2 = data_gt[idx]
                timestamp_point1 = time_gt[idx -1]
                timestamp_point2 = time_gt[idx]
                break

        if data_point1 is None or data_point2 is None:
            return np.array([data_gt[idx -1][:6]]).reshape(6, 1)

        total_change = timestamp_point2 - timestamp_point1

        weight1 = (drone_timestamp - timestamp_point1) / total_change
        weight2 = (timestamp_point2 - drone_timestamp) / total_change

        # Finally taking weighted average between data_point1 and data_point2 given our percentages as weights
        data_point1_vec = np.array([data_point1[:6]]).reshape(6, 1)
        data_point2_vec = np.array([data_point2[:6]]).reshape(6, 1)

        # Create a new ground truth object with the interpolated state vector.
        # The more away from the gt_timestamp, the lesser the weight that is why 1-weight
        interpolated_state = (1 - weight1) * data_point1_vec
        interpolated_state += (1 - weight2) * data_point2_vec

        
        return interpolated_state

    def estimate_covariances(self,filename):

        mat_contents = sio.loadmat(filename,simplify_cells = True )

        ## Get Ground truth ##
        #Data needs to be transposed from (12,n) to (n,12)
        gt_vicon = mat_contents['vicon'].T

        ## Get Estimates
        estimated_positions = []
        estimated_orientations = []
        drone_timestamps = []
        # Looping over each timestep and estimating pose
        for data in mat_contents["data"]:
            estimated_orientation, estimated_position, timestamp = self.estimate_poses(data)
            if not np.isnan(estimated_orientation).any() and not np.isnan(estimated_position).any():
                estimated_orientations.append(estimated_orientation) 
                estimated_positions.append(estimated_position)
                drone_timestamps.append(timestamp)

        estimated_positions = np.array(estimated_positions)
        estimated_orientations = np.array(estimated_orientations)
        drone_timestamps = np.array(drone_timestamps)

        # gt_positions = gt_vicon[:,:3]

        interpolated_gt_states = []
        #Aligning timestamps
        for drone_timestamp in drone_timestamps:
            interpolated_ground_truth_state = self.interpolate_ground_truth_state(mat_contents,drone_timestamp)
            interpolated_gt_states.append(interpolated_ground_truth_state)
        interpolated_gt_states = np.array(interpolated_gt_states)

        estimated_states = np.hstack((estimated_positions,estimated_orientations))

        observation_noise = np.squeeze(interpolated_gt_states) - estimated_states
        R = (observation_noise.T @ observation_noise )/ (len(mat_contents["data"]) -1 )

        return R







        









        


        

        

        

        


            
