# Visual Inertial Odometry 
Visual Inertial Odometry (VIO) is a method for estimating a device's pose by fusing data from a camera and an inertial measurement unit (IMU). By combining visual features from the camera with IMU data, VIO provides accurate motion tracking and position estimation, even in environments with poor GPS signals.  
This repository implements a nonlinear Kalman Filter, specifically the Extended Kalman Filter (EKF), to estimate the pose of a quadcopter drone.   
Kalman Filter assumes linearity in the process and the observation models. But this assumption rarely works in the real world. Extended Kalman Filter allows non-linear functions by linearizing the functions at each timestep by using Taylor expansion.  
 Details and an implementation of the Kalman Filter can be found [here](https://github.com/shounaknaik/KalmanFilter)  

### We use the IMU acceleration readings in the process model and then update the estimates by the pose estimation from the Camera. 

## Pose Estimation Using PnP
To estimate the pose of the quadcopter drone, we will apply the Perspective-n-Point (PnP) problem-solving approach. This technique involves determining the position and orientation of the drone by relating known 3D points in the environment to their corresponding 2D projections in the camera's image plane. Specifically, we utilize AprilTag markers placed in the environment as 3D reference points. These AprilTag corners provide fixed, recognizable locations in the world, and their projections onto the 2D image plane, captured by the onboard camera, allow us to compute the drone's pose relative to the environment.

By solving the PnP problem, we can efficiently estimate the drone's position and orientation using visual data, which can then be combined with other sensor measurements (e.g., from an IMU) to improve the overall accuracy of the pose estimation. 

### Camera Calibration Matrix and Distortion Coefficients

The camera calibration matrix and distortion coefficients are provided as follows:

**Camera Matrix**:  
```
[314.1779  0  199.4848  
0  314.2218  113.7838 
0  0  1 ]
```

**Distortion Coefficients**:  

```
[-0.438607 & 0.248625 & 0.00072 & -0.000476 & -0.0911 ]
```

### Camera Model

The camera model establishes the relationship between the homogeneous world coordinates and the homogeneous image coordinates. It is represented as:

<!-- $$
\begin{bmatrix} 
x \\ 
y \\ 
z \\ 
1 
\end{bmatrix}
=
\begin{bmatrix} 
fx & \gamma & u0 \\ 
0 & fy & v0 \\ 
0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix} 
r11 & r12 & r13 & t1 \\ 
r21 & r22 & r23 & t2 \\ 
r31 & r32 & r33 & t3 
\end{bmatrix}
\begin{bmatrix} 
X \\ 
Y \\ 
Z \\ 
1 
\end{bmatrix}
$$ -->
```
[x, y, z]^T = [ fx γ   u0    [r12 r12  r13 t1     [X                             
                0  fy  vo  * r21 r22 r23 t2     * Y
                0   0  1]    r31 r32 r33 t3 ]     Z
                                                  1]
```
Where:

- `fx` and `fy` are the focal lengths in the x and y directions, respectively.
- `u0` and `v0` are the coordinates of the principal point.
- `γ` is the skew coefficient (typically zero for most cameras).
- `r11, r12, ..., r33` are the components of the rotation matrix, and `t1, t2, t3` are the translation parameters that define the pose of the camera with respect to the world.  

After obtaining the orientation data in the form of a rotation vector, we will convert this vector into a rotation matrix. Using the rotation matrix, we can extract the three Euler angles.  This allows us to compute the transformation matrix from the camera frame to the world frame.

Additionally, the given information—specifically, a yaw rotation of 
π/4 and a roll rotation of 
π is used to determine the transformation from the camera frame to the IMU frame.

## Time Alignment
The time
stamps have been aligned as the rate of data
collection for the IMU and the Vicon system is different. The function performs linear interpolation to estimate the ground truth state of a drone at a given timestamp, drone_timestamp.   

$weight1 = \frac{drone\_timestamp - timestamp1}{timestamp2-timestamp1}$
$weight2 = \frac{timestamp2 - drone\_timestamp}{timestamp2-timestamp1}$

$interpolated\_\text{state}=weight1⋅datapoint2+weight2⋅datapoint1$

## Covariance Estimation
