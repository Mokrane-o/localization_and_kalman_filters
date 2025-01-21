# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Pose
# import numpy as np
# from filterpy.kalman import KalmanFilter
# from filterpy.common import Q_discrete_white_noise

# def pos_vel_filter(x, P, R, Q=0., dt=1.0):
#     """Returns a KalmanFilter that implements a constant velocity model for a state [x, dx]."""
    
#     kf = KalmanFilter(dim_x=2, dim_z=1)  # State: [x, dx], Measurement: [x]
    
#     # Initial state [x, dx]
#     kf.x = np.array([x[0], x[1]])
    
#     # State transition matrix (constant velocity model)
#     kf.F = np.array([
#         [1., dt],
#         [0., 1.]
#     ])
    
#     # Measurement function (we can only measure position)
#     kf.H = np.array([[1., 0.]])
    
#     # Initial uncertainty in state (position and velocity)
#     if np.isscalar(P):
#         kf.P *= P
#     else:
#         kf.P[:] = P
    
#     # Measurement noise covariance
#     kf.R = R
    
#     # Process noise covariance
#     if np.isscalar(Q):
#         kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q)
#     else:
#         kf.Q[:] = Q
    
#     return kf

# class LocalizationNode(Node):
#     def __init__(self):
#         super().__init__('robot_position_filtered')
        
#         # Subscribers
#         self.sub1 = self.create_subscription(Float64, '/d1_with_noise', self.callback_d1, 10)
#         self.sub2 = self.create_subscription(Float64, '/d2_with_noise', self.callback_d2, 10)
#         self.sub3 = self.create_subscription(Float64, '/d3_with_noise', self.callback_d3, 10)
#         self.sub4 = self.create_subscription(Float64, '/d4_with_noise', self.callback_d4, 10)
#         self.sub5 = self.create_subscription(Float64, '/d5_with_noise', self.callback_d5, 10)
#         self.sub6 = self.create_subscription(Float64, '/d6_with_noise', self.callback_d6, 10)

#         # Publishers
#         self.pub1 = self.create_publisher(Pose, '/Estimated_Position_Filtered', 10)

#         # Distances
#         self.d1 = 0.0
#         self.d2 = 0.0
#         self.d3 = 0.0
#         self.d4 = 0.0
#         self.d5 = 0.0
#         self.d6 = 0.0

#         # Anchor positions (retrieved from parameters)
#         self.anchor_positions = []
#         for i in range(6):
#             self.declare_parameter(f'pose_{i+1}_x', 0.0)
#             self.declare_parameter(f'pose_{i+1}_y', 0.0)
#             self.declare_parameter(f'pose_{i+1}_z', 0.0)
#             self.anchor_positions.append(np.array([
#                 self.get_parameter(f'pose_{i+1}_x').value,
#                 self.get_parameter(f'pose_{i+1}_y').value,
#                 self.get_parameter(f'pose_{i+1}_z').value
#             ]))

#         self.declare_parameter('noise_variance', 0.0)
#         self.declare_parameter('timing', 0.0) 
#         self.noise_variance = self.get_parameter('noise_variance').value
#         self.timing = self.get_parameter('timing').value

#         # Kalman Filter Initialization
#         self.dt = self.timing  # Time step for the filter
#         x = np.array([0.0, 0.0])  # Initial state [x, dx]
#         P = np.diag([1.0, 1.0])  # Initial uncertainty
#         R = self.noise_variance  # Measurement noise covariance
#         Q = 0.01  # Process noise variance
#         self.kf = pos_vel_filter(x, P, R, Q=Q, dt=self.dt)

#     def callback_d1(self, dis1):
#         self.d1 = dis1.data

#     def callback_d2(self, dis2):
#         self.d2 = dis2.data

#     def callback_d3(self, dis3):
#         self.d3 = dis3.data

#     def callback_d4(self, dis4):
#         self.d4 = dis4.data

#     def callback_d5(self, dis5):
#         self.d5 = dis5.data

#     def callback_d6(self, dis6):
#         self.d6 = dis6.data

#     def distances(self):
#         return self.d1, self.d2, self.d3, self.d4, self.d5, self.d6

#     def Estimated_position_ft(self):
#         """Estimate the position using the distances and apply Kalman Filter."""
#         # Get distances
#         dis1, dis2, dis3, dis4, dis5, dis6 = self.distances()
#         distances = np.array([float(dis1), float(dis2), float(dis3), float(dis4), float(dis5), float(dis6)]).reshape(6, 1)
#         anchors = np.vstack(self.anchor_positions)

#         # Solve for position using least squares (only x dimension)
#         num_eq = 5
#         A = np.zeros((num_eq, 1))
#         b = np.zeros((num_eq, 1))

#         for i in range(num_eq):
#             A[i, :] = 2 * (anchors[i + 1, 0] - anchors[i, 0])
#             b[i] = distances[i]**2 - distances[i + 1]**2 - anchors[i, 0]**2 + anchors[i + 1, 0]**2

#         pinv_A = np.linalg.pinv(A)
#         result = np.dot(pinv_A, b).flatten()

#         # Kalman Filter Update
#         self.kf.predict()  # Prediction step
#         self.kf.update(result[0])  # Update step with the new measurement (only x)

#         # Get the filtered state
#         filtered_state = self.kf.x

#         # Publish the filtered position
#         rpose = Pose()
#         rpose.position.x = filtered_state[0]
#         rpose.position.y = 0.0  # No y component
#         rpose.position.z = 0.0  # No z component
#         rpose.orientation.x = 0.0
#         rpose.orientation.y = 0.0
#         rpose.orientation.z = 0.0
#         rpose.orientation.w = 1.0

#         self.pub1.publish(rpose)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LocalizationNode()

#     try:
#         while rclpy.ok():
#             node.Estimated_position_ft()
#             rclpy.spin_once(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()