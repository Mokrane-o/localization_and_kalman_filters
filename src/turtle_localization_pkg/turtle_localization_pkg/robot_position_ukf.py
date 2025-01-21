#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_position_ukf')
        
        # Subscribers
        self.sub1 = self.create_subscription(Float64, '/d1_with_noise', self.callback_d1, 10)
        self.sub2 = self.create_subscription(Float64, '/d2_with_noise', self.callback_d2, 10)
        self.sub3 = self.create_subscription(Float64, '/d3_with_noise', self.callback_d3, 10)
        self.sub4 = self.create_subscription(Float64, '/d4_with_noise', self.callback_d4, 10)
        self.sub5 = self.create_subscription(Float64, '/d5_with_noise', self.callback_d5, 10)
        self.sub6 = self.create_subscription(Float64, '/d6_with_noise', self.callback_d6, 10)

        # Publishers
        self.pub1 = self.create_publisher(Pose, '/Estimated_Position_ukf', 10)

        # Distances
        self.d1 = 0.0
        self.d2 = 0.0
        self.d3 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0

        # # Anchor positions (retrieved from parameters)
        # self.anchor_positions = []
        # for i in range(6):
        #     self.declare_parameter(f'pose_{i+1}_x', 0.0)
        #     self.declare_parameter(f'pose_{i+1}_y', 0.0)
        #     self.declare_parameter(f'pose_{i+1}_z', 0.0)
        #     self.anchor_positions.append(np.array([
        #         self.get_parameter(f'pose_{i+1}_x').value,
        #         self.get_parameter(f'pose_{i+1}_y').value,
        #         self.get_parameter(f'pose_{i+1}_z').value
        #     ]))
        
        # delairing the parameters
        self.declare_parameter('pose_1_x', 0.0), self.declare_parameter('pose_1_y', 0.0), self.declare_parameter('pose_1_z', 0.0)
        self.declare_parameter('pose_2_x', 0.0), self.declare_parameter('pose_2_y', 0.0), self.declare_parameter('pose_2_z', 0.0)
        self.declare_parameter('pose_3_x', 0.0), self.declare_parameter('pose_3_y', 0.0), self.declare_parameter('pose_3_z', 0.0)
        self.declare_parameter('pose_4_x', 0.0), self.declare_parameter('pose_4_y', 0.0), self.declare_parameter('pose_4_z', 0.0)
        self.declare_parameter('pose_5_x', 0.0), self.declare_parameter('pose_5_y', 0.0), self.declare_parameter('pose_5_z', 0.0)
        self.declare_parameter('pose_6_x', 0.0), self.declare_parameter('pose_6_y', 0.0), self.declare_parameter('pose_6_z', 0.0)

        # Retreiving the parameters from the launch file
        self.p1_x = self.get_parameter('pose_1_x').value
        self.p1_y = self.get_parameter('pose_1_y').value
        self.p1_z = self.get_parameter('pose_1_z').value

        self.p2_x = self.get_parameter('pose_2_x').value
        self.p2_y = self.get_parameter('pose_2_y').value
        self.p2_z = self.get_parameter('pose_2_z').value

        self.p3_x = self.get_parameter('pose_3_x').value
        self.p3_y = self.get_parameter('pose_3_y').value
        self.p3_z = self.get_parameter('pose_3_z').value

        self.p4_x = self.get_parameter('pose_4_x').value
        self.p4_y = self.get_parameter('pose_4_y').value
        self.p4_z = self.get_parameter('pose_4_z').value

        self.p5_x = self.get_parameter('pose_5_x').value
        self.p5_y = self.get_parameter('pose_5_y').value
        self.p5_z = self.get_parameter('pose_5_z').value

        self.p6_x = self.get_parameter('pose_6_x').value
        self.p6_y = self.get_parameter('pose_6_y').value
        self.p6_z = self.get_parameter('pose_6_z').value

        self.declare_parameter('noise_variance', 0.0)
        self.declare_parameter('timing', 0.0) 
        self.noise_variance = self.get_parameter('noise_variance').value
        self.timing = self.get_parameter('timing').value

        # UKF Initialization
        self.sigmas = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2.0, kappa=-3)
        self.ukf = UnscentedKalmanFilter(dim_x=6, dim_z=6, dt=self.timing, fx=self.fx, hx=self.hx, points=self.sigmas)
        self.ukf.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Initial state
        self.ukf.P *= 5*1e-7  # Initial uncertainty
        self.ukf.R *= self.noise_variance  # Measurement noise
        self.ukf.Q[0:6, 0:6] = np.eye(6) * self.noise_variance * 1e-5 # Process noise
        self.omega = 1


    def fx(self, x, dt):
        """State transition function for 3D circular motion with constant angular velocity."""
        # F = np.array([
        #     [1, 0, 0, np.cos(self.omega*dt), -np.sin(self.omega*dt), 0],
        #     [0, 1, 0, np.sin(self.omega*dt), np.cos(self.omega*dt), 0],
        #     [0, 0, 1, 0, 0, dt],
        #     [0, 0, 0, 1, 0, 0],
        #     [0, 0, 0, 0, 1, 0],
        #     [0, 0, 0, 0, 0, 1]
        # ])

        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=float)


        return F @ x
    

    def hx(self, x):
        """Measurement function: calculates distances to anchors."""
        p_anchor = np.array([[self.p1_x, self.p1_y, self.p1_z],
                             [self.p2_x, self.p2_y, self.p2_z],
                             [self.p3_x, self.p3_y, self.p3_z],
                             [self.p4_x, self.p4_y, self.p4_z],
                             [self.p5_x, self.p5_y, self.p5_z],
                             [self.p6_x, self.p6_y, self.p6_z]])
        xyz = x[0:3]
        distances = []
        for i in range(6):
            distances.append(np.linalg.norm(xyz - p_anchor[i]))  # Calculate distances
        return distances

    def callback_d1(self, dis1):
        self.d1 = dis1.data

    def callback_d2(self, dis2):
        self.d2 = dis2.data

    def callback_d3(self, dis3):
        self.d3 = dis3.data

    def callback_d4(self, dis4):
        self.d4 = dis4.data

    def callback_d5(self, dis5):
        self.d5 = dis5.data

    def callback_d6(self, dis6):
        self.d6 = dis6.data

    def distances(self):
        return self.d1, self.d2, self.d3, self.d4, self.d5, self.d6

    def Estimated_position_ft(self):
        """Estimate the position using the distances and apply UKF."""
        # Get distances
        dis1, dis2, dis3, dis4, dis5, dis6 = self.distances()
        m_distances = np.array([float(dis1), float(dis2), float(dis3), float(dis4), float(dis5), float(dis6)])

        # UKF Update
        self.ukf.predict()  # Prediction step (once per time step)
        self.ukf.update(m_distances)  # Update step with all measurements at once

        # Get the filtered state
        filtered_state = self.ukf.x

        # Publish the filtered position
        rpose = Pose()
        rpose.position.x = filtered_state[0]
        rpose.position.y = filtered_state[1]
        rpose.position.z = filtered_state[2]
        rpose.orientation.x = 0.0
        rpose.orientation.y = 0.0
        rpose.orientation.z = 0.0
        rpose.orientation.w = 1.0

        self.pub1.publish(rpose)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()

    try:
        while rclpy.ok():
            node.Estimated_position_ft()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()