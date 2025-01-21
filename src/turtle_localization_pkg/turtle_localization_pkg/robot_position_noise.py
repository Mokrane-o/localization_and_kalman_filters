#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import matplotlib.pyplot as plt

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_position_noise')
        
        self.savetime_flag = False
        
        # Subscribers
        self.sub1 = self.create_subscription(Float64, '/d1_with_noise', self.callback_d1, 10)
        self.sub2 = self.create_subscription(Float64, '/d2_with_noise', self.callback_d2, 10)
        self.sub3 = self.create_subscription(Float64, '/d3_with_noise', self.callback_d3, 10)
        self.sub4 = self.create_subscription(Float64, '/d4_with_noise', self.callback_d4, 10)
        self.sub5 = self.create_subscription(Float64, '/d5_with_noise', self.callback_d5, 10)
        self.sub6 = self.create_subscription(Float64, '/d6_with_noise', self.callback_d6, 10)

        # Publishers
        self.pub1 = self.create_publisher(Pose, '/Estimated_Position_noise', 10)

        # Distances
        self.d1 = 0.0
        self.d2 = 0.0
        self.d3 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0

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


        # Initial time tracking for Real_Position
        self.t0 = None

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
        p1 = np.array([self.p1_x, self.p1_y, self.p1_z])
        p2 = np.array([self.p2_x, self.p2_y, self.p2_z])
        p3 = np.array([self.p3_x, self.p3_y, self.p3_z])
        p4 = np.array([self.p4_x, self.p4_y, self.p4_z])
        p5 = np.array([self.p5_x, self.p5_y, self.p5_z])
        p6 = np.array([self.p6_x, self.p6_y, self.p6_z])

        self.p = Float64MultiArray()
        self.rpose = Pose()

        dis1, dis2, dis3, dis4, dis5, dis6 = self.distances()
        distances = np.array([float(dis1), float(dis2), float(dis3), float(dis4), float(dis5), float(dis6)]).reshape(6, 1)
        anchors = np.vstack([p1, p2, p3, p4, p5, p6])

        num_eq = 5
        A = np.zeros((num_eq, 3))
        b = np.zeros((num_eq, 1))

        for i in range(num_eq):
            A[i, :] = 2 * (anchors[i + 1] - anchors[i])
            b[i] = distances[i]**2 - distances[i + 1]**2 - np.linalg.norm(anchors[i, :])**2 + np.linalg.norm(anchors[i + 1, :])**2

        pinv_A = np.linalg.pinv(A)
        
        # Ensure the result is a sequence of floats
        result = np.dot(pinv_A, b).flatten()  # Flatten to ensure it's a 1D array

        # Make sure we have the correct type and range
        self.p.data = list(result.astype(float))  # Convert to a list of floats

        self.rpose.position.x = self.p.data[0]
        self.rpose.position.y = self.p.data[1]
        self.rpose.position.z = self.p.data[2]
        self.rpose.orientation.x = 0.0
        self.rpose.orientation.y = 0.0
        self.rpose.orientation.z = 0.0
        self.rpose.orientation.w = 1.0

        self.pub1.publish(self.rpose)

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
