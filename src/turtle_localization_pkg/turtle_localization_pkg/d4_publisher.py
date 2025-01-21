#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float64
import numpy as np


class Pose4Publisher(Node):
    def __init__(self):
        super().__init__('d4_publisher_with_noise')

        # Declare parameters
        self.declare_parameter('pose_4_x', 4.5)
        self.declare_parameter('pose_4_y', 2.0)
        self.declare_parameter('pose_4_z', 1.0)
        self.declare_parameter('noise_mean', 0.0)       # Mean of the noise
        self.declare_parameter('noise_variance', 0.25)  # Variance of the noise
        self.declare_parameter('timing', 0.0)
        self.timing = self.get_parameter('timing').value
        

        # Retrieve parameters
        self.pose_x = self.get_parameter('pose_4_x').value
        self.pose_y = self.get_parameter('pose_4_y').value
        self.pose_z = self.get_parameter('pose_4_z').value
        self.noise_mean = self.get_parameter('noise_mean').value
        self.noise_variance = self.get_parameter('noise_variance').value

        # Subscriber for Real Position
        self.create_subscription(Pose, '/turtle1/pose', self.callback_pr, 10)

        # Publisher for distance
        self.pub = self.create_publisher(Float64, '/d4_with_noise', 10)

        # Timer for periodic updates
        self.timer = self.create_timer(self.timing, self.calculate_distance_4)

        self.xp = 0.0
        self.yp = 0.0
        self.zp = 0.0

    def callback_pr(self, Turtle_Pose):
        """Callback to update the real position."""
        self.xp = Turtle_Pose.x
        self.yp = Turtle_Pose.y

    def calculate_distance_4(self):
        """Compute and publish distance without noise."""
        dis4 = Float64()

        # Calculate standard deviation from variance
        self.noise_std_dev = np.sqrt(self.noise_variance)
        # Add controlled white noise to distance
        noise_distance = np.random.normal(self.noise_mean, self.noise_std_dev)

        # Calculate the differences
        dist_x = np.abs(self.xp - self.pose_x)
        dist_y = np.abs(self.yp - self.pose_y)
        dist_z = np.abs(self.zp - self.pose_z)

        # Compute Euclidean distance
        dis4.data = np.sqrt(dist_x ** 2 + dist_y ** 2 + dist_z ** 2) + noise_distance

        # Publish the distance
        self.pub.publish(dis4)


def main(args=None):
    rclpy.init(args=args)
    node = Pose4Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pose4Publisher node terminated.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
