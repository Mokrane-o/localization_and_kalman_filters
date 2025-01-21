#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class CircleMotion(Node):
    def __init__(self):
        super().__init__('circle_motion')
        
        # Hardcoded values for the circle
        self.radius = 2.0   # Radius of the circle
        self.speed = 2.0    # Linear speed

        # Publisher to control the turtle
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to get the current pose of the turtle
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = None

        # Timer to publish velocity commands
        self.declare_parameter('timing', 0.0)
        self.timing = self.get_parameter('timing').value
        self.timer = self.create_timer(self.timing, self.move_circle)

        print(f"angular velocity is : {self.speed / self.radius}")

    def pose_callback(self, msg):
        """Callback to get the current pose of the turtle."""
        self.current_pose = msg

    def move_circle(self):
        """Move the turtle in a circular path."""
        if self.current_pose is None:
            return  # Wait for the first pose message

        # Calculate the angular velocity for circular motion
        angular_velocity = self.speed / self.radius

        # Create a Twist message to control the turtle
        twist = Twist()
        twist.linear.x = self.speed  # Linear velocity
        twist.angular.z = angular_velocity  # Angular velocity

        # Publish the Twist message
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    circle_motion = CircleMotion()
    rclpy.spin(circle_motion)
    circle_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()