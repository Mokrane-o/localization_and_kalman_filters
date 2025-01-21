import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32  # Message type for publishing x
import numpy as np

class TurtlePoseSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber_publisher')

        # Subscription to the /turtle1/pose topic
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher for the x value
        self.publisher = self.create_publisher(Float32, '/turtle1/x', 10)

    def pose_callback(self, msg):
        # Extract x and angular_velocity (w)
        x = msg.x
        y = msg.y
        w = msg.angular_velocity

        # Log the values
        self.get_logger().info(f'Subscribed -> x: {x}, y: {y}, w: {w}')

        # Publish the x value
        x_msg = Float32()
        y_msg = Float32()
        # noise = np.random.normal(0.0, 0.0)
        x_msg.data = x + np.random.normal(0.0, 0.2)
        y_msg.data = y
        self.publisher.publish(x_msg)
        self.get_logger().info(f'Published -> x: {x} , y: {y}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriberPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown and destroy the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
