#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('test_joy_teleop')
        
        # Create a subscriber to the /joy topic
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10  # QoS history depth
        )

        # Create two publishers for VelocitySetL and VelocitySetR
        self.vel_pub_l = self.create_publisher(Float32, 'VelocitySetL', 10)
        self.vel_pub_r = self.create_publisher(Float32, 'VelocitySetR', 10)

    def joy_callback(self, msg: Joy):
        # Extract axis values from the Joy message
        axes = msg.axes
        
        # For example, use the first axis for left velocity and the second for right velocity
        # You can change this logic to fit your use case
        left_velocity = axes[1]  # Example: left stick vertical (axis 1)
        right_velocity = axes[4]  # Example: right stick vertical (axis 4)

        # Create Float32 messages
        left_velocity_msg = Float32()
        left_velocity_msg.data = left_velocity
        
        right_velocity_msg = Float32()
        right_velocity_msg.data = right_velocity
        
        # Publish the velocities
        self.vel_pub_l.publish(left_velocity_msg)
        self.vel_pub_r.publish(right_velocity_msg)

        # self.get_logger().info(f'Published Left Velocity: {left_velocity}, Right Velocity: {right_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
