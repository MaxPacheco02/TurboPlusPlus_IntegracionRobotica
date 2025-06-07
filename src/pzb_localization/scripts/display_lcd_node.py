#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import threading
import time

# Import our conversion and transmission modules
from convert2base64 import occupancy_grid_to_base64
from send_base64 import send_base64

class MapToSerialNode(Node):
    """
    A ROS2 node that subscribes to OccupancyGrid map, converts it to base64,
    and sends it via serial port to display on LCD.
    """
    
    def __init__(self):
        super().__init__('display_lcd_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB2')
        self.declare_parameter('update_rate', 0.5)  # Hz
        self.declare_parameter('image_width', 240)
        self.declare_parameter('image_height', 128)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # Create a subscription to the map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10  # QoS profile depth
        )
        
        # Store the latest map
        self.latest_map = None
        self.map_lock = threading.Lock()
        
        # Create a timer for periodic transmission
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info(f"Map to Serial Node started. Listening on 'map', sending to {self.serial_port}")
    
    def map_callback(self, msg):
        """Callback for receiving map updates"""
        with self.map_lock:
            self.latest_map = msg
        self.get_logger().debug("Received new map update")
    
    def timer_callback(self):
        """Periodic callback to process and send the map"""
        with self.map_lock:
            if self.latest_map is None:
                return
            
            current_map = self.latest_map
        
        try:
            # Convert the map to base64
            base64_data = occupancy_grid_to_base64(
                current_map, 
                width=self.image_width, 
                height=self.image_height
            )
            
            self.get_logger().info(f"Converted map to base64 - {len(base64_data)} characters")
            
            # Send the base64 data via serial
            success = send_base64(self.serial_port, base64_data)
            
            if success:
                self.get_logger().info("Map successfully sent via serial")
            else:
                self.get_logger().error("Failed to send map via serial")
                
        except Exception as e:
            self.get_logger().error(f"Error processing map: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    node = MapToSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
