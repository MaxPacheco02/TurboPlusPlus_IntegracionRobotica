#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from tf2_ros import TransformListener, Buffer

class Localization(Node):

    def __init__(self):
        super().__init__('Localization')
        self.get_logger().info('LOCALIZACION CON ARUCOS')
        self.pzb_pos = self.create_subscription(Odometry, '/pzb/odom', self.odom_callback, 10)
        
        self.tf_buff = Buffer()
        self.listener = TransformListener(self.tf_buff, self)
        
        # ROBOT MOTION MODEL S
        
        # ROBOT OBSERVATION Z

    def odom_callback(self,msg):
        return
         
    
    def lookup_transform(self):
        if self.tf_buff.can_transform('camera','aruco_center',rclpy.time.Time()):
            tf = self.tf_buff.lookup_transform('camera','aruco_center',rclpy.time.Time())
            self.get_logger().info(f"Aruco_23 in odom → x: {tf.transform.translation.z:.2f}, y: {tf.transform.translation.y:.2f}")
        else:
            return

    def EKF_localization(self):
        return     

    def primer_paso(self):
        return
    
    def segundo_paso(self):
        return 
    
    def tercer_paso(self):
        return
    
    def cuarto_paso(self):
        return

    def quinto_paso(self):
        return

    def sexto_paso(self):
        return

    def septimo_paso(self):
        return

    def octavo_paso(self):
        return

    def noveno_paso(self):
        return

    def decimo_paso(self):
        return

    def onceavo_paso(self):
        return


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()