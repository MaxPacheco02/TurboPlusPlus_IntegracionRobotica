#!/usr/bin/env python3

import rclpy
from service.POS_NOD import ObjPos

def main(args=None):
    rclpy.init(args=args)
    node = ObjPos()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    