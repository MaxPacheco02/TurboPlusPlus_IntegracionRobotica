#!/usr/bin/env python3

import rclpy
from aru_detection.POS_NOD import ObjPos

def main(args=None):
    rclpy.init(args=args)
    node = ObjPos()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    