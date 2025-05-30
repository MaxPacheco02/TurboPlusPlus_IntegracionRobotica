#!/usr/bin/env python3

import rclpy
from aru_detection.POS_CAL import CalNode

def main(args=None):
    rclpy.init(args=args)
    node = CalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()