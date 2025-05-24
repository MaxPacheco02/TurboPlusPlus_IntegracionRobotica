#!/usr/bin/env python3

import sys
import threading

import geometry_msgs.msg
import nav_msgs.msg
import rclpy

import termios
import tty
import math

msg = """
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : auto/teleop op. mode

CTRL-C to quit
"""

x_vel = 0.4
z_vel = 1.5

moveBindings = {
    'i': (x_vel, 0.),
    'o': (x_vel, -z_vel),
    'j': (0., z_vel),
    'l': (0., -z_vel),
    'u': (x_vel, z_vel),
    ',': (-x_vel, 0.),
    '.': (-x_vel, z_vel),
    'm': (-x_vel, -z_vel),
}

opModeBindings = {
    'a': 1.,
    'z': 0.,
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(op_mode):
    mode = ['tele', 'auto']
    return 'currently:\top mode %s ' % (mode[int(op_mode)])   

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('pzb_teleop_low_level_node')

    op_msg = geometry_msgs.msg.Twist()

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    op_mode = 0.
    x = 0.
    z = 0.

    try:
        print(msg)
        print(vels(op_mode))
        status = 0
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                z = moveBindings[key][1]
            elif key in opModeBindings.keys():
                op_mode = opModeBindings[key]
                print(op_mode)

                print(vels(op_mode))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                z = 0.0
                if (key == '\x03'):
                    break
            
            op_msg.linear.x = x
            op_msg.angular.z = z
            pub.publish(op_msg)

    except Exception as e:
        print(e)

    finally:
        op_msg.linear.x = 0.0
        op_msg.angular.z = 0.0
        pub.publish(op_msg)
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()