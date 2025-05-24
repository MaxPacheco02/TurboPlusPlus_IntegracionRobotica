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

moveBindings = {
    'i': (1., 0.),
    'o': (1., -1.),
    'j': (0., 1.),
    'l': (0., -1.),
    'u': (1., 1.),
    ',': (-1., 0.),
    '.': (-1., 1.),
    'm': (-1., -1.),
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

    node = rclpy.create_node('pzb_teleop_node')

    op_msg = geometry_msgs.msg.Vector3()

    pub = node.create_publisher(geometry_msgs.msg.Vector3, '/op_cmd', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    op_mode = 0.
    x = 0.
    y = 0.

    try:
        print(msg)
        print(vels(op_mode))
        status = 0
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
            elif key in opModeBindings.keys():
                op_mode = opModeBindings[key]
                print(op_mode)

                print(vels(op_mode))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                op_mode = 0.
                if (key == '\x03'):
                    break
            
            op_msg.x = x
            op_msg.y = y
            op_msg.z = op_mode
            pub.publish(op_msg)

    except Exception as e:
        print(e)

    finally:
        op_msg.x = 0.
        op_msg.y = 0.
        op_msg.z = 0.
        pub.publish(op_msg)
        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()