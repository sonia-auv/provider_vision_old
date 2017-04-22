#!/usr/bin/env python

import roslib
import math

roslib.load_manifest('provider_vision')
import rospy

from proc_image_processing.msg import VisionTarget

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to:
%s
---------------------------
Moving around:
        w
   a    s    d

Angle:
---------------------------
CCW: Shift+W
CQ:  Shift+S

Size:
---------------------------
Right Arrow : Increase Width (+width)
Left Arrow  : Decrease Width (-width)
Up Arrow    : Increase Height (+height)
Down Arrow  : Decrease Height (-height)

CTRL-C to quit
"""

move_bindings = {
    'w': (0.5, 0, 0, 0, 0),
    's': (-0.5, 0, 0, 0, 0),
    'a': (0, 0.5, 0, 0, 0),
    'd': (0, -0.5, 0, 0, 0),
    'W': (0, 0, -0.5, 0, 0),
    'S': (0, 0, 0.5, 0, 0),
    '\x41': (0, 0, 0, 0.5, 0),  # UP
    '\x42': (0, 0, 0, -0.5, 0),  # DOWN
    '\x43': (0, 0, 0, 0, 0.5),  # RIGHT
    '\x44': (0, 0, 0, 0, -0.5),  # LEFT
}


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    if key == '\x03':
        exit(0)
    return key


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def get_odom_from_key():
    global x, y, angle, height, width
    key = get_key()
    if key in move_bindings.keys():
        x += move_bindings[key][0]
        y += move_bindings[key][1]
        angle += move_bindings[key][2]
        angle = clamp(angle, -math.pi, math.pi)
        height += move_bindings[key][3]
        width += move_bindings[key][4]


def usage():
    print("Usage: %s <detection_task_name>" % sys.argv[0])


if __name__ == "__main__":

    if len(sys.argv) != 2:
        usage()
        exit(1)

    detection_task_name = sys.argv[1]

    settings = termios.tcgetattr(sys.stdin)

    target_pub = rospy.Publisher('/provider_vision/' + detection_task_name +
                                 '_result',
                                 VisionTarget, queue_size=100)
    rospy.init_node('target_simulator_' + detection_task_name)

    x = 0
    y = 0
    angle = 0
    height = 0
    width = 0

    print(msg % ('/provider_vision/' + detection_task_name + '_result'))
    while True:
        get_odom_from_key()

        target = VisionTarget()
        target.x = x
        target.y = y
        target.angle = angle
        target.width = width
        target.height = height
        target_pub.publish(target)
