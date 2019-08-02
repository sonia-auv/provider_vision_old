#!/usr/bin/env python

import rospy
import os
import time
import subprocess
import sys
from provider_vision.srv import *

if __name__ == "__main__":
    while True:
        value = os.system("roslaunch provider_vision provider_vision.launch")
        print "Return value : {}".format(value)
        if value != 0:
            break
