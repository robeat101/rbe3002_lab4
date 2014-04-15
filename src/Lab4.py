#!/usr/bin/env python

import rospy
from AStar.py import *
from Movement.py import *


if __name__ == '__main__':
    try:
        astar_run()
    except rospy.ROSInterruptException:
        pass
