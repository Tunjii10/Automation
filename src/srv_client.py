#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from automation.srv import *


def movePoseClient():
    # Wait for service 
    rospy.wait_for_service('move_pose') 
    try:
        # position arguments for target object
        posX = -0.136153
        posY = -0.57000
        posZ = 1.490
       
        # initialise service proxy    
        move_Pose = rospy.ServiceProxy('move_pose', movePose)
        resp = move_Pose(posX, posY, posZ)
        # print service call response to screen
        return print(resp.result)
    except rospy.ServiceException as e:
        print("The Service call failed: %s" % e)


if __name__ == "__main__":

    movePoseClient()
