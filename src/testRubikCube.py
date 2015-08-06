#!/usr/bin/env python

import rospy
import tf
import actionlib

import time

import openrave_irp6
from irpos import *

import rubikCubeManipulator

#MAIN

if __name__ == '__main__':
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=False,planner=None,manageIrpos=True,csn=True)
	print "init done"
	
	#gotCube = RCM.getCubeFromHumanToPostument()
	RCM.setCorrectionFace()
	RCM.setCorrectionSide()
	
	RCM.showCubeFaceToPostument(side=1)

	#TEST FLIPPING CUBE
	"""RCM.flipCubePostument()
	RCM.giveBackCubePostument()

	#TEST WATCHING CUBE
	RCM.showCubeFaceToPostument(side=0)
	time.sleep(2)
	RCM.showCubeFaceToPostument(side=1)
	time.sleep(2)
	RCM.showCubeFaceToPostument(side=2)
	time.sleep(2)
	RCM.showCubeFaceToPostument(side=3)
	time.sleep(2)
	RCM.showCubeFaceToPostument(side=4)"""

	#time.sleep(100)
	print "program done" 
