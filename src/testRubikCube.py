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
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=False)
	print "init done"
	#RCM.moveToSynchroPosition()
	RCM.sidePostument(z_dist=0.1,showAngle=0)
	RCM.sidePostument(z_dist=0.1,showAngle=1)
	print "program done"
