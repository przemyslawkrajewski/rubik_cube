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
	RCM.facePostument(z_dist=0.0)
	print "program done"
