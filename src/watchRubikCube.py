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
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=False,planner=None,manageIrpos=True,csn=False)
	
	#RCM.determineColors()
	
	RCM.watchCubeTrack()
