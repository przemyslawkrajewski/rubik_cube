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
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=False,planner=None)
	print "init done"
	#RCM.getCubeFromTrackToPostument()
	
	#RCM.moveToSynchroPosition()
	
	gotCube = RCM.getCubeFromHumanToTrack()
	
	if gotCube:
		RCM.correctCubeGripTrack()
	
		RCM.getCubeFromTrackToPostument()
		RCM.correctCubeGripPostument()
		
		RCM.giveBackCubePostument()
	time.sleep(100)
	print "program done" 
