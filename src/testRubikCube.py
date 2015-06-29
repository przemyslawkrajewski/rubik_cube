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
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=False,planner=None,manageIrpos=True)
	print "init done"
	
	#RCM.moveToSynchroPosition()
	
	gotCube = RCM.getCubeFromHumanToPostument()
	
	#if gotCube:

	#RCM.sidePostumentRobotTest()
	
	RCM.getCubeFromPostumentToTrack()
	RCM.correctCubeGripTrack()
	
	RCM.getCubeFromTrackToPostument()
	RCM.correctCubeGripPostument()
	
	RCM.getCubeFromPostumentToTrackSide()
		
	RCM.giveBackCubeTrack()
	
	#time.sleep(100)
	print "program done" 
