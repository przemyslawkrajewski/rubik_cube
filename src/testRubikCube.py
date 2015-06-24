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
	
	RCM = rubikCubeManipulator.RubikCubeManipulator(simulate=True,planner=None)
	print "init done"
	
	#RCM.moveToSynchroPosition()
	
	gotCube = RCM.getCubeFromHumanToPostument()
	
	#if gotCube:
	#RCM.correctCubeGripPostument()
	RCM.getCubeFromPostumentToTrack()
	RCM.correctCubeGripTrack()
	RCM.getCubeFromTrackToPostument()
	
	#RCM.correctCubeGripTrack()
	#RCM.getCubeFromTrackToPostument()
	#RCM.getCubeFromTrackToPostumentSide()
		
	RCM.giveBackCubePostument()
	
	time.sleep(100)
	print "program done" 
