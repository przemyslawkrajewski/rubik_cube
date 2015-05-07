#!/usr/bin/env python

import rospy
import tf
import actionlib

import time

from openrave_irp6 import *
from irpos import *

import PyKDL

class RubikCubeManipulator:

	robotsDistance=2.10
	pi=3.1415926535

	def __init__(self,mode='urdf',xmlFile='data/irp6both.env.xml',viewerEnabled=True,manageIrpos=True,simulate=True):
		self.env, self.robot = openraveIrp6.initialize(mode,xmlFile,viewerEnabled,manageIrpos)
		self.simulate=simulate
		
	def moveToSynchroPosition(self):
		self.robot.track.moveToSynchroPosition(self.simulate)
		self.robot.postument.moveToSynchroPosition(self.simulate)
	def facePostument(self,z_dist=0.1,x_dist=0.0,y_dist=0.0):
		#Set postument position
		self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		#Get postument position
		postTrans = self.robot.postument.getCartesianPosition()
		#Transform rotation to PyKDL
		postRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	
		#rotate
		trackRot = postRot
		trackRot.DoRotY(3.14)
		# build quaternion
		trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])
		#build position with translation along z axis
		trackPoint = Point (postTrans[0][2]*z_dist+postTrans[0][1]*y_dist+postTrans[0][0]*x_dist+postTrans[0][3],
							postTrans[1][2]*z_dist+postTrans[1][1]*y_dist+postTrans[1][0]*x_dist+postTrans[1][3]+self.robotsDistance,
							postTrans[2][2]*z_dist+postTrans[2][1]*y_dist+postTrans[2][0]*x_dist+postTrans[2][3])
		#perform movement
		self.robot.track.moveToCartesianPosition(Pose(trackPoint,trackQuaternion),self.simulate)
	def faceTrack(self,z_dist=0.1,x_dist=0.0,y_dist=0.0):
		#Set postument position
		#self.robot.track.moveToJointPosition([0.2, 1.887562902394057, -1.3432024707381522, -0.2854391974722371, -0.8070492267377007, 4.67043035734827, -0.05821693027504106],self.simulate)
		#Get postument position
		trackTrans = self.robot.track.getCartesianPosition()
		#Transform rotation to PyKDL
		trackRot = PyKDL.Rotation(trackTrans[0][0],trackTrans[0][1],trackTrans[0][2],	trackTrans[1][0],trackTrans[1][1],trackTrans[1][2],	trackTrans[2][0],trackTrans[2][1],trackTrans[2][2])	
		#rotate
		postRot = trackRot
		postRot.DoRotY(self.pi)
		# build quaternion
		postQuaternion = Quaternion(postRot.GetQuaternion()[0],postRot.GetQuaternion()[1],postRot.GetQuaternion()[2],postRot.GetQuaternion()[3])
		#build position with translation along z axis
		postPoint = Point (trackTrans[0][2]*z_dist+trackTrans[0][1]*y_dist+trackTrans[0][0]*x_dist+trackTrans[0][3],
							trackTrans[1][2]*z_dist+trackTrans[1][1]*y_dist+trackTrans[1][0]*x_dist+trackTrans[1][3]+self.robotsDistance,
							trackTrans[2][2]*z_dist+trackTrans[2][1]*y_dist+trackTrans[2][0]*x_dist+trackTrans[2][3])
		#perform movement
		self.robot.postument.moveToCartesianPosition(Pose(postPoint,postQuaternion),self.simulate)
	def sidePostument(self,z_dist=0.1,x_dist=0.0,y_dist=0.0,showAngle=0,angle=None):	
		#Set postument position
		self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		#Get postument position
		postTrans = self.robot.postument.getCartesianPosition()
		#Transform rotation to PyKDL
		postRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	

		#build position with translation along z axis
		trackPoint = Point (postTrans[0][2]*z_dist+postTrans[0][1]*y_dist+postTrans[0][0]*x_dist+postTrans[0][3],
							postTrans[1][2]*z_dist+postTrans[1][1]*y_dist+postTrans[1][0]*x_dist+postTrans[1][3]+self.robotsDistance,
							postTrans[2][2]*z_dist+postTrans[2][1]*y_dist+postTrans[2][0]*x_dist+postTrans[2][3])
		if angle==None:
			angle=0
			while angle<self.pi*2:
				#rotate
				trackRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])
				trackRot.DoRotZ(angle)
				trackRot.DoRotY(self.pi/2)
				trackRot.DoRotZ(showAngle)
				# build quaternion
				trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])
				if self.robot.track.isIKSolutionExist(Pose(trackPoint,trackQuaternion)):
					break
				angle+=0.01
		#rotate
		trackRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	
		trackRot.DoRotZ(angle)
		trackRot.DoRotY(self.pi/2)
		trackRot.DoRotZ(showAngle)
		# build quaternion
		trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])
		if self.robot.track.isIKSolutionExist(Pose(trackPoint,trackQuaternion)):
			print "Yay!"
		else:
			print "No!"
		#perform movement
		self.robot.track.moveToCartesianPosition(Pose(trackPoint,trackQuaternion),self.simulate)
