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
	gripJoint=0.055
	openJoint=0.068
	wideOpenJoint=0.09

	def __init__(self,mode='urdf',xmlFile='data/irp6both.env.xml',viewerEnabled=True,manageIrpos=True,simulate=True,planner=None):
		self.env, self.robot = openraveIrp6.initialize(mode,xmlFile,viewerEnabled,manageIrpos,planner)
		self.simulate=simulate
		
	def moveToSynchroPosition(self):
		self.robot.track.moveToSynchroPosition(self.simulate)
		self.robot.postument.moveToSynchroPosition(self.simulate)
	#
	#
	#	moving manipulator to each other
	#
	#
	
	def facePostument(self,z_dist=0.2,x_dist=0.0,y_dist=-0.0,showAngle=pi/2):
		#Set postument position
		#self.robot.track.moveToJointPosition([0,7.412760409739285e-06, -1.4791420483915523, -0.16172125899257106, 0.07006621675194569, 4.712388138719054, -1.5707949127454675],self.simulate)
		#self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		#return
		#Get postument position
		postTrans = self.robot.postument.getCartesianPosition()
		#Transform rotation to PyKDL
		postRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	
		#rotate
		trackRot = postRot
		trackRot.DoRotY(self.pi)
		trackRot.DoRotZ(showAngle)
		# build quaternion
		trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])
		#build position with translation along z axis
		trackPoint = Point (postTrans[0][2]*z_dist + postTrans[0][1]*y_dist + postTrans[0][0]*x_dist + postTrans[0][3],
							postTrans[1][2]*z_dist + postTrans[1][1]*y_dist + postTrans[1][0]*x_dist + postTrans[1][3]+self.robotsDistance,
							postTrans[2][2]*z_dist + postTrans[2][1]*y_dist + postTrans[2][0]*x_dist + postTrans[2][3])
		#perform movement
		self.robot.track.moveToCartesianPosition(Pose(trackPoint,trackQuaternion),self.simulate)
	def faceTrack(self,z_dist=0.2,x_dist=0.0,y_dist=0.0,showAngle=pi/2):
		#Set postument position
		#self.robot.track.moveToJointPosition([0.2, 1.887562902394057, -1.3432024707381522, -0.2854391974722371, -0.8070492267377007, 4.67043035734827, -0.05821693027504106],self.simulate)
		#Get postument position
		trackTrans = self.robot.track.getCartesianPosition()
		#Transform rotation to PyKDL
		trackRot = PyKDL.Rotation(trackTrans[0][0],trackTrans[0][1],trackTrans[0][2],	trackTrans[1][0],trackTrans[1][1],trackTrans[1][2],	trackTrans[2][0],trackTrans[2][1],trackTrans[2][2])	
		#rotate
		postRot = trackRot
		postRot.DoRotY(self.pi)
		trackRot.DoRotZ(showAngle)
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
		#self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		#Get postument position
		postTrans = self.robot.postument.getCartesianPosition()
		#Transform rotation to PyKDL
		postRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	

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

				trackMat = self.quaternion_to_matrix(trackRot.GetQuaternion())

				#build position with translation along z axis
				trackPoint = Point (trackMat[0][2]*z_dist+trackMat[0][1]*y_dist+trackMat[0][0]*x_dist+postTrans[0][3],
									trackMat[1][2]*z_dist+trackMat[1][1]*y_dist+trackMat[1][0]*x_dist+postTrans[1][3]+self.robotsDistance,
									trackMat[2][2]*z_dist+trackMat[2][1]*y_dist+trackMat[2][0]*x_dist+postTrans[2][3])

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
		
		trackMat = self.quaternion_to_matrix(trackRot.GetQuaternion())
		
		#build position with translation along z axis
		trackPoint = Point (-trackMat[0][2]*z_dist+trackMat[0][1]*y_dist+trackMat[0][0]*x_dist+postTrans[0][3],
							-trackMat[1][2]*z_dist+trackMat[1][1]*y_dist+trackMat[1][0]*x_dist+postTrans[1][3]+self.robotsDistance,
							-trackMat[2][2]*z_dist+trackMat[2][1]*y_dist+trackMat[2][0]*x_dist+postTrans[2][3])
		
		#perform movement
		self.robot.track.moveToCartesianPosition(Pose(trackPoint,trackQuaternion),self.simulate)
	#
	#
	#	Getting cube
	#
	#

	def getCubeFromHumanToTrack(self):
		#self.robot.track.moveToJointPosition([0.549969, 0.9265588068203238, -1.5550934102940197, 0.04871716017958537, 1.3063556317588827, 2.6762106103323573, -0.13097870260389663],self.simulate)
		self.robot.track.moveToCartesianPosition(Pose(Point(1,1,1.18),Quaternion(-0.441448339193, 0.440445333852, -0.552396228639, 0.55316333781)),self.simulate)
		self.robot.track.tfgToJointPosition(position=0.068,simulate=self.simulate)
		self.robot.track.startForceControl(tran_z=True,mov_z=0.013)
		gotCube=False
		for i in range(1,50):
			reading = self.robot.track.getForceReadings()
			time.sleep(0.1)
			if reading.force.z > 8:
				gotCube=True
				break
		self.robot.track.stopForceControl()
		if gotCube:
			self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		else:
			self.robot.track.moveToCartesianPosition(Pose(Point(1,1,1.18),Quaternion(-0.441448339193, 0.440445333852, -0.552396228639, 0.55316333781)),self.simulate)

		return gotCube

	def getCubeFromHumanToPostument(self):
		#self.robot.track.moveToJointPosition([1, 0.0, -1.4476547584053457, 0.012313341484619551, 1.2106388401258297, 1.57, 0],self.simulate)
		self.robot.postument.moveToCartesianPosition(Pose(Point(0.89668151225, 0.10325876098, 1.32973306404),Quaternion(-0.247541182161, 0.554930714941, -0.682010199583, 0.406985690674)),self.simulate)
		self.robot.postument.tfgToJointPosition(position=0.068,simulate=self.simulate)
		self.robot.postument.startForceControl(tran_z=True,mov_z=0.013)
		gotCube=False
		for i in range(1,50):
			reading = self.robot.postument.getForceReadings()
			time.sleep(0.1)
			if reading.force.z > 8:
				gotCube=True
				break
		self.robot.postument.stopForceControl()
		if gotCube:
			self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)

		return gotCube
		
	def getCubeFromTrackToPostument(self):
		self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		self.facePostument(z_dist=0.15,y_dist=-0.01)
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		self.facePostument(z_dist=0.006,y_dist=-0.01)
		
		self.robot.track.startForceControl(tran_x=True,tran_y=True)
		self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		self.robot.track.stopForceControl()
		
		self.facePostument(z_dist=0.15,y_dist=-0.01)
	def getCubeFromTrackToPostumentSide(self,angle=0):
		self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		self.sidePostument(z_dist=0.15,y_dist=-0.015,x_dist=-0.01,showAngle=3*self.pi/4,angle=self.pi/4)
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		self.sidePostument(z_dist=0.02,y_dist=-0.015,x_dist=-0.01,showAngle=3*self.pi/4,angle=self.pi/4)
		
		self.sidePostument(z_dist=0.15,y_dist=-0.015,x_dist=-0.01,showAngle=3*self.pi/4,angle=self.pi/4)
		
	def getCubeFromPostumentToTrack(self):
		self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		self.facePostument(z_dist=0.15,y_dist=-0.01)
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		self.facePostument(z_dist=0.006,y_dist=-0.01)
		
		self.robot.postument.startForceControl(tran_x=True,tran_y=True)
		self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		
		self.facePostument(z_dist=0.15,y_dist=-0.01)
		
	def correctCubeGripPostument(self):
		self.robot.postument.moveToJointPosition([0, -1.541861095576026, 0.0, 0.1007174886590251, 1.57, -1.01731301362624],self.simulate)
		self.robot.postument.tfgToJointPosition(position=self.openJoint,simulate=self.simulate)
		self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		
	def correctCubeGripTrack(self):
		self.robot.track.moveToJointPosition([0.1, 0, -1.541861095576026, 0.0, 0.1007174886590251, 1.57, -1.01731301362624],self.simulate)
		self.robot.track.tfgToJointPosition(position=self.openJoint,simulate=self.simulate)
		self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		
	def giveBackCubeTrack(self,throw=False):
		self.robot.track.moveToJointPosition([0.1, 0, -1.541861095576026, 0.0, 0.1007174886590251, 4.73, -1.01731301362624],self.simulate)
		if throw==False:
			self.robot.track.moveRelativeToCartesianPosition(z_tran=0.15)
		self.robot.track.tfgToJointPosition(position=self.openJoint,simulate=self.simulate)
		if throw==False:
			self.robot.track.moveRelativeToCartesianPosition(z_tran=-0.15)
	def giveBackCubePostument(self,throw=False):
		self.robot.postument.moveToJointPosition([0, -1.541861095576026, 0.0, 0.1007174886590251, 4.73, -1.01731301362624],self.simulate)
		if throw==False:
			self.robot.postument.moveRelativeToCartesianPosition(z_tran=0.15)
		self.robot.postument.tfgToJointPosition(position=self.openJoint,simulate=self.simulate)
		if throw==False:
			self.robot.postument.moveRelativeToCartesianPosition(z_tran=-0.15)
	#
	#
	#	Watching cube
	#
	#


	def quaternion_to_matrix(self,q):
		X=q[0];
		Y=q[1];
		Z=q[2];
		W=q[3];
		#return numpy.array([
		#[1.0-2*Y*Y-2*Z*Z,	2*X*Y+2*Z*W,	2*X*Z-2*Y*W],
		#[2*X*Y-2*Z*W,		1-2*X*X-2*Z*Z,	2*Y*Z+2*X*W],
		#[2*X*Z+2*Y*W,		2*Y*Z-2*X*W,	1-2*X*X-2*Y*Y]
		#])
		return numpy.array([
		[1.0-2*Y*Y-2*Z*Z,	2*X*Y-2*Z*W,	2*X*Z+2*Y*W],
		[2*X*Y+2*Z*W,		1-2*X*X-2*Z*Z,	2*Y*Z-2*X*W],
		[2*X*Z-2*Y*W,		2*Y*Z+2*X*W,	1-2*X*X-2*Y*Y]
		])
