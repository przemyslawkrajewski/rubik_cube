#!/usr/bin/env python

import rospy
import tf
import actionlib

import time
import math

import os,sys

import subprocess

from rubik_cube.msg import *

from openrave_irp6 import *
from irpos import *

from RubikCube import *

import cube_convert

import PyKDL

class ColorSample:
	red=0
	blue=0
	green=0
	tile=0
	face=0

class RubikCubeManipulator:

	

	robotsDistance=2.10
	pi=3.1415926535
	gripJoint=0.054
	openJoint=0.072
	wideOpenJoint=0.089
	
	cubeSize = 0.046
	fcx=0.026 #prawo
	fcy=-0.015
	
	#spcx1=0.02 #dol
	#spcy1=0.012
	spcx1=0.0 # dol
	spcy1=0.0
	spcx2=-0.0075
	spcy2=0.01
	
	def __init__(self,mode='urdf',xmlFile='data/irp6both.env.xml',viewerEnabled=True,manageIrpos=True,simulate=True,planner=None,csn=False):
		self.env, self.robot = openraveIrp6.initialize(mode,xmlFile,viewerEnabled,manageIrpos,planner,csn)
		self.simulate=simulate
		self.cube = self.env.ReadKinBodyXMLFile('data/cube.kinbody.xml')
		self.env.Add(self.cube)
		self.cubeColors= CubeColor()
		
		rospy.Subscriber("cube_face_color", Cube_face_color, self.recieveCubeColors)
		
	def moveToSynchroPosition(self):
		self.robot.track.moveToSynchroPosition(self.simulate)
		self.robot.postument.moveToSynchroPosition(self.simulate)
	#
	#
	#	moving manipulator to each other
	#
	#
	
	def facePostument(self,z_dist=0.2,x_dist=0.0,y_dist=-0.0,y_rot=0.0,showAngle=pi/2):
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
		
		#build position with translation along z axis
		trackPoint = Point (postTrans[0][2]*z_dist + postTrans[0][1]*y_dist + postTrans[0][0]*x_dist + postTrans[0][3],
							postTrans[1][2]*z_dist + postTrans[1][1]*y_dist + postTrans[1][0]*x_dist + postTrans[1][3]+self.robotsDistance,
							postTrans[2][2]*z_dist + postTrans[2][1]*y_dist + postTrans[2][0]*x_dist + postTrans[2][3])
		
		trackRot.DoRotZ(showAngle)
		trackRot.DoRotY(y_rot)
		# build quaternion
		trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])
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
	def sidePostument(self,z_dist=0.1,x_dist=0.0,y_dist=0.0,y_rot=0.0,x_rot=0.0,showAngle=0,angle=None):
		#Set postument position
		#self.robot.postument.moveToJointPosition([-2.38227752354415, -1.441861095576026, 0.1, 0.1007174886590251, 0.754815971689398, -1.91731301362624],self.simulate)
		#Get postument position
		postTrans = self.robot.postument.getCartesianPosition()
		#Transform rotation to PyKDL
		postRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	

		postMat = self.quaternion_to_matrix(postRot.GetQuaternion())

		if angle==None:
			angle=0
			while angle<self.pi*2:
				#rotate
				trackRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])
				trackRot.DoRotZ(angle)
				trackRot.DoRotY(self.pi/2)
				trackRot.DoRotZ(showAngle)
				trackRot.DoRotY(y_rot)
				trackRot.DoRotY(x_rot)
				# build quaternion
				trackQuaternion = Quaternion(trackRot.GetQuaternion()[0],trackRot.GetQuaternion()[1],trackRot.GetQuaternion()[2],trackRot.GetQuaternion()[3])

				trackMat = self.quaternion_to_matrix(trackRot.GetQuaternion())

				#build position with translation along z axis
				trackPoint = Point (-trackMat[0][2]*z_dist+trackMat[0][1]*y_dist+trackMat[0][0]*x_dist+postTrans[0][3],
									-trackMat[1][2]*z_dist+trackMat[1][1]*y_dist+trackMat[1][0]*x_dist+postTrans[1][3]+self.robotsDistance,
									-trackMat[2][2]*z_dist+trackMat[2][1]*y_dist+trackMat[2][0]*x_dist+postTrans[2][3])

				if self.robot.track.isIKSolutionExist(Pose(trackPoint,trackQuaternion)):
					break
				angle+=0.01
		#rotate
		trackRot = PyKDL.Rotation(postTrans[0][0],postTrans[0][1],postTrans[0][2],	postTrans[1][0],postTrans[1][1],postTrans[1][2],	postTrans[2][0],postTrans[2][1],postTrans[2][2])	
		trackRot.DoRotZ(angle)
		trackRot.DoRotY(self.pi/2)
		trackRot.DoRotZ(showAngle)
		trackRot.DoRotY(y_rot)
		trackRot.DoRotY(x_rot)
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

	def getCubeFromConveyorToPostument(self):
		self.robot.postument.moveToJointPosition([0.0, -1.57, 0.0, 0.0, 4.71, 0.0],self.simulate)
		
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		self.robot.postument.moveRelativeToCartesianPosition(z_tran=0.05)
		
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors!=None:
			time.sleep(0.5)
		while not hasattr(self.cubeFaceColors,'x'):
			time.sleep(0.5)
		
		print self.cubeFaceColors.x/1000-0.012
		print self.cubeFaceColors.y/1000-0.059
		
		self.robot.postument.moveRelativeToCartesianPosition(x_tran=-(self.cubeFaceColors.y/1000-0.059),y_tran=(self.cubeFaceColors.x/1000))
		self.robot.postument.moveRelativeToJointPosition([0,0,0,0,0,self.cubeFaceColors.z_rot])
		
		self.robot.postument.moveRelativeToCartesianPosition(z_tran=0.13)
		
		self.robot.postument.tfgToJointPosition(position=0.068,simulate=self.simulate)

		self.robot.postument.startForceControl(tran_z=True,mov_z=0.013)
		time.sleep(5)
		self.robot.postument.stopForceControl()

		self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		
		self.robot.postument.moveRelativeToCartesianPosition(z_tran=-0.13)
		
		self.robot.postument.attachItem(self.cube)

	def getCubeFromHumanToTrack(self):
		#self.robot.track.moveToJointPosition([0.549969, 0.9265588068203238, -1.5550934102940197, 0.04871716017958537, 1.3063556317588827, 2.6762106103323573, -0.13097870260389663],self.simulate)
		#Preparing arm and gripper to get cube
		self.robot.track.moveToCartesianPosition(Pose(Point(1,1,1.18),Quaternion(-0.441448339193, 0.440445333852, -0.552396228639, 0.55316333781)),self.simulate)
		self.robot.track.tfgToJointPosition(position=0.064,simulate=self.simulate)
		
		#Force controll to approach the cube
		self.robot.track.startForceControl(tran_z=True,mov_z=0.013)
		gotCube=False
		for i in range(1,50):
			reading = self.robot.track.getForceReadings()
			time.sleep(0.1)
			#if reading force is enough cube is obtained
			print reading.force.z
			if reading.force.z > 8:
				gotCube=True
				break
		self.robot.track.stopForceControl()
		
		#closing gripper
		if gotCube:
			self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		else:
			self.robot.track.moveToCartesianPosition(Pose(Point(1,1,1.18),Quaternion(-0.441448339193, 0.440445333852, -0.552396228639, 0.55316333781)),self.simulate)

		self.robot.track.attachItem(self.cube)
		return gotCube

	def getCubeFromHumanToPostument(self):
		#self.robot.track.moveToJointPosition([1, 0.0, -1.4476547584053457, 0.012313341484619551, 1.2106388401258297, 1.57, 0],self.simulate)
		#Preparing arm and gripper to get cube
		self.robot.postument.moveToCartesianPosition(Pose(Point(0.89668151225, 0.10325876098, 1.32973306404),Quaternion(-0.247541182161, 0.554930714941, -0.682010199583, 0.406985690674)),self.simulate)
		self.robot.postument.tfgToJointPosition(position=0.064,simulate=self.simulate)
		
		#Force controll to approach the cube
		self.robot.postument.startForceControl(tran_z=True,mov_z=0.013)
		gotCube=False
		for i in range(1,50):
			reading = self.robot.postument.getForceReadings()
			time.sleep(0.1)
			#if reading force is enough cube is obtained
			if reading.force.z > 8:
				gotCube=True
				break
		self.robot.postument.stopForceControl()
		
		#closing gripper
		if gotCube:
			self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.postument.attachItem(self.cube)
		return gotCube
		
	def getCubeFromTrackToPostument(self,angle=0):
		if angle==0:
			angle=self.pi/2
			fcx=self.fcx
			fcy=self.fcy
		else:
			angle=-self.pi/2
			fcx=self.fcx
			fcy=self.fcy
		print fcx
		print fcy
		
		#Preparing postument to give cube
		self.robot.postument.moveToJointPosition([-2.25, -1.44, 0.1, 0.10, 0.75, -1.91],self.simulate)
		
		#Tracks first approach to cube
		self.facePostument(z_dist=0.15,x_dist=fcx,y_dist=fcy,showAngle=angle)
		
		self.robot.track.releaseItem(self.cube)
		
		#Postument opens gripper
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)

		#Track comes nearer
		self.facePostument(z_dist=0.005,x_dist=fcx,y_dist=fcy,showAngle=angle)

		#Postument tights gripper but still open
		self.robot.track.startForceControl(tran_x=True,tran_y=True)
		self.robot.postument.tfgToJointPosition(position=0.068,simulate=self.simulate)
		self.robot.track.stopForceControl()

		#Force controll to track approach to the end
		self.robot.track.startForceControl(tran_z=True,mov_z=0.013)
		self.robot.postument.startForceControl(rot_x=True,rot_y=True)
		time.sleep(5)
		self.robot.postument.stopForceControl()
		self.robot.track.stopForceControl()
	
		#Postument closes gripper and Track opens
		self.robot.track.startForceControl(tran_x=True,tran_y=True)
		self.robot.postument.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.track.stopForceControl()
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		#Track is moving back
		self.facePostument(z_dist=0.25,x_dist=fcx,y_dist=fcy,showAngle=angle)
		self.robot.postument.attachItem(self.cube)
		
	def getCubeFromPostumentToTrackSide(self,side=0):
		#self.sidePostument(z_dist=0.15,y_dist=self.spcy1,x_dist=self.spcx1,angle=-self.pi/4,showAngle=self.pi/4)
		#Parameters correcting position of gripping used in 2 function calls sidePostument
		if side % 2 ==0:
			sAngle=self.pi/4
			posCorrectionX=self.spcx1
			posCorrectionY=self.spcy1
		else:
			sAngle=-self.pi/4
			posCorrectionX=self.spcx2
			posCorrectionY=self.spcy2
		distanceZ=0.02
		a=-self.pi/4-side*self.pi/2
		
		#Preparing postument to give cube
		#if side<2:
		#	self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -1.91],self.simulate)
		#else:
		#	self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, 1.24],self.simulate)
	
		self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -2.01+side*self.pi/2],self.simulate)
			
		#Tracks first approach to cube
		self.sidePostument(z_dist=0.15,y_dist=posCorrectionY,x_dist=posCorrectionX,showAngle=sAngle,angle=a)
		
		#Track opens gripper
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		self.robot.postument.releaseItem(self.cube)
		
		#Track comes nearer
		self.sidePostument(z_dist=distanceZ,y_dist=posCorrectionY,x_dist=posCorrectionX,showAngle=sAngle,angle=a)
		self.robot.track.startForceControl(tran_z=True,mov_z=0.010)
		time.sleep(2)
		self.robot.track.stopForceControl()
		
		#Track tights gripper but still open
		self.robot.postument.startForceControl(tran_x=True,tran_y=True)
		self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		
		self.robot.track.tfgToJointPosition(position=0.065,simulate=self.simulate)
		
		#Force controll to track approach to the end
		self.robot.track.startForceControl(tran_x=True,tran_y=True,tran_z=True,mov_z=0.010)
		#self.robot.postument.startForceControl(rot_x=True,rot_y=True)
		time.sleep(5)
		#self.robot.postument.stopForceControl()
		self.robot.track.stopForceControl()
		
		#Track closes gripper and Postument opens
		self.robot.postument.startForceControl(tran_x=True,tran_z=True)
		self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		self.robot.postument.startForceControl(tran_z=True,mov_z=-0.023)
		time.sleep(4)
		self.robot.postument.stopForceControl()
		
		self.robot.track.startForceControl(tran_z=True,mov_z=-0.023)
		time.sleep(4)
		self.robot.track.stopForceControl()
		
		
		self.robot.track.attachItem(self.cube)
		
	def getCubeFromPostumentToTrack(self,angle=0):
		if angle==0:
			angle=self.pi/2
		else:
			angle=-self.pi/2
		
		#Preparing postument to give cube
		self.robot.postument.moveToJointPosition([-2.25, -1.44, 0.1, 0.10, 0.75, -1.91],self.simulate)
		
		#Tracks first approach to cube
		self.facePostument(z_dist=0.15,x_dist=self.fcx,y_dist=self.fcy,showAngle=angle)
		
		#Track opens gripper
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		self.robot.postument.releaseItem(self.cube)
		
		#Track comes nearer
		#self.postument.moveRelativeToCartesianPosition(z_tran=0.15-0.005)
		self.facePostument(z_dist=0.005,x_dist=self.fcx,y_dist=self.fcy,showAngle=angle)
		
		#Track tights gripper but still open
		self.robot.postument.startForceControl(tran_x=True,tran_y=True)
		self.robot.track.tfgToJointPosition(position=0.068,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		
		#Force controll to track approach to the end
		self.robot.track.startForceControl(tran_z=True,mov_z=0.013)
		self.robot.postument.startForceControl(rot_x=True,rot_y=True)
		time.sleep(5)
		self.robot.postument.stopForceControl()
		self.robot.track.stopForceControl()
		
		#Track closes gripper and Postument opens
		self.robot.postument.startForceControl(tran_x=True,tran_y=True)
		self.robot.track.tfgToJointPosition(position=self.gripJoint,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		self.robot.postument.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		#Track is moving back
		self.facePostument(z_dist=0.15,x_dist=self.fcx,y_dist=self.fcy,showAngle=angle)
		
		self.robot.track.attachItem(self.cube)
		
		
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
		self.robot.track.releaseItem(self.cube)
	def giveBackCubePostument(self,throw=False):
		self.robot.postument.moveToJointPosition([0.0, -1.57, 0.0, 0.0, 4.71, 0.0],self.simulate)
		if throw==False:
			self.robot.postument.moveRelativeToCartesianPosition(z_tran=0.18,y_tran=0.08)
		self.robot.postument.tfgToJointPosition(position=self.openJoint,simulate=self.simulate)
		if throw==False:
			self.robot.postument.moveRelativeToCartesianPosition(z_tran=-0.17)
		self.robot.postument.releaseItem(self.cube)
		
	def flipCubePostument(self):
		self.getCubeFromPostumentToTrackSide(side=0)
		
		self.getCubeFromTrackToPostument(angle=0)
		
		self.getCubeFromPostumentToTrackSide(side=0)
		
		self.getCubeFromTrackToPostument(angle=0)
		
	def flipCubeTrack(self):
		
		self.getCubeFromTrackToPostument(angle=0)
		
		self.getCubeFromPostumentToTrackSide(side=2)
		self.robot.track.moveRelativeToCartesianPosition(z_tran=-0.15)
		
		self.getCubeFromTrackToPostument(angle=0)
		
		self.getCubeFromPostumentToTrackSide(side=2)
		self.robot.track.moveRelativeToCartesianPosition(z_tran=-0.15)
	#
	#
	#	Watching cube
	#
	#
	def setCorrectionFace(self):
		self.fcx=0;
		self.fcy=0;
		self.showCubeFaceToPostument(side=9)
		
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors!=None:
			time.sleep(0.5)
		while not hasattr(self.cubeFaceColors,'x'):
			time.sleep(0.5)
			
		self.fcy = (self.cubeFaceColors.y/1000)-0.000
		self.fcx = (-self.cubeFaceColors.x/1000)+0.005

	def setCorrectionSide(self):
		self.spcx1=0;
		self.spcy1=0;
		self.showCubeFaceToPostument(side=8)
		
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors!=None:
			time.sleep(0.5)
		while not hasattr(self.cubeFaceColors,'x'):
			time.sleep(0.5)

		self.spcy1 = (-self.cubeFaceColors.x/1000)+0.003
		self.spcx1 = (self.cubeFaceColors.y/1000)+0.002
		
		self.spcx2=0;
		self.spcy2=0;
		self.showCubeFaceToPostument(side=7)
		
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors!=None:
			time.sleep(0.5)
		while not hasattr(self.cubeFaceColors,'x'):
			time.sleep(0.5)

		self.spcy2 = (-self.cubeFaceColors.x/1000)
		self.spcx2 = (self.cubeFaceColors.y/1000)+0.004
		

	def setCorrectionSide2(self):
		self.spcx2=0;
		self.spcy2=0;
		self.showCubeFaceToPostument(side=7)
		
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors!=None:
			time.sleep(0.5)
		while not hasattr(self.cubeFaceColors,'x'):
			time.sleep(0.5)

		self.spcy2 = (-self.cubeFaceColors.x/1000)+0.003
		self.spcx2 = (self.cubeFaceColors.y/1000)+0.002
		print "[Liczba na dzis]"
		print self.spcx2
		print self.spcy2
	def recieveCubeColors(self,data):
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
		self.cubeFaceColors=data
	def getCubeFaceColors(self):
		return self.cubeFaceColors	
	def showCubeFaceToPostument(self,side=0):
		if side==9 or side==0:
			self.robot.postument.moveToJointPosition([-2.25, -1.44, 0.1, 0.10, 0.75, -1.91],self.simulate)
		elif side==8: 
			self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -2.01],self.simulate)
		elif side==7:
			self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -2.01+self.pi/2],self.simulate)
		else:
			#self.robot.postument.moveToJointPosition([-2.15, -1.64, -0.3, 0.00, 5.2, -3.48+side*self.pi/2],self.simulate)
			self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -3.48+side*self.pi/2],self.simulate)
			
		if side==9: # watch for catching cube from front
			self.facePostument(z_dist=0.1,x_dist=0.002+self.fcx,y_dist=0.052+self.fcy,showAngle=self.pi/2)
		elif side==8: # -- || --  from side
			self.sidePostument(z_dist=0.1,x_dist=0.06+self.spcx1,y_dist=-0.0+self.spcy1,angle=-self.pi/4,showAngle=self.pi/4)
		elif side==7: # -- || --  from other side
			self.sidePostument(z_dist=0.1,x_dist=0.055+self.spcx2,y_dist=-0.0+self.spcy2,angle=-3*self.pi/4,showAngle=-self.pi/4)
		elif side==0:  # watching front side of cube
			self.facePostument(z_dist=0.1,x_dist=0.002+self.fcx,y_dist=0.062+self.fcy,showAngle=self.pi/2+self.pi/4-self.pi/8)
		elif side==1: # watching side of cube
			self.sidePostument(z_dist=0.1,x_dist=0.06+self.spcx1,y_dist=-0.0+self.spcy1,angle=-1*self.pi/4,showAngle=self.pi/2-self.pi/8)
		elif side==2:
			self.sidePostument(z_dist=0.1,x_dist=0.06+self.spcx1,y_dist=-0.0+self.spcy1,angle=-3*self.pi/4,showAngle=self.pi/2-self.pi/8)
		elif side==3:
			self.sidePostument(z_dist=0.1,x_dist=0.06+self.spcx1,y_dist=-0.0+self.spcy1,angle=-5*self.pi/4,showAngle=self.pi/2-self.pi/8)
#		elif side==4:
#			self.sidePostument(z_dist=0.1,x_dist=0.06+self.spcx1,y_dist=-0.0+self.spcy1,angle=-7*self.pi/4,showAngle=self.pi/2-self.pi/8)

	def showCubeFaceToPostument2(self,side=0):
		if side==9 or side==0:
			self.robot.postument.moveToJointPosition([-0.75, -1.44, 0.1, 0.10, 2.5, -1.91],self.simulate)
		elif side==8: 
			self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -1.91],self.simulate)
		else:
			self.robot.postument.moveToJointPosition([-1.1, -1.54, 0.1, 0.70, 1.0, -4.1+side*self.pi/2],self.simulate)
			#self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -3.48+side*self.pi/2],self.simulate)
		
		if side==0:  # watching front side of cube
			self.facePostument(z_dist=0.1,x_dist=-0.02+self.fcx,y_dist=0.05+self.fcy,showAngle=self.pi/2+self.pi/4-self.pi/8)	

	def fillCubeColor(self,side=0):
		time.sleep(3)
		self.cubeFaceColors = None
		while self.cubeFaceColors is None:
			time.sleep(0.5)
		if side==0:
			self.fillTileColor(0,0,self.cubeFaceColors.tile1)
			self.fillTileColor(0,1,self.cubeFaceColors.tile2)
			self.fillTileColor(0,2,self.cubeFaceColors.tile3)
			self.fillTileColor(0,3,self.cubeFaceColors.tile4)
			self.fillTileColor(0,4,self.cubeFaceColors.tile5)
			self.fillTileColor(0,5,self.cubeFaceColors.tile6)
			self.fillTileColor(0,6,self.cubeFaceColors.tile7)
			self.fillTileColor(0,7,self.cubeFaceColors.tile8)
			self.fillTileColor(0,8,self.cubeFaceColors.tile9)
		if side==5:
			self.fillTileColor(5,0,self.cubeFaceColors.tile1)
			self.fillTileColor(5,1,self.cubeFaceColors.tile2)
			self.fillTileColor(5,2,self.cubeFaceColors.tile3)
			self.fillTileColor(5,3,self.cubeFaceColors.tile4)
			self.fillTileColor(5,4,self.cubeFaceColors.tile5)
			self.fillTileColor(5,5,self.cubeFaceColors.tile6)
			self.fillTileColor(5,6,self.cubeFaceColors.tile7)
			self.fillTileColor(5,7,self.cubeFaceColors.tile8)
			self.fillTileColor(5,8,self.cubeFaceColors.tile9)
		if side==1 or side==3:
			self.fillTileColor(side,0,self.cubeFaceColors.tile1)
			self.fillTileColor(side,3,self.cubeFaceColors.tile4)
			self.fillTileColor(side,4,self.cubeFaceColors.tile5)
			self.fillTileColor(side,5,self.cubeFaceColors.tile6)
			self.fillTileColor(side,6,self.cubeFaceColors.tile7)
			self.fillTileColor(side,7,self.cubeFaceColors.tile8)
			self.fillTileColor(side,8,self.cubeFaceColors.tile9)
		if side==2 or side==4:
			self.fillTileColor(side,0,self.cubeFaceColors.tile1)
			self.fillTileColor(side,1,self.cubeFaceColors.tile2)
			self.fillTileColor(side,2,self.cubeFaceColors.tile3)
			self.fillTileColor(side,3,self.cubeFaceColors.tile4)
			self.fillTileColor(side,4,self.cubeFaceColors.tile5)
			self.fillTileColor(side,5,self.cubeFaceColors.tile6)
			self.fillTileColor(side,6,self.cubeFaceColors.tile7)
		if side==6 or side==8:
			self.fillTileColor((side-5),1,self.cubeFaceColors.tile8)
			self.fillTileColor((side-5),2,self.cubeFaceColors.tile7)
		if side==7 or side==9:
			self.fillTileColor((side-5),7,self.cubeFaceColors.tile2)
			self.fillTileColor((side-5),8,self.cubeFaceColors.tile1)
	
	def fillTileColor(self,f,t,color):
		#print str(color.red) + "  " + str(color.green) + "  " + str(color.blue)
		self.cubeColors.setSampleColor(f,t,color.red,color.green,color.blue)
		
	def determineColors(self):
		for l in range(0,6):
			firstSampleIdx=0
			maxDistance=-1
			for i in range(0,54):
				sampleColor = self.cubeColors.getColor(i);
				distance = math.sqrt( (sampleColor.red)*(sampleColor.red) + (sampleColor.green)*(sampleColor.green) + (sampleColor.blue)*(sampleColor.blue) )
				if maxDistance<distance and sampleColor.number==-1:
					maxDistance=distance
					firstSampleIdx=i
			
			self.cubeColors.setColor(firstSampleIdx,l);
			samplefirstColor = self.cubeColors.getColor(firstSampleIdx);
			
			restColorDistance = [999999,999999,999999,999999,999999,999999,999999,999999]
			restColorIdx = [-1, -1, -1, -1, -1, -1,-1,-1]
			
			for i in range(0,54):
				sampleColor = self.cubeColors.getColor(i)
				if sampleColor.number!=-1:
					continue
					
				distance = self.cubeColors.calculateDistance(sampleColor,samplefirstColor)
				
				maxDistance=0
				furthestIdx=0
				for j in range(0,8):
					if maxDistance < restColorDistance[j] or restColorIdx[j]==-1:
						furthestIdx = j
						maxDistance = restColorDistance[j]
				
				if restColorDistance[furthestIdx] > distance:
					restColorDistance[furthestIdx] = distance
					restColorIdx[furthestIdx] = i
			for j in range(0,8):
				self.cubeColors.setColor(restColorIdx[j],l)
					
		
		#print numerki pol
		for i in range(0,6):
			for j in range(0,3):
				print str(self.printColor(self.cubeColors.getTile(i,j*3+2).number)) + " " + str(self.printColor(self.cubeColors.getTile(i,j*3+1).number)) + " " + str(self.printColor(self.cubeColors.getTile(i,j*3).number))
			print " " 
			
		#print kolorki pol
		for i in range(0,6):
			for j in range(0,3):
				print str(self.cubeColors.getTile(i,j*3).red) +"," + str(self.cubeColors.getTile(i,j*3).green) + "," + str(self.cubeColors.getTile(i,j*3).blue) + "  " + str(self.cubeColors.getTile(i,j*3+1).red) +"," + str(self.cubeColors.getTile(i,j*3+1).green) + "," + str(self.cubeColors.getTile(i,j*3+1).blue) + "  " + str(self.cubeColors.getTile(i,j*3+2).red) +"," + str(self.cubeColors.getTile(i,j*3+2).green) + "," + str(self.cubeColors.getTile(i,j*3+2).blue)
			print " " 
			
		for i in range(0,54):
			print str(self.cubeColors.getColor(i).red) +"," + str(self.cubeColors.getColor(i).green) + "," + str(self.cubeColors.getColor(i).blue)
			
	def printColor(self,n):
		#return str(n)
		if n==0:
			return "white"
		if n==1:
			return "yellow"
		if n==2:
			return "orange"
		if n==3:
			return "red"
		if n==4:
			return "green"
		if n==5:
			return "blue" 

	#
	#
	#	SOLVING CUBE
	#
	#
				
	def rotateCube(self,angle):
		fcx=self.fcx
		fcy=self.fcy
		
		#Preparing postument to give cube
		self.robot.postument.moveToJointPosition([-2.25, -1.44, 0.1, 0.10, 0.75, -1.91],self.simulate)
		
		#Tracks first approach to cube
		self.facePostument(z_dist=0.15,x_dist=fcx,y_dist=fcy,showAngle=0)
		
		self.robot.postument.releaseItem(self.cube)
		
		#Track opens gripper
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)

		#Track comes nearer
		self.facePostument(z_dist=0.01,x_dist=fcx,y_dist=fcy,showAngle=0)
		
		self.robot.postument.startForceControl(tran_x=True,tran_y=True)
		self.robot.track.tfgToJointPosition(position=0.060,simulate=self.simulate)
		self.robot.postument.stopForceControl()
		
		
		if angle==90:
			position = (self.robot.track.getJointPosition())[6]
			self.robot.track.startForceControl(rot_z=True,force_z=0.2,value=0.2)
			while math.fabs(position-(self.robot.track.getJointPosition())[6])<1.57:
				time.sleep(0.1)
			self.robot.track.stopForceControl()	
		elif angle==-90:
			position = (self.robot.track.getJointPosition())[6]
			self.robot.track.startForceControl(rot_z=True,force_z=-0.2,value=0.2)
			while math.fabs(position-(self.robot.track.getJointPosition())[6])<1.57:
				time.sleep(0.1)
			self.robot.track.stopForceControl()	
		elif angle==180:
			position = (self.robot.track.getJointPosition())[6]
			self.robot.track.startForceControl(rot_z=True,force_z=0.2,value=0.2)
			while math.fabs(position-(self.robot.track.getJointPosition())[6])<3.14:
				time.sleep(0.1)
			self.robot.track.stopForceControl()	
		
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		self.facePostument(z_dist=0.20,x_dist=fcx,y_dist=fcy,showAngle=0)
		
		self.robot.track.attachItem(self.cube)
	
	def findSolution(self,cubestate=None):
	
		if cubestate==None:
			cubestate = cube_convert.cube_convert(self.cubeColors)
		
		#self.giveBackCubePostument()
		
		currentpath = os.getcwd()
		
		path = os.path.dirname(sys.argv[0])
		print path
		os.chdir(path)
		cmd = ["./solver", cubestate]
		p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
		out, err = p.communicate()

		listOfStrings = out.split()

		return listOfStrings
	
	#
	#
	#	THE FINAL METHOD!!!!!!!!
	#
	#
	def watchCubeTrack(self):
		gotCube = self.getCubeFromHumanToPostument()
		if not gotCube:
			print "Where is the cube?"
			return
		
		
		self.showCubeFaceToPostument2(side=0)
		self.fillCubeColor(0)
		
		
		self.showCubeFaceToPostument2(side=1)
		self.fillCubeColor(1)
		self.showCubeFaceToPostument2(side=2)
		self.fillCubeColor(2)
		self.showCubeFaceToPostument2(side=3)
		self.fillCubeColor(3)
		self.showCubeFaceToPostument2(side=4)
		self.fillCubeColor(4)
		
		self.setCorrectionFace()
		self.setCorrectionSide()
		
		self.flipCubePostument()
		
		self.showCubeFaceToPostument2(side=0)
		self.fillCubeColor(5)
		
		self.showCubeFaceToPostument2(side=1)
		self.fillCubeColor(8)
		self.showCubeFaceToPostument2(side=2)
		self.fillCubeColor(7)
		self.showCubeFaceToPostument2(side=3)
		self.fillCubeColor(6)
		self.showCubeFaceToPostument2(side=4)
		self.fillCubeColor(9)
		
		self.determineColors()
		
		self.giveBackCubePostument()
		
		self.moveToSynchroPosition()

	def solveRubikCube(self):
		
		gotCube = self.getCubeFromHumanToPostument()
		if not gotCube:
			print "Where is the cube?"
			return
		
		self.showCubeFaceToPostument2(side=0)
		self.fillCubeColor(0)
		
		
		self.showCubeFaceToPostument2(side=1)
		self.fillCubeColor(1)
		self.showCubeFaceToPostument2(side=2)
		self.fillCubeColor(2)
		self.showCubeFaceToPostument2(side=3)
		self.fillCubeColor(3)
		self.showCubeFaceToPostument2(side=4)
		self.fillCubeColor(4)
		
		self.setCorrectionFace()
		self.setCorrectionSide()
		
		self.flipCubePostument()
		
		self.showCubeFaceToPostument2(side=0)
		self.fillCubeColor(5)
		
		self.showCubeFaceToPostument2(side=1)
		self.fillCubeColor(8)
		self.showCubeFaceToPostument2(side=2)
		self.fillCubeColor(7)
		self.showCubeFaceToPostument2(side=3)
		self.fillCubeColor(6)
		self.showCubeFaceToPostument2(side=4)
		self.fillCubeColor(9)
		
		self.determineColors()

		solution = self.findSolution()

		#solution = self.findSolution("nybonnnnnyongynnnobnnnynngynnnnnrybnnnrynngnnnyrnnnbnognnnnonnrnbnnrngnnnnnbwonnnnwowognnnbnwnnnwgnnnnwnrnbnnrnnnwnrngnw")
	
		cubeInHand = ['D','B','R','T','L','F']
		print solution
		print cubeInHand
		for i in solution:
			if len(i)<2:
				print "Rotate " + i[0] + " 90"
				s = i[0]
				d=90
			elif i[0]=='2':
				print "Rotate " + i[1] + " 180"
				s = i[1]
				d=180
			else:
				print "Rotate " + i[0] + " -90"
				s = i[0]
				d=-90
			current_side=0
			for j in range(0,6):
				if cubeInHand[j]==s:
					current_side=j
					break;
			print current_side
			if current_side==2:
				self.getCubeFromPostumentToTrackSide(side=3)
				self.getCubeFromTrackToPostument(angle=180)
				tmp = [' ',' ',' ',' ',' ',' ']
				tmp[0]=cubeInHand[2]; tmp[1]=cubeInHand[1]; tmp[2]=cubeInHand[3]; tmp[3]=cubeInHand[4]; tmp[4]=cubeInHand[0]; tmp[5]=cubeInHand[5]
				cubeInHand=tmp
			elif current_side==5:
				self.getCubeFromPostumentToTrackSide(side=2)
				self.getCubeFromTrackToPostument(angle=180)
				tmp = [' ',' ',' ',' ',' ',' ']
				tmp[0]=cubeInHand[5]; tmp[1]=cubeInHand[3]; tmp[2]=cubeInHand[4]; tmp[3]=cubeInHand[1]; tmp[4]=cubeInHand[2]; tmp[5]=cubeInHand[0]
				cubeInHand=tmp
			elif current_side==3:
				self.flipCubePostument()
				tmp = [' ',' ',' ',' ',' ',' ']
				tmp[0]=cubeInHand[3]; tmp[1]=cubeInHand[1]; tmp[2]=cubeInHand[4]; tmp[3]=cubeInHand[0]; tmp[4]=cubeInHand[2]; tmp[5]=cubeInHand[5]
				cubeInHand=tmp
			elif current_side==1:
				self.getCubeFromPostumentToTrackSide(side=0)
				self.getCubeFromTrackToPostument(angle=180)
				tmp = [' ',' ',' ',' ',' ',' ']
				tmp[0]=cubeInHand[1]; tmp[1]=cubeInHand[3]; tmp[2]=cubeInHand[2]; tmp[3]=cubeInHand[5]; tmp[4]=cubeInHand[4]; tmp[5]=cubeInHand[0]
				cubeInHand=tmp	
			elif current_side==4:
				self.getCubeFromPostumentToTrackSide(side=1)
				self.getCubeFromTrackToPostument(angle=180)
				tmp = [' ',' ',' ',' ',' ',' ']
				tmp[0]=cubeInHand[4]; tmp[1]=cubeInHand[5]; tmp[2]=cubeInHand[3]; tmp[3]=cubeInHand[2]; tmp[4]=cubeInHand[0]; tmp[5]=cubeInHand[0]
				cubeInHand=tmp
			self.rotateCube(d)
			print cubeInHand
		self.giveBackCubePostument()
		
		self.moveToSynchroPosition()
		
	#
	#
	#	Testing
	#
	#

	def faceRobotTest(self):
		self.robot.track.tfgToJointPosition(position=0.068,simulate=self.simulate)
		self.robot.postument.moveToJointPosition([-2.25, -1.44, 0.1, 0.10, 0.75, -1.91],self.simulate)
		self.facePostument(z_dist=0.15,x_dist=self.fcx,y_dist=self.fcy,showAngle=self.pi/2)
		self.facePostument(z_dist=0.03,x_dist=self.fcx,y_dist=self.fcy,showAngle=self.pi/2)
		time.sleep(20)
		self.facePostument(z_dist=0.15,x_dist=self.fcx,y_dist=self.fcy,showAngle=self.pi/2)

	def sidePostumentRobotTest1(self):
		self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -2.01],self.simulate)
		self.robot.track.tfgToJointPosition(position=0.068,simulate=self.simulate)
		
		self.sidePostument(z_dist=0.15,y_dist=self.spcy1,x_dist=self.spcx1,angle=-self.pi/4,showAngle=self.pi/4)
		self.sidePostument(z_dist=0.03,y_dist=self.spcy1,x_dist=self.spcx1,angle=-self.pi/4,showAngle=self.pi/4)
		time.sleep(20)
		self.sidePostument(z_dist=0.15,y_dist=self.spcy1,x_dist=self.spcx1,angle=-self.pi/4,showAngle=self.pi/4)

	def sidePostumentRobotTest2(self):
		self.robot.postument.moveToJointPosition([-2.05, -1.34, 0.0, 0.10, 0.45, -2.01+1.57],self.simulate)
		self.robot.track.tfgToJointPosition(position=self.wideOpenJoint,simulate=self.simulate)
		
		#self.sidePostument(z_dist=0.15,y_dist=self.spcy2,x_dist=self.spcx2,showAngle=-self.pi/4,angle=-3*self.pi/4)
		self.sidePostument(z_dist=0.03,y_dist=self.spcy2,x_dist=self.spcx2,showAngle=-self.pi/4,angle=-3*self.pi/4)
		time.sleep(20)
		#self.sidePostument(z_dist=0.15,y_dist=self.spcy2,x_dist=self.spcx2,showAngle=-self.pi/4,angle=-3*self.pi/4)

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
