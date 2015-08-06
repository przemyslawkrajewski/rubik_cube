#!/usr/bin/env python

import rospy
import tf
import actionlib

import time

import openrave_irp6
from irpos import *

import rubikCubeManipulator

from rubik_cube.msg import *

#MAIN

def talker():
	pub = rospy.Publisher('chatter', Cube_face_color, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		message= Cube_face_color
		tile = Tile_color(3,2,14)
		msg = Cube_face_color(tile,tile,tile,tile,tile,tile,tile,tile,tile)
		"""tile.red=3
		tile.green=3
		tile.blue=3
		message.tile1 = tile
		message.tile2 = tile
		message.tile3 = tile
		message.tile4 = tile
		message.tile5 = tile
		message.tile6 = tile
		message.tile7 = tile"""

		#tile.data = "hello world %s" % rospy.get_time()
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
