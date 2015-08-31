#!/usr/bin/env python

import math
import colorsys

class TileColor:
	red=0
	blue=0
	green=0
	number=-1
	
class ColorState:
	tile = []

	def __init__(self):
		for i in range(0,54):
				self.tile.append(i) 
	def getTile(self,f,t):
		return self.tile[f*9+t]
	def setTileNumber(self,face,tile,n):
		self.tile[face*9+tile].number=n
	def getColor(self,n):
		return self.tile[n]
	def rotate(self,s):
		return
	def printCube(self):
		for i in range(0,6):
			for j in range(0,3):
				print str(self.getTile(i,j*3+0)) + "	" + str(self.getTile(i,j*3+1)) + "	" + str(self.getTile(i,j*3+2))
			print " " 

c = ColorState()
c.printCube()
