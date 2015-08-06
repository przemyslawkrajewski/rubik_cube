#!/usr/bin/env python

import math

"""class RubikCubeTileColors:
	red=0
	green=0
	blue=0
	number=0

class RubikCubeFaceColors:
	tile = [RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors, RubikCubeTileColors]

class RubikCubeColors:
	face = [RubikCubeFaceColors, RubikCubeFaceColors, RubikCubeFaceColors, RubikCubeFaceColors, RubikCubeFaceColors, RubikCubeFaceColors]"""

class TileColor:
	red=0
	blue=0
	green=0
	number=-1
	
class CubeColor:
	tile = []

	def __init__(self):
		for j in range(0,54):
			t = TileColor()
			self.tile.append(t)
	def getTile(self,f,t):
		return self.tile[f*9+t]
	def setTileNumber(self,face,tile,n):
		self.tile[face*9+tile].number=n
	def getColor(self,n):
		return self.tile[n]
	def setColor(self,n,v):
		self.tile[n].number=v
	def setSampleColor(self,face,tile,r,g,b):
		self.tile[face*9+tile].red=r
		self.tile[face*9+tile].green=g
		self.tile[face*9+tile].blue=b
	def calculateDistance(self,tile1,tile2):
		return math.sqrt( (tile1.red-tile2.red)*(tile1.red-tile2.red) + (tile1.green-tile2.green)*(tile1.green-tile2.green) + (tile1.blue-tile2.blue)*(tile1.blue-tile2.blue) )
