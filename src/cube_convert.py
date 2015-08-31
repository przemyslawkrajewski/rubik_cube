#!/usr/bin/env python

import sys
from RubikCube import *

def cube_convert(Cube):
	# Prepending a character because I used the wrong indexes below and
	# I'm not correcting it
	l = [2,1,0,5,3,8,7,6,18,21,24,9,12,15,36,39,42,27,30,33,19,25,10,16,37,43,28,34,20,23,26,11,14,17,38,41,44,29,32,35,47,46,45,50,48,53,52,51]
	
	colors = [0,0,0,0,0,0]
	
	colors[Cube.tile[4].number]='r'
	colors[Cube.tile[31].number]='y'
	colors[Cube.tile[22].number]='b'
	colors[Cube.tile[40].number]='g'
	colors[Cube.tile[13].number]='w'
	colors[Cube.tile[49].number]='o'
	cc=''
	for i in l:
		cc = cc+colors[Cube.tile[i].number]
	print cc
	cc = "@" + cc.strip().lower()
		
	if len(cc) != 49:
		print "Bad input"
		sys.exit(1)

	# Now for some painful processing

	cubies = []

	# the cubie in position 1
	cubies.append("nn%s%s%sn" % (cc[29],cc[40],cc[46]))
	# 2
	cubies.append("nnn%s%sn" % (cc[39],cc[47]))
	# 3
	cubies.append("nnn%s%s%s" % (cc[38],cc[48],cc[37]))
	# 4
	cubies.append("nn%s%snn" % (cc[21],cc[28]))
	# 5
	cubies.append("nnn%sn%s" % (cc[27],cc[26]))
	# 6
	cubies.append("n%s%s%snn" % (cc[1],cc[9],cc[20]))
	# 7
	cubies.append("n%sn%snn" % (cc[2],cc[19]))
	# 8
	cubies.append("n%sn%sn%s" % (cc[3],cc[18],cc[17]))
	# 9
	cubies.append("nn%sn%sn" % (cc[30],cc[44]))
	# 10
	cubies.append("nnnn%s%s" % (cc[45],cc[36]))
	# 11
	cubies.append("n%s%snnn" % (cc[4],cc[10]))
	# 12
	cubies.append("n%snnn%s" % (cc[5],cc[16]))
	# 13
	cubies.append("%sn%sn%sn" % (cc[32],cc[31],cc[41]))
	# 14
	cubies.append("%snnn%sn" % (cc[33],cc[42]))
	# 15
	cubies.append("%snnn%s%s" % (cc[34],cc[43],cc[35]))
	# 16
	cubies.append("%sn%snnn" % (cc[23],cc[22]))
	# 17
	cubies.append("%snnnn%s" % (cc[24],cc[25]))
	# 18
	cubies.append("%s%s%snnn" % (cc[12],cc[6],cc[11]))
	# 19
	cubies.append("%s%snnnn" % (cc[13],cc[7]))
	# 20
	cubies.append("%s%snnn%s" % (cc[14],cc[8],cc[15]))

	# Now that we have all the cubies, put them in order
	fc = ""
	try:
		# Pos 1
		fc += filter(lambda a: 'y' in a and 'b'  in a and 'o' in a, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'o'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'o'  in a and 'g' in a, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'b'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'g'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'b'  in a and 'r' in a, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'r'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'y' in a and 'r'  in a and 'g' in a, cubies)[0]
		# Pos 9
		fc += filter(lambda a: 'o' in a and 'b'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'o' in a and 'g'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'b' in a and 'r'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'g' in a and 'r'  in a and a.count('n')==4, cubies)[0]
		# Pos 13
		fc += filter(lambda a: 'w' in a and 'b'  in a and 'o' in a, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'o'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'o'  in a and 'g' in a, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'b'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'g'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'b'  in a and 'r' in a, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'r'  in a and a.count('n')==4, cubies)[0]
		fc += filter(lambda a: 'w' in a and 'r'  in a and 'g' in a, cubies)[0]
	except IndexError:
		sys.exit( "Invalid cube entered: not all cubies accounted for")

	if len(fc) != 120:
		sys.exit( "An odd error has occured, final code for cube is not correct length: %r" % fc)

	return fc
