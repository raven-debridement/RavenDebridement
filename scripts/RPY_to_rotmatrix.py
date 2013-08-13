#!/usr/bin/env python 	

import numpy

from math import sin, cos


def convert_to_matrix(roll, pitch, yaw, x, y, z):
	rotation = [[],[],[], []]
	rotation[0].append(cos(pitch)*cos(yaw))
	rotation[0].append(-sin(yaw)*cos(pitch))
	rotation[0].append(sin(pitch))
	rotation[0].append(x)

	rotation[1].append(cos(yaw)*sin(pitch)*sin(roll) + sin(yaw)*cos(roll))
	rotation[1].append(-sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll))
	rotation[1].append(-cos(pitch)*sin(roll))
	rotation[1].append(y)

	rotation[2].append(-cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll))
	rotation[2].append(sin(yaw)*sin(pitch)*cos(roll) + cos(yaw)*sin(roll))
	rotation[2].append(cos(pitch)*cos(roll))
	rotation[2].append(z)
	
	rotation[3] = [0.0, 0.0, 0.0, 1.0]
	rotation = numpy.matrix(rotation)
	return rotation

def main():
	roll = float(input("Input roll in radians: "))
	yaw = float(input("Input yaw in radians: "))
	pitch = float(input("Input pitch in radians: "))
	x = float(input("Input x in meters: "))
	y = float(input("Input y in meters: "))
	z = float(input("Input z in meters: "))
	result = convert_to_matrix(roll, pitch, yaw, x, y, z)
	print result

def test():
	elements = [-2.131, -.012, -1.557, -0.064, -0.327, 0.324]
	result = convert_to_matrix(elements[0], elements[1], elements[2], elements[3], elements[4], elements[5])
	print result

main()




	
