#!/usr/bin/env python3
import rospy
import math
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# Create Initial A and B matrices
matrix_a = []
matrix_b = []
# Create empty numpy array for P
P = np.array([])
# Create Empty Sphere Parameter Message
Sphere_Parameters = SphereParams()


def receive_point_data(point_data):
	global matrix_a
	global matrix_b
	
	# Loop through point data and create our initial A and B matrices
	for point in point_data.points:
		try:
			matrix_a.append([2*point.x, 2*point.y, 2*point.z, 1])
			matrix_b.append([point.x**2 + point.y**2 + point.z**2])
		except:
			# Message to display if something happens when appending data
			print("Data is not being appended")

		
def model_fitting_formula(matrix_a, matrix_b):
	global P
	# Create numpy arrays out of our initial matrices
	A = np.array(matrix_a)
	B = np.array(matrix_b)
	# Calculate P -- Catch error when matrice dimensions do not match requirements
	try:
		ATA = np.matmul(A.T, A)
		ATB = np.matmul(A.T, B)
		P = np.matmul(np.linalg.inv(ATA), ATB)
	except:
		print("Dimension error in calculation -- Continuing on!")
	
	
def calculate_sphere_params(P):
	# Set center parameters for sphere
	Sphere_Parameters.xc = P[0]
	Sphere_Parameters.yc = P[1]
	Sphere_Parameters.zc = P[2]
	# Calculate radius
	radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
	# Set radius parameter
	Sphere_Parameters.radius = radius

if __name__ == '__main__':
	# Initialize the ball detection node
	rospy.init_node('sphere_fit', anonymous = True)
	# Subscribe to point data
	point_data_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, receive_point_data)
	# Create Publisher for Sphere Parameters
	sphere_parameter_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 10)
	# Set 10hz loop rate
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		# If our initial matrices are constructed and not empty -- Run model fitting function
		if len(matrix_a) > 0 and len(matrix_b) > 0:
			model_fitting_formula(matrix_a, matrix_b)
			# If P has received valid data -- Calculate Sphere Parameters
			if len(P) > 0:
				calculate_sphere_params(P)
				# Publish Sphere Parameters
				sphere_parameter_pub.publish(Sphere_Parameters)
			
		rate.sleep()
