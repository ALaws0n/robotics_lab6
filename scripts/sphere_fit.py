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
# Create global variables for initial filter guesses
initial_measurements_unobtained = True
fil_x_out = 0.0
fil_y_out = 0.0
fil_z_out = 0.0
fil_r_out = 0.0
fil_x_in = 0.0
fil_y_in = 0.0
fil_z_in = 0.0
fil_r_in = 0.0


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
		#ATA = np.matmul(A.T, A)
		#ATB = np.matmul(A.T, B)
		#P = np.matmul(np.linalg.inv(ATA), ATB)
		P = np.linalg.lstsq(A, B, rcond=None)[0]
	except:
		print("Dimension error in calculation -- Continuing on!")
	
	
def calculate_and_filter_sphere_params(P):
	global initial_measurements_unobtained
	global fil_x_out
	global fil_y_out
	global fil_z_out
	global fil_r_out
	global fil_x_in
	global fil_y_in
	global fil_z_in
	global fil_r_in
	# Set center parameters for sphere
	#print(P[0])
	#print(P[1])
	#print(P[2])
	
	# Define Filter Gain value
	fil_gain = 0.8
	# Grab initial center and radius position measurements
	if initial_measurements_unobtained:
		fil_x_out = P[0]
		fil_y_out = P[1]
		fil_z_out = P[2]
		fil_r_out = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
		initial_measurements_unobtained = False
	# Update filter input
	fil_x_in = P[0]
	print('fil_x_in:', fil_x_in, ' fil_x_out now:', fil_x_out)
	fil_y_in = P[1]
	fil_z_in = P[2]
	fil_r_in = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
	# Filter the data
	fil_x_out = fil_gain*fil_x_in + (1 - fil_gain)*fil_x_out
	print('fil_x_out next:', fil_x_out)
	fil_y_out = fil_gain*fil_y_in + (1 - fil_gain)*fil_y_out
	fil_z_out = fil_gain*fil_z_in + (1 - fil_gain)*fil_z_out
	fil_r_out = fil_gain*fil_r_in + (1 - fil_gain)*fil_r_out
	# Set Sphere Parameters
	Sphere_Parameters.xc = fil_x_out
	Sphere_Parameters.yc = fil_y_out
	Sphere_Parameters.zc = fil_z_out
	Sphere_Parameters.radius = fil_r_out

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
				calculate_and_filter_sphere_params(P)
				# Publish Sphere Parameters
				sphere_parameter_pub.publish(Sphere_Parameters)
			
		rate.sleep()
