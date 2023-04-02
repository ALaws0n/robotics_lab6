#!/usr/bin/env python3
import rospy
import math
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# Create Initial A and B matrices
matrix_a = []
matrix_b = []

# Create Initial filter output values
fil_x_out = 0.0
fil_y_out = 0.0
fil_z_out = 0.0
fil_r_out = 0.0

# Create Global Boolean Flags
valid_unfiltered_parameters = False
first_filtering = True


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

		
def model_fitting(matrix_a, matrix_b):

	# Boolean flag for detecting the presence of valid sphere parameters
	global valid_unfiltered_parameters
	
	# Create numpy arrays out of our initial matrices
	A = np.array(matrix_a)
	B = np.array(matrix_b)
	
	# Create Empty Numpy array for storing result of model fitting
	P = np.array([])
	
	# Create Empty Sphere Params message for unfiltered Params
	Unfiltered_Sphere_Params = SphereParams()
	

	# Calculate P -- Catch error when matrice dimensions do not match requirements
	try:
		P = np.linalg.lstsq(A, B, rcond=None)[0]
		# Set unfiltered sphere param message
		Unfiltered_Sphere_Params.xc = P[0]
		Unfiltered_Sphere_Params.yc = P[1]
		Unfiltered_Sphere_Params.zc = P[2]
		Unfiltered_Sphere_Params.radius = math.sqrt(P[3] + P[0]**2 + P[1]**2 + P[2]**2)
		# Flip boolean flag due to valid parameters being recieved 
		valid_unfiltered_parameters = True
		# Return unfiltered parameter message
		return Unfiltered_Sphere_Params
	except:
		# Flip boolean flag due to parameter calculation failure
		valid_unfiltered_parameters = False
		print("Dimension error in calculation -- Continuing on!")
		
	
def filter_sphere_params(Unfiltered_Sphere_Params, fil_gain):

	# Boolean flag for detecting the first filtering attempt
	global first_filtering
	# References to the inital filter output values
	global fil_x_out
	global fil_y_out
	global fil_z_out
	global fil_r_out
	
	# Create Empty Sphere Parameter Message for Filtered Parameters
	Filtered_Sphere_Params = SphereParams()
	
	# Stores first parameter readins as our intial output guesses
	if first_filtering:
		# Set filter output values
		fil_x_out = Unfiltered_Sphere_Params.xc
		fil_y_out = Unfiltered_Sphere_Params.yc
		fil_z_out = Unfiltered_Sphere_Params.zc
		fil_r_out = Unfiltered_Sphere_Params.radius
		# Flip flag to ensure this does not execute again
		first_filtering = False
	
	# Set filter input values
	fil_x_in = Unfiltered_Sphere_Params.xc
	fil_y_in = Unfiltered_Sphere_Params.yc
	fil_z_in = Unfiltered_Sphere_Params.zc
	fil_r_in = Unfiltered_Sphere_Params.radius
	
	# Filter the Sphere Parameters
	fil_x_out = fil_gain*fil_x_in + (1 - fil_gain)*fil_x_out
	fil_y_out = fil_gain*fil_y_in + (1 - fil_gain)*fil_y_out
	fil_z_out = fil_gain*fil_z_in + (1 - fil_gain)*fil_z_out
	fil_r_out = fil_gain*fil_r_in + (1 - fil_gain)*fil_r_out
	
	# Set Filtered_Sphere_Params message values based off filter ouput
	Filtered_Sphere_Params.xc = fil_x_out
	Filtered_Sphere_Params.yc = fil_y_out
	Filtered_Sphere_Params.zc = fil_z_out
	Filtered_Sphere_Params.radius = fil_r_out
	
	# Return Filtered Sphere Parameters
	return Filtered_Sphere_Params
	
	
if __name__ == '__main__':
	# Initialize the ball detection node
	rospy.init_node('sphere_fit', anonymous = True)
	# Subscribe to point data
	point_data_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, receive_point_data)
	# Create Publisher for Sphere Parameters
	sphere_parameter_pub = rospy.Publisher("/sphere_params", SphereParams, queue_size = 10)
	# Set 10hz loop rate
	rate = rospy.Rate(10)
	# Set Filter Gain
	fil_gain = 0.05
	
	while not rospy.is_shutdown():
		# If our initial matrices are constructed and not empty -- Run model fitting function
		if len(matrix_a) > 0 and len(matrix_b) > 0:
			Unfiltered_Sphere_Params = model_fitting(matrix_a, matrix_b)
			# If valid unfiltered sphere parameters have been returned
			if valid_unfiltered_parameters:
				Filtered_Sphere_Parameters = filter_sphere_params(Unfiltered_Sphere_Params, fil_gain)
				# Publish Filtered Sphere Parameters
				sphere_parameter_pub.publish(Filtered_Sphere_Parameters)
			
		rate.sleep()
