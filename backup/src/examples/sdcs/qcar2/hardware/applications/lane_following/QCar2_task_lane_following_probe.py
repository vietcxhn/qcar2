## task_lane_following.py
# This example combines both the left csi and motor commands to
# allow the QCar to follow a yellow lane. Use the joystick to manually drive the QCar
# to a starting position and enable the line follower by holding the X button on the LogitechF710
# To troubleshoot your camera use the hardware_test_csi_camera_single.py found in the hardware tests

# from pal.utilities.vision import Camera2D
from pal.products.qcar import QCar, QCarCameras
from pal.utilities.math import Filter
from pal.utilities.gamepad import LogitechF710
from pal.utilities.probe import Probe
from hal.utilities.image_processing import ImageProcessing

import time
import numpy as np
import cv2
import math

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Timing Parameters and methods
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional parameters
ipHost, ipDriver = '192.168.3.10', 'localhost'
counter 	= 0
imageWidth  = 1640
imageHeight = 820
# cameraID 	= '2'

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#Setting Filter
steeringFilter = Filter().low_pass_first_order_variable(25, 0.033)
next(steeringFilter)
dt = 0.033

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Initialize the CSI cameras
# myCam = Camera2D(cameraId=cameraID, frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate)
myCam = QCarCameras(frameWidth=imageWidth, frameHeight=imageHeight, frameRate=sampleRate, enableFront=True)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## QCar, Gamepad, and probe Initialization
myCar = QCar(readMode=1, frequency=60)
gpad  = LogitechF710()
probe = Probe(ip = ipHost)
probe.add_display(imageSize = [imageHeight, imageWidth, 3], scaling = True,
					scalingFactor= 4, name="Detection Overlay")

def control_from_gamepad(LB, RT, leftLateral, A):
	'''	User control function for use with the LogitechF710
	LB on gamepad is used to enable motor commands based on the RT input.
	Button A on gamepad is used to reverse the motor direction.
	'''
	if LB == 1:
			if A == 1 :
				throttle_axis = -0.3 * RT #going backward
				steering_axis = leftLateral * 0.5
			else:
				throttle_axis = 0.3 * RT #going forward
				steering_axis = leftLateral * 0.5
	else:
		throttle_axis = 0
		steering_axis = 0

	command = np.array([throttle_axis, steering_axis])
	return command


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		flags=myCam.readAll()
		if any(flags): counter +=1

		# Crop out a piece of the RGB to improve performance
		croppedRGB = myCam.csiFront.imageData[524:674, 0:820]

		# Convert to HSV and then threshold it for yellow
		hsvBuf = cv2.cvtColor(croppedRGB, cv2.COLOR_BGR2HSV)

		binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf,
													lowerBounds=np.array([10, 50, 100]),
													upperBounds=np.array([45, 255, 255]))
		
		# Overlay detected yellow lane over raw RGB image
		binaryImage=binaryImage/255
		processed = myCam.csiFront.imageData
		processed[524:674, 0:820,2]=processed[524:674, 0:820,2]+(255-processed[524:674, 0:820,2])*binaryImage
		processed[524:674, 0:820,1]=processed[524:674, 0:820,1]*(1-binaryImage)
		processed[524:674, 0:820,0]=processed[524:674, 0:820,0]*(1-binaryImage)

		# Send the processed image to the observer on the loacl PC to display
		if not probe.connected:
			probe.check_connection()
		if probe.connected and counter%2==0:
			sending = probe.send(name="Detection Overlay",imageData=processed)

		# Find slope and intercept of linear fit from the binary image
		slope, intercept = ImageProcessing.find_slope_intercept_from_binary(binary=binaryImage)

		# steering from slope and intercept
		rawSteering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steeringFilter.send((np.clip(rawSteering, -0.5, 0.5), dt))

		# Write steering to qcar
		new = gpad.read()
		QCarCommand = control_from_gamepad(gpad.buttonLeft, gpad.trigger, gpad.leftJoystickY, gpad.buttonA)
		if gpad.buttonX == 1:
			if math.isnan(steering):
				QCarCommand[1] = 0
			else:
				QCarCommand[1] = steering
			QCarCommand[0] = QCarCommand[0]*np.cos(steering)

		LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
		myCar.read_write_std(QCarCommand[0],QCarCommand[1],LEDs)

		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")
		
finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
	probe.terminate()
	gpad.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
