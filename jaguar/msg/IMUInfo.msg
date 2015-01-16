# IMU Sensor information message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the data reading
#make sure below sensors on your robot or not
uint32[3] accelRawData 		#Raw accel sensor reading, 0-x, 1-y,z-2
uint32[3] gyroRawData    	#Gyro raw data
uint32[3] magRawData   		# magRaw
float32[3] accelReading		# accel sensor reading  m/s2
float32[3] gyroReading		# gyro sensor reading   rad/m

