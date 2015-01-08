Ladybug5 for ROS

Receiver_nodelet: Connectes to the windows pc running the ladybug grabber.
	stream images from the Windows Ladybug SDK to ROS.
	allows easy image and sensor data recording to bagfile
Image_nodelet: Publishes images using image transport.
Service: Connects to the windows pc allows control an reconfiguration of the Windows PC from ROS
Rectifier_nodelet: Uses the extracted look up table (windows) to rectify the images, 
	allows zooming and publishes a projection matrix. 	 

Recieves ladybug5 jpg color seperated images over network from a windows pc.
Publishes the jpg images as seperated message for bagfile recording. 
Processes the jpg color seperated images to a ros image transport messages.

requires
- protobuff 
- zmq (zeromq-3.2.4)
