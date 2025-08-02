# Line-Following-Robot

Hardware Used:   
-motors, chassis, jumper wires, L298N driver, Rasberry Pi Camera, Rasberry Pi Computer, double AA batteries, SD Card.  
-all components put together from scratch.

Software Used:   
-ROS, C++, Python, Ubuntu OS.

How to run the code:    
-Build and Source your ROS Workspace!  
-Run the Camera Driver Node: streams in camera images.  
-Run the Image Filter Node: turn pixels to either white or black so we know where the white line is.  Slice and use only the bottom of the image, and convert it to a ROS image.  
-Run the Line Follower Node: Includes a centroid calculation to publish the correct linear and angular values.  
-Run the Motor Controller Node:  Sends the correct signal to the correct motor over the gpio pins, for the right amount of time.  

Demo:  
-Download .mp4 file to see it live!
