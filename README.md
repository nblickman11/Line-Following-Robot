# Line-Following-Robot

I built a ROS 2-based autonomous line-following robot from scratch using a Raspberry Pi. The robot performs real-time computer vision using an onboard camera, computes steering commands with centroid-based proportional control, and controls a differential-drive platform through a modular ROS 2 software architecture.

https://youtube.com/shorts/W9OxeIG6SVM?feature=share

Hardware Used:   
-Motors, Chassis, Dupont Wires, L298N Driver, Rasberry Pi Camera, Rasberry Pi Computer, Double AA Batteries, SD Card.  

Software Used:   
-ROS, C++, Python, Ubuntu OS.

How to run the code:    
-Build and Source your ROS Workspace!  
-Run the Camera Driver Node: streams in camera images.  
-Run the Image Filter Node: turn pixels to either white or black so we know where the white line is.  Slice and use only the bottom of the image, and convert it to a ROS image.  
-Run the Line Follower Node: Includes a centroid calculation to publish the correct linear and angular values.  
-Run the Motor Controller Node:  Sends the correct signal to the correct motor over the gpio pins, for the right amount of time.  
-To run the code of the faster robot, you must swap src folder with faster_src folder.

Demo:  
-Download .mp4 file = 54 second robot  
-Download .mov file = 38 second robot  

Improvements:  
-The faster robot was thanks to adjusting camera location, and increasing frame rate I published in the Camera Driver Node.  
-Also, I reduced the frames sent to the motor later down the node pipeline so that the motor control could keep up with the images it was seeing in real time.  
-In addition, in the motor node, I compare the resulting value of the P-Control centroid calculation which represents our error from the center, to, a threshold.
An error that is higher than the threshold resulted in a turn, but this was causing too much over-correction, and the robot moved horizontally too much.
Increasing that threshold value allowed for less turns, and a faster robot.  
-This faster robot started to overshoot the turns, so I increased the size of the image to look further ahead, which successfully prevented this.
