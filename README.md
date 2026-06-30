# Line-Following-Robot

#### Summary: 
I built a ROS 2-based autonomous line-following robot from scratch using a Raspberry Pi. The robot performs real-time computer vision using an onboard camera, computes steering commands with centroid-based proportional control, and controls a differential-drive platform through a modular ROS 2 software architecture.

#### Video Link:
https://youtube.com/shorts/W9OxeIG6SVM?feature=share

#### System Architecture:  
<img width="1052" height="926" alt="Screenshot 2026-06-30 at 4 57 26 PM" src="https://github.com/user-attachments/assets/a46be888-1a78-467c-9791-1c12c59900e4" />

#### Hardware Used:   
L298N Driver, Rasberry Pi Camera, Rasberry Pi Computer, Double AA Batteries, SD Card, Motors, Chassis, Dupont Wires.  
<img width="384" height="512" alt="IMG_0534" src="https://github.com/user-attachments/assets/23201af6-87ec-41c3-a4cd-59e60bc1b80b" />
<img width="248" height="325" alt="Screenshot 2026-06-30 at 3 50 09 PM" src="https://github.com/user-attachments/assets/ad7a8110-d954-4186-95e0-84866be1b838" />
<img width="166" height="190" alt="Screenshot 2026-06-30 at 3 50 03 PM" src="https://github.com/user-attachments/assets/e439481e-6874-45e8-baa0-f11a56b1b4ca" />

#### Software Used:   
ROS 2 – Robotics middleware for inter-node communication.  
Ubuntu – Linux operating system running on the Raspberry Pi.  
OpenCV – Image processing and computer vision.  
GStreamer – Camera streaming framework.  
C++ – Core robotics software implementation.  
GPIO Library – Controls the L298N motor driver via Raspberry Pi GPIO pins.  

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
