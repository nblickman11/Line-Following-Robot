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

#### Algorithm and Implementation:

The robot uses a modular ROS 2 perception and control pipeline to convert camera images into real-time steering commands.

###### Camera Capture
- Captures 640×480 RGB images from the Raspberry Pi Camera using a GStreamer pipeline.
- Converts image frames into ROS 2 image messages using OpenCV and `cv_bridge`.
- Publishes synchronized `Image` and `CameraInfo` messages.

###### Image Processing
- Converts RGB images to grayscale.
- Applies binary thresholding to isolate the white line from the background.
- Publishes the thresholded image for downstream processing.

###### Line Detection
- Computes the centroid of the detected line using OpenCV image moments.
- Calculates steering error as the horizontal distance between the line centroid and the center of the image.
- Processes every third image frame to reduce computation and improve controller responsiveness.

###### Proportional Steering Control
- Uses a proportional (P) controller to convert centroid error into an angular steering command.
- Publishes linear and angular velocity commands (`/cmd_vel`) for robot navigation.

###### Differential Drive Motor Control
- Converts velocity commands into GPIO signals for the L298N motor driver.
- Controls the left and right motors independently to drive forward, turn, and perform smooth curved steering.
- Applies steering thresholds and pulsed motor commands to reduce oscillation and improve tracking performance.

#### Improvements:

- Improved overall lap time by repositioning the camera and increasing the camera frame rate in the Camera Driver node, allowing the robot to detect upcoming turns earlier.
- Reduced the number of image frames processed downstream in the ROS 2 pipeline, allowing the motor controller to keep pace with incoming sensor data and reducing control latency.
- Tuned the proportional steering controller by increasing the steering threshold. Small centroid errors were ignored, reducing unnecessary steering corrections, minimizing oscillation, and allowing the robot to maintain higher speeds.
- Increased the image region used for line detection, enabling the robot to look farther ahead on the track. This reduced overshooting during turns and produced smoother, more stable navigation.
