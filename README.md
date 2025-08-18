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

...
Future SLAM:
-run image proc node which subscribes to my camera nodes image_raw and camera_info and then publishes the image (we map the image name to rect color name)
ros2 run image_proc image_proc \
  --ros-args \
  -r image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
 -r image_rect:=/camera/image_rect

 ********** SKIP ODOM FOR NOW
-run slam node again, which subscribes to image_rect and camera_info and publishes /odom and /tf.  This allows SLAM to estimate camera position over time.
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/image_rect \
  camera_info_topic:=/camera/camera_info \
  odom_topic:=/odom \
  frame_id:=camera_link  
*************  

-run the SLAM node. I probably don’t need to remap the naming to image_rect_color, since that name is the same in the launch file.  Maybe same with Odom.  Also \tf not needed since done automatically.
ros2 launch rtabmap_launch rtabmap.launch.py \
   rgb_topic:=/camera/image_rect \
   camera_info_topic:=/camera/camera_info \
   use_odometry:=false \
   rtabmap.subscribe_depth:=false \
   rtabmap.subscribe_odom:=false \
   rtabmap.subscribe_odom_info:=false \
   approx_sync:=true \
   rtabmap_viz:=false  

I went into my launch file: in the /opt directroy:
-i changed the file in 3 spots 

Run from VM:
LIBGL_ALWAYS_SOFTWARE=1 ros2 run rtabmap_viz rtabmap_viz --ros-args -r __node:=rtabmap_viz -r __ns:=/rtabmap  

Other SLAM Notes:
SLAM PICKUP:
https://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
https://introlab.github.io/rtabmap/
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 run tf2_ros static_transform_publisher 0.1 0 0.15 0 0 0 
