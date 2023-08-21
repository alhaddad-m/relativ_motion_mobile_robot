# relativ_motion_mobile_robot
ROS node for motion mobile robot forward and backward for a specific distance and orientation in relative to this position in a map

## Launch
every distance 20cm requires parameter how_long equals to 1, the first and last orientations are given as theta1 and theta2 as follwoing:

roslaunch relative_motion motion.launch how_long:=1 theta1:=0 theta2:=1.57
