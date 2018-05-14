# 3d-hector-octree-slam
A complete odometry free solution for mapping and exploring an unknown environment via UAV

# Prerequisites
1. Knowledge of ROS
2. ROS needs to be installed

# Getting ROS packages
1. sudo apt-get install ros-<distro>-hector-slam
2. sudo apt-get install mavros
3. sudo apt-get install ros-<distro>-husky-navigation

# Steps to run the 3d SLAM
1. roslaunch <package name> dp3d.launch ( it will require hector_laser_to_pointcloud node, needs to be compiled in a catkin workspace)

# Also, please remap the topics as per your need 
Hardware in my case:
1. Rplidar (node name: rplidar_node, topic name: /scan)
2. Pixhawk 1 (node name: mavros, imu topic name: /mavros/imu/data)
3. I wrote a seperate node to publish height data by subscribing to the barometer topic, as I didn't have a dedicated height sensor. (My approach was inaccurate). Please get a height sensor and modify the python script in "learning_tf" folder as per your needs and "rosrun learning_tf <script_name>.py

# Results 
https://www.youtube.com/watch?v=w6ku_PNN5HA
