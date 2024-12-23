# Homework4_rl2024
Homework 4 for Robotics Lab 2024/2025

First build all the packages by using:

```
colcon build --packages-select aruco aruco_msgs aruco_ros rl_fra2mo_description ros2_opencv
```
In each terminal you open, source the install directory:
```
source install/setup.bash
```
# 1 Launch the Gazebo simulation and spawn the mobile robot in the world leonardo_race_field
Run the following command:
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
NOTE: to visualize the view of the camera we placed, run:
```
ros2 run rqt_image_view rqt_image_view
```
and select the /videocamera topic


# 2 Autonomous navigation task
In the first terminal run:
```
ros2 launch rl_fra2mo_description rviz_gazebo.launch.py
```
In another terminal launch:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```
In the last terminal run:
```
ros2 run rl_fra2mo_description follow_waypoints.py 
```

# 3 Tuning the navigation stack’s parameters:
In the first terminal run:
```
ros2 launch rl_fra2mo_description rviz_gazebo.launch.py
```
By defualt, if you run:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py 
```
will be selected low values for the slam parameters and medium values for the explore parameters.

If you want change the configurations, run:
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py slam_config_type:="medium" explore_config_type:="low"
```
This command selects medium values for the slam parameters and low values for the explore parameters.
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py slam_config_type:="high" explore_config_type:="default"
```
This command selects high values for the slam parameters and medium values for the explore parameters.
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py slam_config_type:="high" explore_config_type:="high"
```
This command selects high values for the slam parameters and high values for the explore parameters.

In general, the following terminal values can be provided for the slam_config_type parameter:
-defualt
-medium
-high

For the explore_config_type parameter, the following terminal values can be provided:
-defualt
-low
-high


# 4a-4b 2D navigation task
In the first terminal run:
```
ros2 launch rl_fra2mo_description rviz_gazebo.launch.py
```
In another terminal launch:
```
ros2 launch rl_fra2mo_description nav_aruco.launch.py
```
In another terminal run:
```
ros2 run rl_fra2mo_description vision_based_navigation.py
```
In another terminal, to check whether the robot is actually detecting the marker, run:
```
ros2 run rqt_image_view rqt_image_view 
```
and select as topic: aruco_single/result.


# 4c  Publish the Aruco pose as TF
In the first terminal run:
```
ros2 launch rl_fra2mo_description rviz_gazebo.launch.py
```
In second terminal, run:
```
ros2 launch rl_fra2mo_description nav_aruco.launch.py
```
To move the mobile robot close to obstacle 9 so that it can see the Aruco Marker with the camera, run:
```
ros2 run rl_fra2mo_description go_to_aruco.py
```
In another terminal, to check whether the robot is actually detecting the marker, run:
```
ros2 run rqt_image_view rqt_image_view
```
and select as topic: aruco_single/result.
 
In another terminal run:
```
ros2 run rl_fra2mo_description aruco_to_map
```
In the last terminal, to see the transformation published by the node, run:
```
ros2 topic echo /aruco_to_map
```
 
NOTES:
After running the command provided above, as soon as Gazebo opens, PRESS THE PLAY BUTTON in the lower left corner!!!