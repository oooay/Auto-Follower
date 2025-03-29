# Auto-Follower
**TODU**

## Menu
- [**System Overview**](#system-overview)
- [**Package dependency**](#dependency)
- [**Package install**](#install)
- [**Run the package**](#run-the-package)
- [**Related Package**](#related-package)

## System Overview
"TODU" 

## Dependency
This is the original ROS1 implementation of Auto-Follower. Auto-Follower has benn tested on ubuntu18.04 with the ROS melodic and ubuntu20.04 with the ROS noetic.  
```
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

## Install
Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/oooay/Auto-Follower.git
cd ..
catkin_make
```

## Run the package
**1. Run the sensor drivers launch file:**  
```
roslaunch yhs_can_control yhs_can_control.launch  
roslaunch rslidar_sdk start.launch  
roslaunch usb_cam usb_cam.launch  
roslaunch visual_follow_motor visual_follow_motor.launch  
```
**2. Person Identification and Localization**  
```
roslaunch tracker track_main.launch  
roslaunch hybrid_astar path_planning_new.launch  
```
**3. Local mapping and IRPS-based path navigation** 
```
roslaunch linefit_ground_segmentation_ros segmentation.launch  
roslaunch visual_follow_mapping visual_follow_mapping.launch  
roslaunch hybrid_astar path_planning.launch  
rosrun hybrid_astar path_tracking_purepursuit.py  
```

## Related packages
- [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation)
- [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [usb_cam](http://wiki.ros.org/usb_cam)
- [hybrid-a-star-annotation](https://github.com/teddyluo/hybrid-a-star-annotation)
- [path_planner](https://github.com/karlkurzer/path_planner)
