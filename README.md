# icp_slam

## How to use
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/RyuYamamoto/icp_slam
catkin build icp_slam -DCMAKE_BUILD_TYPE=Release
roslaunch icp_slam icp_slam.launch
```
open other terminal,
```
rosbag play --clock <ROSBAG PATH>
```

## Required packages
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
