
WORK IN PROGRESS

# Localization using ICP in a known map

## Overview

This package localizes the lidar sensor in a given map using the ICP algorithm. It subscribes to lidar scan and it registers them in a given map. If provided, it can use odometry or IMU to extrapolate the pose between the two iterations of ICP.

The user should provide a reference map (point cloud) as as a `.pcd` file and an initial pose in the reference map.

Released under [BSD 3-Clause license](LICENSE). Parts of the code in this repo have been inspired by the code inside google cartogrpaher. We've retained the copyright headers where applicable.

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)




| handheld | skid steer robot | legged robot |
|:--------:|------------------|--------------|
|          |                  |              |



## Installation
install libpointmatcher from https://github.com/leggedrobotics/libpointmatcher

install pcl_ros and pcl_conversions

install pointmatcher_ros from https://github.com/leggedrobotics/pointmatcher-ros

## Usage
You can launch the program with: `roslaunch icp_localization icp_node.launch`. The `pcd_filepath` parameter in the launch file should point to the location where you stored your refrence map (pointcloud) 



