
TODO

Dependecies:

install libpointmatcher from https://github.com/leggedrobotics/libpointmatcher

install pcl_ros and pcl_conversions

install pointmatcher_ros from https://github.com/leggedrobotics/pointmatcher-ros


This package provides a localization in a given map using the ICP algorithm. You can launch the program with: `roslaunch icp_localization icp_node.launch`. The `pcd_filepath` parameter in the launch file should point to the location where you stored your refrence map (pointcloud) as a `.pcd` file.
