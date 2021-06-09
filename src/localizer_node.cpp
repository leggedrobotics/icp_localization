/*
 * pcl_loader_node.cpp
 *
 *  Created on: Mar 4, 2021
 *      Author: jelavice
 */
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include "icp_localization/common/typedefs.hpp"
#include "icp_localization/helpers.hpp"
#include "icp_localization/RangeDataAccumulator.hpp"
#include "icp_localization/ICPlocalization.hpp"

using namespace icp_loco;
Pointcloud::Ptr mapCloud;
ros::Publisher cloudPub;
const double kRadToDeg = 180.0/M_PI;

Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename)
{
  Pointcloud::Ptr cloud(new Pointcloud);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

void publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId)
{
  cloud->header.frame_id = frameId;
  cloud->header.seq = 0;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "tree_detection_eval_node");
  ros::NodeHandle nh("~");

  cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 1, true);
  const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
  mapCloud = loadPointcloudFromPcd(pclFilename);
  const Eigen::Quaterniond qInit = icp_loco::getOrientationFromParameterServer(nh,"icp_localization/initial_pose/",true);
  const Eigen::Vector3d pInit = icp_loco::getPositionFromParameterServer(nh,"icp_localization/initial_pose/");
  ICPlocalization icp(nh);
  icp.setMapCloud(*mapCloud);
  icp.setInitialPose(pInit, qInit);
  icp.initialize();
  std::cout << "succesfully initialized icp" << std::endl;


  publishCloud(mapCloud, cloudPub, icp.getFixedFrame());

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

