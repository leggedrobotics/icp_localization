/*
 * TfPublisher.hpp
 *
 *  Created on: Apr 27, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization/transform/RigidTransform.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace icp_loco {

class FrameTracker;
class ImuTracker;

class TfPublisher
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TfPublisher(const ros::NodeHandle &nh, std::shared_ptr<FrameTracker> frameTracker,
              std::shared_ptr<ImuTracker> imuTracker);
  ~TfPublisher() = default;
  void setOdometryTopic(const std::string &topic);
  void setImuTopic(const std::string &topic);

  void initialize();
  void publishMapToOdom(const Time &time) ;
  void publishMapToRangeSensor(const Time &time) ;
  void setIsProvideOdomFrame(bool value);
  void setIsUseOdometry(bool value);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
 private:

  void odometryCallback(const nav_msgs::Odometry &msg);
  void imuCallback(const sensor_msgs::Imu &msg);

  tf2_ros::TransformBroadcaster tfBroadcaster_;
  std::string odometryTopic_;
  std::string imuTopic_;
  ros::NodeHandle nh_;
  ros::Subscriber odomSubscriber_;
  ros::Subscriber imuSubscriber_;
  std::shared_ptr<FrameTracker> frameTracker_;
  std::shared_ptr<ImuTracker> imuTracker_;
  bool isProvideOdomFrame_ = false;
  bool isUseOdometry_ = false;
  Eigen::Vector3d initPosition_;
  Eigen::Quaterniond initOrientation_;
};

}  // namespace icp_loco
