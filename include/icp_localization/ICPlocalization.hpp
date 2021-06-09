/*
 * ICPlocalization.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>
#include <thread>
#include "pointmatcher/PointMatcher.h"

#include "pointmatcher/IO.h"
#include <thread>
#include "icp_localization/common/typedefs.hpp"
#include "icp_localization/helpers.hpp"
#include "icp_localization/RangeDataAccumulator.hpp"


namespace icp_loco {

class TfPublisher;
class FrameTracker;
class ImuTracker;

class ICPlocalization
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ICPlocalization(const ros::NodeHandle &nh);
  ~ICPlocalization();
  void setMapCloud(const Pointcloud &map);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
  void initialize();
  DP fromPCL(const Pointcloud &pcl);
  void matchScans();


  void icpWorker();
  void publishPose() const;
  void publishRegisteredCloud() const;
  const std::string &getFixedFrame() const;

 private:
  void initializeInternal();

  Eigen::Vector3d lastPosition_;
  Eigen::Quaterniond lastOrientation_;
  Eigen::Vector3d currentPosition_;
  Eigen::Quaterniond currentOrientation_;
  Pointcloud mapCloud_;
  ros::NodeHandle nh_;
  ros::Publisher registeredCloudPublisher_;
  ros::Publisher posePub_;
  PM::ICPSequence icp_;
  PM::DataPointsFilters inputFilters_;
  DP refCloud_;
  DP regCloud_;
  bool isMapSet_ = false;
  mutable int seq_ = 0;
  RangeDataAccumulatorRos rangeDataAccumulator_;
  std::thread icpWorker_;
  PM::TransformationParameters optimizedPose_;
  Time optimizedPoseTimestamp_;
  Time lastOptimizedPoseTimestamp_;
  Time regCloudTimestamp_;
  std::shared_ptr<TfPublisher> tfPublisher_;
  std::shared_ptr<FrameTracker> frameTracker_;
  std::shared_ptr<ImuTracker> imuTracker_;
  bool isFirstScanMatch_ = true;
  bool isUseOdometry_ = false;
  std::string fixedFrame_ = "map";
};



} // namespace icp_loco
