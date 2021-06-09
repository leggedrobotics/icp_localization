/*
 * RangeDataAccumulator.cpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#include "icp_localization/RangeDataAccumulator.hpp"

#include "pointmatcher_ros/serialization.h"
#include "pointmatcher_ros/deserialization.h"
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"
#include "pointmatcher_ros/transform.h"
#include <pointmatcher_ros/StampedPointCloud.h>

namespace icp_loco {

void RangeDataAccumulator::setParam(const RangeDataAccumulatorParam &p)
{
  param_ = p;
}
const RangeDataAccumulatorParam &RangeDataAccumulator::getParam() const
{
  return param_;
}

bool RangeDataAccumulator::isAccumulatedTargetNumRangeData() const
{
  return currentNumRangeDataAccumulated_ == param_.numAccumulatedRangeData_;
}

const Time &RangeDataAccumulator::getAccumulatedRangeDataTimestamp() const
{
  return accumulatedRangeData_.timestamp_;
}

bool RangeDataAccumulator::isAccumulatedRangeDataReady() const
{
  return isRangeDataReady_;
}

const TimedRangeData &RangeDataAccumulator::popAccumulatedRangeData() const
{
  std::lock_guard<std::mutex> lck(accumulatedDataMutex_);
  isRangeDataReady_ = false;
  return accumulatedRangeData_;
}

void RangeDataAccumulator::addRangeData(const DP &rangeData, const Time &t)
{
  //todo add motion compensation
  std::lock_guard<std::mutex> lck(accumulatedDataMutex_);
  // check if we need to erase the data
  if (currentNumRangeDataAccumulated_ == 0) {
    workingRangeData_.data_ = rangeData;
    workingRangeData_.timestamp_ = t;
    currentNumRangeDataAccumulated_ = 1;
    return;
  }

  if (isAccumulatedTargetNumRangeData()) {
    accumulatedRangeData_ = workingRangeData_;
    workingRangeData_.data_ = rangeData;
    workingRangeData_.timestamp_ = t;
    currentNumRangeDataAccumulated_ = 1;
    isRangeDataReady_ = true;
    return;
  }

  workingRangeData_.data_.concatenate(rangeData);
  ++currentNumRangeDataAccumulated_;

  if (isAccumulatedTargetNumRangeData()) {
    accumulatedRangeData_ = workingRangeData_;
    workingRangeData_.data_ = rangeData;
    workingRangeData_.timestamp_ = t;
    currentNumRangeDataAccumulated_ = 1;
    isRangeDataReady_ = true;
  }

}

const TimedRangeData &RangeDataAccumulator::getAccumulatedRangeData() const
{
  return accumulatedRangeData_;
}

/*
 * *******************************************
 * *******************************************
 * *******************************************
 */

RangeDataAccumulatorRos::RangeDataAccumulatorRos(const ros::NodeHandle &nh)
    : nh_(nh)
{

}

void RangeDataAccumulatorRos::initialize()
{
  cloudSubscriber_ = nh_.subscribe(param_.inputRangeDataTopic_, 1,
                                   &RangeDataAccumulatorRos::cloudCallback, this);
  accumulatedRangeDataPub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      param_.accumulatedRangeDataTopic_, 1, true);
  //launch worker
  auto callable = [this]() {
    publishAccumulatedRangeDataWorker();
  };
  publisherWorker_ = std::thread(callable);
}

void RangeDataAccumulatorRos::cloudCallback(const sensor_msgs::PointCloud2 &msg)
{
//  DP cloud = pointmatcher_ros::rosMsgToPointMatcherCloud<float>(msg);
  DP cloud = pointmatcher_ros::RosPointCloud2Deserializer<float>::deserialize(msg);

  Time timestamp;
  timestamp = fromRos(msg.header.stamp);
  addRangeData(cloud, timestamp);
  frameId_ = msg.header.frame_id;
}

void RangeDataAccumulatorRos::setParam(const RangeDataAccumulatorParamRos &p)
{
  param_ = p;
  RangeDataAccumulator::setParam(*(p.as<RangeDataAccumulatorParam>()));
}
const RangeDataAccumulatorParamRos &RangeDataAccumulatorRos::getParam() const
{
  return param_;
}

void RangeDataAccumulatorRos::publishAccumulatedRangeDataWorker() const
{
  ros::Rate r(50);
  while (ros::ok()) {
    if (isAccumulatedTargetNumRangeData()) {
      const auto cloud = getAccumulatedRangeData();
      const auto timestamp = getAccumulatedRangeDataTimestamp();
      if (toUniversal(timestamp) == 0){
    	  r.sleep();
    	  continue;
      }

      const ros::Time rosTime = toRos(timestamp);
      sensor_msgs::PointCloud2 msg = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(cloud.data_,
                                                                                        frameId_,
                                                                                        rosTime);
      accumulatedRangeDataPub_.publish(msg);
    }
    r.sleep();
  }

}

RangeDataAccumulatorRos::~RangeDataAccumulatorRos()
{
  publisherWorker_.join();
}

}  // namespace icp_loco
