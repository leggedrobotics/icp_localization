/*
 * RangeDataAccumulator.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once

#include "icp_localization/Parameters.hpp"
#include "icp_localization/common/time.hpp"
#include "icp_localization/common/typedefs.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <mutex>



namespace icp_loco{

struct TimedRangeData{
  Time timestamp_;
  DP data_;
};

class RangeDataAccumulator {

 public:
  RangeDataAccumulator() = default;
  virtual ~RangeDataAccumulator() = default;


  void setParam( const RangeDataAccumulatorParam &p);
  const RangeDataAccumulatorParam &getParam() const;

  bool isAccumulatedRangeDataReady() const;
  void addRangeData(const DP &rangeData, const Time &t);
  const TimedRangeData &getAccumulatedRangeData() const;
  const TimedRangeData &popAccumulatedRangeData() const;
  const Time &getAccumulatedRangeDataTimestamp() const;
  void resetAccumulatedRangeData() const;

 protected:

  bool isAccumulatedTargetNumRangeData() const;

 private:
  TimedRangeData workingRangeData_;
  RangeDataAccumulatorParam param_;
  int currentNumRangeDataAccumulated_ = 0;
  TimedRangeData accumulatedRangeData_;
  mutable bool isRangeDataReady_ = false;
  mutable std::mutex accumulatedDataMutex_;

};

class RangeDataAccumulatorRos : public RangeDataAccumulator {
 public:
   RangeDataAccumulatorRos(const ros::NodeHandle &nh);
   ~RangeDataAccumulatorRos() override;

   void initialize();
   void cloudCallback(const sensor_msgs::PointCloud2 &msg);
   void setParam( const RangeDataAccumulatorParamRos &p);
   const RangeDataAccumulatorParamRos &getParam() const;
   void publishAccumulatedRangeDataWorker() const;

 private:
   ros::NodeHandle nh_;
   ros::Subscriber cloudSubscriber_;
   ros::Publisher accumulatedRangeDataPub_;
   RangeDataAccumulatorParamRos param_;
   std::thread publisherWorker_;
   std::string frameId_ ="map";



};


} // namespace icp_loco
