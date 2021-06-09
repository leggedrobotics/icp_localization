/*
 * FrameTracker.hpp
 *
 *  Created on: Apr 29, 2021
 *      Author: jelavice
 */

/*
 * TfPublisher.hpp
 *
 *  Created on: Apr 27, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization/transform/RigidTransform.hpp"
#include "icp_localization/transform/TransformInterpolationBuffer.hpp"

namespace icp_loco{

class ImuTracker;

class FrameTracker{

 public:
  explicit FrameTracker(std::shared_ptr<ImuTracker> imuTracker);

  virtual ~FrameTracker() = default;


  Rigid3d getTransformOdomToOdomSource(const Time &time) const;
  Rigid3d getTransformMapToOdom(const Time &time) const;
  Rigid3d getTransformMapToRangeSensor(const Time &time) const;
  Rigid3d getTransformOdomSourceToRangeSensor(const Time &time) const;
  Rigid3d getTransformImuToRangeSensor(const Time &time) const;
  Rigid3d getPoseChangeOfRangeSensorInMapFrame(const Time &start, const Time &finish) const;
  void setTransformMapToRangeSensor(const TimestampedTransform &mapToTracking);
  void setTransformOdomToOdomSource(const TimestampedTransform &odomToTracking);
  void setTransformImuToRangeSensor(const Rigid3d &t);
  void setTransformOdometrySourceToRangeSensor(const Rigid3d &t);
  void setIsUseOdometryForRangeSensorPosePrediction(bool value);
  void setMinNumOdomMeasurementsBeforeReady(int val);

  bool isRangeSensorTrasformBufferEmpty() const;
  bool isOdomTransformBufferEmpty() const;
  bool isReady() const;

 private:

  Rigid3d getTransform(const Time &time, const TransformInterpolationBuffer &buffer) const;
  Rigid3d computeFromOdometry(const Time &start, const Time &finish) const;
  Rigid3d computeFromImu(const Time &start, const Time &finish) const;

  TransformInterpolationBuffer mapToLidar_;
  TransformInterpolationBuffer odomToTrackingCamera_;
  Rigid3d cameraToLidar_;
  Rigid3d imuToLidar_;
  bool isUseOdometryForRangeSensorPosePrediction_ = true;
  std::shared_ptr<ImuTracker> imuTracker_;
  int minNumOdomMeasurementsBeforeReady_ = 300;

};

} // namespace icp_loco
