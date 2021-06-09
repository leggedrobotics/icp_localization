/*
 * ImuTracker.hpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#pragma once

#include "icp_localization/transform/ImuInterpolationBuffer.hpp"
#include "icp_localization/transform/TransformInterpolationBuffer.hpp"
#include "icp_localization/common/time.hpp"
#include <functional>
namespace icp_loco {

class ImuTracker
{

  using IntegrationBuffer = std::vector<TimestampedImuReading>;
  using Accessor = std::function<Eigen::Vector3d&(TimestampedImuReading &)>;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuTracker();
  ~ImuTracker() = default;

  Eigen::Vector3d getLinearVelocityChange(const Time &start, const Time &end) const;
  Eigen::Quaterniond getOrientationChange(const Time &start, const Time &end) const;
  Eigen::Vector3d getLienarPositionChange(const Time &start, const Time &end,
                                          const Eigen::Vector3d &v0 =
                                              Eigen::Vector3d::Zero()) const;
  void setGravityVectorFilterTimeConstant(double val);
  ImuReadingd lookup(const Time &t) const;
  void addReading(const Time &t, const ImuReadingd &reading);
  const ImuInterpolationBuffer &getBuffer() const;

  // this is the transformation from gravity aligned frame
  // to the imu frame, only roll and pitch are good
  Eigen::Quaterniond orientation(const Time &t) const;
  Rigid3d getLatestOdometry() const;
  void setInitialPose (const Eigen::Vector3d &p, const Eigen::Quaterniond &q);

 private:
  void printBuffer(Accessor accessor) const;
  void integrateBufferInPlace(Accessor access, const Eigen::Vector3d &init) const;
  void fillIntegrationBuffer(const Time &start, const Time &end) const;
  void updateOrientationFromLinearAcceleration();
  void removeGravityAccelerationFromBuffer() const;
  void integrateImuReading(const Time &t,const ImuReadingd &reading);
  Eigen::Vector3d removeGravity(const Eigen::Vector3d &acc, const Eigen::Quaterniond &imuOrientation) const;

  ImuInterpolationBuffer imuBuffer_;
  mutable IntegrationBuffer integrationBuffer_;
  Eigen::Vector3d currentGravityVector_;
  double imuGravityTimeConstant_;
  Eigen::Quaterniond currentOrientation_;
  TransformInterpolationBuffer integratedPoses_;
  Eigen::Vector3d integratedLinearVelocity_;
  Eigen::Vector3d integratedPosition_;
  Eigen::Vector3d integratedRPY_;
  Time tPrev_;
};

}  // namespace icp_loco
