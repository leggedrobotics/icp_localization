/*
 * Imu.hpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include "icp_localization/common/time.hpp"

#include <sensor_msgs/Imu.h>

namespace icp_loco {

template<typename FloatType>
class ImuReading
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  ImuReading()
      : ImuReading(Vector::Zero(), Vector::Zero(), Quaternion::Identity())
  {
  }
  ImuReading(const Vector& lin, const Vector& ang)
      : ImuReading(lin, ang, Quaternion::Identity())
  {
  }

  ImuReading(const Vector& lin, const Vector& ang, const Quaternion &q)
      : acc_(lin),
        angVel_(ang),
        q_(q)
  {
  }

  template<typename OtherType>
  ImuReading<OtherType> cast() const
  {
    return ImuReading<OtherType>(acc_.template cast<OtherType>(),
                                 angVel_.template cast<OtherType>(), q_.template cast<OtherType>());
  }

  const Vector& acceleration() const
  {
    return acc_;
  }
  const Vector& angularVelocity() const
  {
    return angVel_;
  }
  const Quaternion& rotation() const
  {
    return q_;
  }

  Vector& acceleration()
  {
    return acc_;
  }
  Vector& angularVelocity()
  {
    return angVel_;
  }
  Quaternion& rotation()
  {
    return q_;
  }

  std::string asString() const{
    const double kRadToDeg = 180.0 / M_PI;

    const std::string acc  =  string_format("acc:[%f, %f, %f]", acc_.x(), acc_.y(), acc_.z());
    const std::string angVel  =  string_format("angVel:[%f, %f, %f]", angVel_.x(), angVel_.y(), angVel_.z());
    const std::string rot = string_format("q:[%f, %f, %f, %f]",q_.x(), q_.y(), q_.z(), q_.w());
    const auto rpy = toRPY(q_) * kRadToDeg;
    const std::string rpyString = string_format("rpy (deg):[%f, %f, %f]",rpy.x(), rpy.y(),rpy.z());
    return acc + " ; " +angVel + " ; " + rot + " ; " + rpyString;
  }

 private:
  Vector acc_;
  Vector angVel_;
  Quaternion q_;
};

using ImuReadingd = ImuReading<double>;
using ImuReadingf = ImuReading<float>;

struct TimestampedImuReading
{
  Time time_;
  ImuReadingd imu_;
};

TimestampedImuReading interpolate(const TimestampedImuReading& start, const TimestampedImuReading& end,
                             const Time &time);

TimestampedImuReading fromRos(const sensor_msgs::Imu &msg);

} // namespace icp_loco
