/*
 * Imu.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/ImuReading.hpp"

#include "icp_localization/common/math.hpp"

#include <eigen_conversions/eigen_msg.h>

namespace icp_loco {

TimestampedImuReading interpolate(const TimestampedImuReading& start,
                                  const TimestampedImuReading& end, const Time &time)
{
  if (time > end.time_ || time < start.time_) {
    throw std::runtime_error("imu interpolate: query time is not between start and end time");
  }

  const auto &s = start.imu_;
  const auto &e = end.imu_;
  TimestampedImuReading retVal;
  retVal.imu_.acceleration() = interpolateVector(s.acceleration(), e.acceleration(), start.time_,
                                                 end.time_, time);
  retVal.imu_.angularVelocity() = interpolateVector(s.angularVelocity(), e.angularVelocity(),
                                                    start.time_, end.time_, time);
  retVal.imu_.rotation() = interpolateQuaternion(s.rotation(), e.rotation(), start.time_, end.time_,
                                                 time);
  return retVal;

}

TimestampedImuReading fromRos(const sensor_msgs::Imu &msg)
{
  Eigen::Vector3d lin, ang;
  Eigen::Quaterniond q;
  tf::vectorMsgToEigen(msg.linear_acceleration, lin);
  tf::vectorMsgToEigen(msg.angular_velocity, ang);
  tf::quaternionMsgToEigen(msg.orientation, q);
  TimestampedImuReading retVal;
  retVal.imu_ = ImuReadingd(lin,ang,q);
  retVal.time_ = fromRos(msg.header.stamp);
  return retVal;
}




}  // namespace icp_loco
