/*
 * Twist.cpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */
#include "icp_localization/transform/Twist.hpp"

#include <eigen_conversions/eigen_msg.h>

namespace icp_loco {

TimestampedTwist interpolate(const TimestampedTwist& start, const TimestampedTwist& end,
                             const Time &time)
{
  if (time > end.time_ || time < start.time_) {
    throw std::runtime_error("twist interpolate: query time is not between start and end time");
  }

  const double duration = toSeconds(end.time_ - start.time_);
  const double factor = toSeconds(time - start.time_) / duration;
  const Eigen::Vector3d lin = start.twist_.linear()
      + (end.twist_.linear() - start.twist_.linear()) * factor;

  const Eigen::Vector3d ang = start.twist_.angular()
      + (end.twist_.angular() - start.twist_.angular()) * factor;

  return TimestampedTwist { time, Twist3d(lin, ang) };
}


Twist3d fromRos(const geometry_msgs::Twist &msg){
  Eigen::Vector3d lin,ang;
  tf::vectorMsgToEigen(msg.linear, lin);
  tf::vectorMsgToEigen(msg.angular, ang);
  return Twist3d(lin,ang);
}

}  // namespace icp_loco
