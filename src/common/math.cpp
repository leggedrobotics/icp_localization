/*
 * math.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#include "icp_localization/common/math.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace icp_loco {

Eigen::Quaterniond fromRPY(const double roll, const double pitch, const double yaw)
{

  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

Eigen::Vector3d toRPY(const Eigen::Quaterniond &_q)
{
  Eigen::Quaterniond q(_q);
  q.normalize();
  const double r = getRollFromQuat(q.w(), q.x(), q.y(), q.z());
  const double p = getPitchFromQuat(q.w(), q.x(), q.y(), q.z());
  const double y = getYawFromQuat(q.w(), q.x(), q.y(), q.z());
  return Eigen::Vector3d(r,p,y);
}

Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy){
	return fromRPY(rpy.x(), rpy.y(), rpy.z());
}

}  // namespace icp_loco

