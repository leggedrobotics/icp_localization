/*
 * math.hpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <Eigen/Dense>
#include "icp_localization/common/time.hpp"

namespace icp_loco {

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond fromRPY(double roll, double pitch, double yaw);
Eigen::Vector3d toRPY(const Eigen::Quaterniond &q);
Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy);

template<typename T>
inline T getRollFromQuat(T w, T x, T y, T z)
{
  return std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

template<typename T>
inline T getPitchFromQuat(T w, T x, T y, T z)
{
  return std::asin(2 * (w * y - x * z));
}

template<typename T>
inline T getYawFromQuat(T w, T x, T y, T z)
{
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

template<typename EigenVector>
EigenVector interpolateVector(const EigenVector &vStart,  const EigenVector &vEnd, const Time &timeStart,
                          const Time &timeEnd, const Time &queryTime)
{
  const double duration = toSeconds(timeEnd - timeStart);
  const double factor = toSeconds(queryTime - timeStart) / duration;
  const Eigen::Vector3d interp = vStart + (vEnd - vStart) * factor;
  return interp;
}

template<typename EigenQuaternion>
EigenQuaternion interpolateQuaternion(const EigenQuaternion &qStart,  const EigenQuaternion &qEnd, const Time &timeStart,
                          const Time &timeEnd, const Time &queryTime)
{
  const double duration = toSeconds(timeEnd - timeStart);
  const double factor = toSeconds(queryTime - timeStart) / duration;
  const Eigen::Quaterniond interp =
      Eigen::Quaterniond(qStart)
          .slerp(factor, Eigen::Quaterniond(qEnd));
  return interp;
}

template <typename T>
Eigen::Quaternion<T> angleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

}  // namespace icp_loco
