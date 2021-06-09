/*
 * helpers.hpp
 *
 *  Created on: Apr 22, 2021
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include "icp_localization/common/typedefs.hpp"
#include <geometry_msgs/Pose.h>
#include "pointmatcher_ros/transform.h"
#include <memory>

namespace icp_loco {

Eigen::Vector3d getPositionFromParameterServer(const ros::NodeHandle &nh, const std::string &prefix);
Eigen::Quaterniond getOrientationFromParameterServer(const ros::NodeHandle &nh, const std::string &prefix, bool isRPYInDegrees=false);

template<typename Scalar>
Eigen::Matrix<Scalar,-1,-1> getTransformationMatrix(const Eigen::Matrix<Scalar,3,1> &position,
                                                         const Eigen::Quaternion<Scalar> &orientation);

template<typename Scalar>
void getPositionAndOrientation(const Eigen::Matrix<Scalar,-1,-1> &T, Eigen::Matrix<Scalar,3,1> *position,
                                                Eigen::Quaternion<Scalar> *orientation);

Eigen::Quaternionf toFloat(const Eigen::Quaterniond &q);
Eigen::Quaterniond toDouble(const Eigen::Quaternionf &q);

Eigen::MatrixXf toFloat(const Eigen::MatrixXd &m);
Eigen::MatrixXd toDouble(const Eigen::MatrixXf &m);


} // namespace icp_loco

template<typename Scalar>
Eigen::Matrix<Scalar,-1,-1> icp_loco::getTransformationMatrix(const Eigen::Matrix<Scalar,3,1> &position,
                                                         const Eigen::Quaternion<Scalar> &orientation)
{
  Eigen::Matrix<Scalar,-1,-1> T(4,4);
  T.setIdentity();
  T.block(0, 3, 3, 1) = position;
  T.block(0, 0, 3, 3) = orientation.toRotationMatrix();
  return T;
}

template<typename Scalar>
void icp_loco::getPositionAndOrientation(const Eigen::Matrix<Scalar,-1,-1> &T, Eigen::Matrix<Scalar,3,1> *position,
                                                Eigen::Quaternion<Scalar> *orientation)
{
  geometry_msgs::Pose pose = pointmatcher_ros::eigenMatrixToPoseMsg<Scalar>(T);
  *position = Eigen::Matrix<Scalar,3,1>(pose.position.x, pose.position.y, pose.position.z);
  const auto &q = pose.orientation;
  *orientation = Eigen::Quaternion<Scalar>(q.w, q.x, q.y, q.z);
  orientation->normalize();
}
