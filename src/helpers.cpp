/*
 * helpers.cpp
 *
 *  Created on: Apr 22, 2021
 *      Author: jelavice
 */

#include "icp_localization/helpers.hpp"
#include "icp_localization/common/math.hpp"
namespace icp_loco {

namespace {
const double kDegToRad = M_PI / 180.0;
}  // namespace

Eigen::Quaternionf toFloat(const Eigen::Quaterniond &q) {
	return q.cast<float>();
//  return Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
}

Eigen::Quaterniond toDouble(const Eigen::Quaternionf &q) {
	return q.cast<double>();
//  return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

Eigen::MatrixXf toFloat(const Eigen::MatrixXd &m) {
	return m.cast<float>();
}
Eigen::MatrixXd toDouble(const Eigen::MatrixXf &m) {
	return m.cast<double>();
}

Eigen::Vector3d getPositionFromParameterServer(const ros::NodeHandle &nh,
		const std::string &prefix) {
	double x, y, z;
//  const std::string prefix = "icp_localization/initial_pose/";

	bool res = nh.param(prefix + "x", x, 0.0) && nh.param(prefix + "y", y, 0.0)
			&& nh.param(prefix + "z", z, 0.0);
	if (!res) {
		ROS_ERROR_STREAM("Failed to load position " << prefix);
	}
	return Eigen::Vector3d(x, y, z);
}
Eigen::Quaterniond getOrientationFromParameterServer(const ros::NodeHandle &nh,
		const std::string &prefix, bool isRPYInDegrees) {
	double r, p, y;
//  const std::string prefix = "icp_localization/initial_pose/";

	bool res = nh.param(prefix + "roll", r, 0.0)
			&& nh.param(prefix + "pitch", p, 0.0)
			&& nh.param(prefix + "yaw", y, 0.0);
	if (!res) {
		ROS_ERROR_STREAM("Failed to load orientation : " << prefix);
	}

	if (isRPYInDegrees) {
		r *= kDegToRad;
		p *= kDegToRad;
		y *= kDegToRad;
	}
	return fromRPY(r, p, y).normalized();
}

}  // namespace icp_loco

