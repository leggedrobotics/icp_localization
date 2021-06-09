/*
 * FrameTracker.cpp
 *
 *  Created on: Apr 29, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/FrameTracker.hpp"
#include "icp_localization/common/assert.hpp"
#include "icp_localization/transform/ImuTracker.hpp"

namespace icp_loco {

FrameTracker::FrameTracker(std::shared_ptr<ImuTracker> imuTracker) :
		imuTracker_(imuTracker) {
	odomToTrackingCamera_.setSizeLimit(600);
	mapToLidar_.setSizeLimit(100);

	if (imuTracker_ == nullptr) {
		throw std::runtime_error("Imu tracker is nullptr");
	}

}

Rigid3d FrameTracker::getTransform(const Time &time,
		const TransformInterpolationBuffer &buffer) const {
	if (time < buffer.earliest_time()) {
		return buffer.lookup(buffer.earliest_time());
	}
	if (time > buffer.latest_time()) {
		return buffer.lookup(buffer.latest_time());
	}
	return buffer.lookup(time);

}

void FrameTracker::setMinNumOdomMeasurementsBeforeReady(int val){
	minNumOdomMeasurementsBeforeReady_ = val;
}

bool FrameTracker::isReady() const{
	const bool isOdomBufferReady = odomToTrackingCamera_.size()>minNumOdomMeasurementsBeforeReady_ || !isUseOdometryForRangeSensorPosePrediction_;
	// std::cout << "Odom buffer ready: " << isOdomBufferReady << std::endl;
	// std::cout << "Range buffer ready: " << isRangeBufferReady << std::endl;
	// std::cout << "is use odom for prediciton: " << isUseOdometryForRangeSensorPosePrediction_ << std::endl;
	return isOdomBufferReady;
}


bool FrameTracker::isRangeSensorTrasformBufferEmpty() const{
	return mapToLidar_.empty();

}
  bool FrameTracker::isOdomTransformBufferEmpty() const{
  	return odomToTrackingCamera_.empty();
  }

Rigid3d FrameTracker::getTransformOdomSourceToRangeSensor(
		const Time &time) const {
	return cameraToLidar_;
}

Rigid3d FrameTracker::getTransformImuToRangeSensor(const Time &time) const {
	return imuToLidar_;
}

void FrameTracker::setTransformImuToRangeSensor(const Rigid3d &t) {
	imuToLidar_ = t;
}
void FrameTracker::setTransformOdometrySourceToRangeSensor(const Rigid3d &t) {
	cameraToLidar_ = t;
}

void FrameTracker::setIsUseOdometryForRangeSensorPosePrediction(bool value) {
	isUseOdometryForRangeSensorPosePrediction_ = value;
}

Rigid3d FrameTracker::getTransformOdomToOdomSource(const Time &time) const {
	const auto ret = getTransform(time, odomToTrackingCamera_);
	return ret;
}

Rigid3d FrameTracker::getTransformMapToRangeSensor(const Time &time) const {
	const auto ret = getTransform(time, mapToLidar_);
	return ret;
}

Rigid3d FrameTracker::getTransformMapToOdom(const Time &time) const {

	if (mapToLidar_.empty() || odomToTrackingCamera_.empty()){
		std::cerr << "empty buffer cannot compute map to odom" << std::endl;
		return Rigid3d::Identity();
	}

	const auto mapToLidar = getTransformMapToRangeSensor(time);
	const auto odomToCamera = getTransformOdomToOdomSource(time);
	const auto lidarToOdom = (odomToCamera * cameraToLidar_).inverse();
	const auto mapToOdom = mapToLidar * lidarToOdom;

	return mapToOdom;

}
void FrameTracker::setTransformMapToRangeSensor(
		const TimestampedTransform &mapToTracking) {
//  std::cout << "set map to lidar: " << mapToTracking.transform_.asString() << std::endl;
	mapToLidar_.push(mapToTracking.time_, mapToTracking.transform_);
}

void FrameTracker::setTransformOdomToOdomSource(
		const TimestampedTransform &odomToTracking) {
//  std::cout << "set odom to camera: " << odomToTracking.transform_.asString() << std::endl;

	odomToTrackingCamera_.push(odomToTracking.time_, odomToTracking.transform_);
}

Rigid3d FrameTracker::getPoseChangeOfRangeSensorInMapFrame(const Time &start,
		const Time &finish) const {

	assert_ge(toUniversal(finish), toUniversal(start));

//	const auto dTodometry = computeFromOdometry(start, finish);
//	const auto dTimu = computeFromImu(start, finish);
// std::cout << "From odometry: " << dTodometry.asString() << std::endl;
// std::cout << "From imu: " << dTimu.asString() << "\n\n";
	if (isUseOdometryForRangeSensorPosePrediction_) {
		return computeFromOdometry(start, finish);
	} else {
		return computeFromImu(start, finish);
	}

}

Rigid3d FrameTracker::computeFromOdometry(const Time &start,
		const Time &finish) const {
	//this is not correct I think
	const auto startPose = getTransformOdomToOdomSource(start);
	const auto finishPose = getTransformOdomToOdomSource(finish);
	Rigid3d poseDifference = startPose.inverse() * finishPose;

//	const auto odomRPY = toRPY(getTransformMapToOdom(start).rotation());
//	const auto odomYawCorrection = Rigid3d(Eigen::Vector3d::Zero(),
//			fromRPY(0.0, 0.0, -odomRPY.z()));
//	poseDifference.translation() = odomYawCorrection
//			* poseDifference.translation();

	return cameraToLidar_.inverse() * poseDifference * cameraToLidar_;
}
Rigid3d FrameTracker::computeFromImu(const Time &start,
		const Time &finish) const {

	const double kRadToDeg = 180.0 / M_PI;
	const auto imuRPY = toRPY(imuTracker_->orientation(start));
	const auto imuYawCorrection = Rigid3d(Eigen::Vector3d::Zero(),
			fromRPY(0.0, 0.0, -imuRPY.z()));
//  std::cout << "Imu orientation: " << toRPY(imuTracker_->orientation(start)).transpose() * kRadToDeg << std::endl;
	const Eigen::Vector3d dp = imuYawCorrection
			* imuTracker_->getLienarPositionChange(start, finish);
	const Eigen::Quaterniond dq = imuTracker_->getOrientationChange(start,
			finish);
//  std::cout << "dp: " << dp.transpose() << std::endl;
//  std::cout << "dq: " << dq.coeffs().transpose() << std::endl;
	return imuToLidar_.inverse() * Rigid3d(dp, dq) * imuToLidar_;
}

}  // namespace icp_loco
