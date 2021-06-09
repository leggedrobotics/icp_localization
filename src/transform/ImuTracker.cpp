/*
 * ImuTracker.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/ImuTracker.hpp"
#include "icp_localization/common/math.hpp"
#include "icp_localization/common/assert.hpp"

namespace icp_loco {

namespace {
const double kRadToDeg = 180.0 / M_PI;
const Eigen::Vector3d gravityVector(0.0, 0.0, 9.806);
}

ImuTracker::ImuTracker() :
		currentOrientation_(Eigen::Quaterniond::Identity()), imuGravityTimeConstant_(
				0.01), currentGravityVector_(gravityVector), integratedPosition_(
				Eigen::Vector3d::Zero()), integratedLinearVelocity_(
				Eigen::Vector3d::Zero()), integratedRPY_(
				Eigen::Vector3d::Zero()),
				tPrev_(fromUniversal(0)){
	integrationBuffer_.reserve(imuBuffer_.size_limit());
}

void ImuTracker::setGravityVectorFilterTimeConstant(double val) {
	imuGravityTimeConstant_ = val;
}

Eigen::Vector3d ImuTracker::getLinearVelocityChange(const Time &start,
		const Time &end) const {
	fillIntegrationBuffer(start, end);
	if (integrationBuffer_.empty()) {
		return Eigen::Vector3d::Zero();
	}
	Accessor accAccessor = [](TimestampedImuReading &r) -> Eigen::Vector3d& {
		return r.imu_.acceleration();
	};
	integrateBufferInPlace(accAccessor, Eigen::Vector3d::Zero());
	return accAccessor(integrationBuffer_.back());
}
Eigen::Quaterniond ImuTracker::getOrientationChange(const Time &start,
		const Time &end) const {
	fillIntegrationBuffer(start, end);
	if (integrationBuffer_.empty()) {
		return Eigen::Quaterniond::Identity();
	}
	Accessor angVelAccessor = [](TimestampedImuReading &r) -> Eigen::Vector3d& {
		return r.imu_.angularVelocity();
	};
	integrateBufferInPlace(angVelAccessor, Eigen::Vector3d::Zero());
	const auto dRPY = angVelAccessor(integrationBuffer_.back());
//  std::cout << "d RPY from tracker: " << dRPY.transpose() * 180.0/M_PI << std::endl;
	return fromRPY(dRPY.x(), dRPY.y(), dRPY.z()).normalized();
}
Eigen::Vector3d ImuTracker::getLienarPositionChange(const Time &start,
		const Time &end, const Eigen::Vector3d &v0) const {
	fillIntegrationBuffer(start, end);
	if (integrationBuffer_.empty()) {
		return Eigen::Vector3d::Zero();
	}
	Accessor accAccessor = [](TimestampedImuReading &r) -> Eigen::Vector3d& {
		return r.imu_.acceleration();
	};
//  printBuffer(accAccessor);
	removeGravityAccelerationFromBuffer();
//  printBuffer(accAccessor);
//  std::cout << "\n";
	integrateBufferInPlace(accAccessor, v0);
	integrateBufferInPlace(accAccessor, Eigen::Vector3d::Zero());

	return accAccessor(integrationBuffer_.back());
}

ImuReadingd ImuTracker::lookup(const Time &t) const {
	return imuBuffer_.lookup(t);
}
void ImuTracker::addReading(const Time &t, const ImuReadingd &reading) {
	imuBuffer_.push(t, reading);
	updateOrientationFromLinearAcceleration();
	integrateImuReading(t, reading);
}

void ImuTracker::integrateImuReading(const Time &t,
		const ImuReadingd &reading) {
	if (t < tPrev_) {
		return;
	}
	if (toUniversal(tPrev_) == 0){
		tPrev_ = t;
	}

	const double dt = toSeconds(t - tPrev_); // prevent integrating backwards
	integratedPosition_ += dt * integratedLinearVelocity_;
	integratedLinearVelocity_ += dt
			* removeGravity(reading.acceleration(), currentOrientation_);
	integratedRPY_ += dt * reading.angularVelocity();

	integratedPoses_.push(t,
			Rigid3d(integratedPosition_, fromRPY(integratedRPY_)));

	tPrev_ = t;
}

Rigid3d ImuTracker::getLatestOdometry() const {
	const auto rpyAccelerometer = toRPY(currentOrientation_);
	return Rigid3d(integratedPosition_, fromRPY(integratedRPY_.x(), integratedRPY_.y(), integratedRPY_.z()));
}
void ImuTracker::setInitialPose(const Eigen::Vector3d &p,
		const Eigen::Quaterniond &q) {
	integratedPosition_ = p;
	integratedRPY_ = toRPY(q);
}

const ImuInterpolationBuffer& ImuTracker::getBuffer() const {
	return imuBuffer_;
}

void ImuTracker::fillIntegrationBuffer(const Time &start,
		const Time &end) const {
	if (imuBuffer_.empty()) {
		return;
	}
	const auto startTime =
			start < imuBuffer_.earliest_time() ?
					imuBuffer_.earliest_time() : start;
	const auto endTime =
			end > imuBuffer_.latest_time() ? imuBuffer_.latest_time() : end;
//  std::cout << "size of imu buffer: " << imuBuffer_.size() << std::endl;
	imuBuffer_.getRawReadings(startTime, endTime, &integrationBuffer_);

	//debugging example DO NOT USE
//  integrationBuffer_.clear();
//  for (int i=0; i < 10; ++i){
//    TimestampedImuReading reading;
//    reading.time_ = Time(fromSeconds(i));
//    reading.imu_ = ImuReadingd(Eigen::Vector3d::Ones(),Eigen::Vector3d::Ones());
//    integrationBuffer_.push_back(reading);
//  }
}

void ImuTracker::removeGravityAccelerationFromBuffer() const {

	for (int i = 0; i < integrationBuffer_.size(); ++i) {
		auto &imu = integrationBuffer_.at(i).imu_;
		//imu.acceleration() -= imu.rotation().inverse() * gravityVector;
		imu.acceleration() = removeGravity(imu.acceleration(), imu.rotation());
	}

}

Eigen::Vector3d ImuTracker::removeGravity(const Eigen::Vector3d &acc,
		const Eigen::Quaterniond &imuOrientation) const {
	return acc - imuOrientation.inverse() * gravityVector;
}

void ImuTracker::integrateBufferInPlace(Accessor access,
		const Eigen::Vector3d &init) const

		{
	if (integrationBuffer_.empty()) {
		return;
	}
	Eigen::Vector3d cumSum = init;
	for (int i = 1; i < integrationBuffer_.size(); ++i) {
		const double dt = toSeconds(
				integrationBuffer_.at(i).time_
						- integrationBuffer_.at(i - 1).time_);
		const auto measurement = access(integrationBuffer_.at(i - 1));
		access(integrationBuffer_.at(i - 1)) = cumSum;
		cumSum += measurement * dt;
	}
	access(integrationBuffer_.back()) = cumSum;
}

void ImuTracker::printBuffer(Accessor accessor) const {
	for (int i = 0; i < integrationBuffer_.size(); ++i) {
		std::cout << accessor(integrationBuffer_.at(i)).transpose() << ", ";
	}
	std::cout << std::endl;
}

void ImuTracker::updateOrientationFromLinearAcceleration() {
	const auto &imuReading = imuBuffer_.latest_measurement();
	const bool isSingleReading = imuBuffer_.size() < 2;
	const double dt =
			isSingleReading ?
					std::numeric_limits<double>::infinity() :
					toSeconds(
							imuReading.time_
									- imuBuffer_.latest_measurement(2).time_);
	assert_ge(dt, 0.0);

	const double alpha = 1.0 - std::exp(-dt / imuGravityTimeConstant_);
	currentGravityVector_ = (1.0 - alpha) * currentGravityVector_
			+ alpha * imuReading.imu_.acceleration();
	// Change the 'orientation_' so that it agrees with the current
	// 'gravity_vector_'.
	const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
			currentGravityVector_,
			currentOrientation_.conjugate() * Eigen::Vector3d::UnitZ());
	currentOrientation_ = (currentOrientation_ * rotation).normalized();
	assert_gt((currentOrientation_ * currentGravityVector_).z(), 0.);
	assert_gt((currentOrientation_ * currentGravityVector_).normalized().z(),
			0.99);
	imuBuffer_.latest_measurement().imu_.rotation() = currentOrientation_;

// some debug printouts
//  std::cout << "Imu tracker, dt: " << dt << std::endl;
//  std::cout << "Acceleration: " << imuReading.imu_.acceleration().transpose() << "\n";
//  std::cout << "Gravity vector: " << currentGravityVector_.transpose() << "\n";
//  std::cout << "Orientation: " << currentOrientation_.coeffs().transpose() << std::endl;
//  std::cout << "current RPY: " << toRPY(currentOrientation_).transpose() * kRadToDeg << std::endl;
//  std::cout << "\n";

}

Eigen::Quaterniond ImuTracker::orientation(const Time &t) const {
	if (!imuBuffer_.has(t)) {
		return Eigen::Quaterniond::Identity();
	}
	return imuBuffer_.lookup(t).rotation();
}

}  // namespace icp_loco
