/*
 * TfPublisher.cpp
 *
 *  Created on: Apr 27, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/TfPublisher.hpp"
#include "icp_localization/transform/FrameTracker.hpp"
#include "icp_localization/transform/ImuTracker.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include "icp_localization/common/assert.hpp"

namespace icp_loco {

namespace {

constexpr bool isDebug = false;

geometry_msgs::TransformStamped toRos(const Rigid3d &T, const Time &time,
		const std::string &frame, const std::string &childFrame) {
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = toRos(time);
	transformStamped.header.frame_id = frame;
	transformStamped.child_frame_id = childFrame;
	tf::vectorEigenToMsg(T.translation(),
			transformStamped.transform.translation);
	tf::quaternionEigenToMsg(T.rotation(), transformStamped.transform.rotation);
	return transformStamped;
}

}  // namespace

TfPublisher::TfPublisher(const ros::NodeHandle &nh,
		std::shared_ptr<FrameTracker> frameTracker,
		std::shared_ptr<ImuTracker> imuTracker) :
		nh_(nh), frameTracker_(frameTracker), imuTracker_(imuTracker), initPosition_(
				Eigen::Vector3d::Zero()), initOrientation_(
				Eigen::Quaterniond::Identity()) {

}

void TfPublisher::setOdometryTopic(const std::string &topic) {
	odometryTopic_ = topic;
}

void TfPublisher::setImuTopic(const std::string &topic) {
	imuTopic_ = topic;
}

void TfPublisher::initialize() {

	odomSubscriber_ = nh_.subscribe(odometryTopic_, 1,
			&TfPublisher::odometryCallback, this);
	imuSubscriber_ = nh_.subscribe(imuTopic_, 1, &TfPublisher::imuCallback,
			this);

}

void TfPublisher::publishMapToRangeSensor(const Time &t) {
	const auto mapToLidar = frameTracker_->getTransformMapToRangeSensor(t);
	geometry_msgs::TransformStamped transformStamped = toRos(mapToLidar, t,
			"map", "range_sensor");
	tfBroadcaster_.sendTransform(transformStamped);
}

void TfPublisher::setIsProvideOdomFrame(bool value) {
	isProvideOdomFrame_ = value;
}

void TfPublisher::setIsUseOdometry(bool value){
	isUseOdometry_ = value;
}

void TfPublisher::setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q){
	initPosition_ = p;
	initOrientation_ = q;
}

void TfPublisher::publishMapToOdom(const Time &t) {
	const auto mapToOdom = frameTracker_->getTransformMapToOdom(t);
	geometry_msgs::TransformStamped transformStamped = toRos(mapToOdom, t,
			"map", "odom");
	tfBroadcaster_.sendTransform(transformStamped);
	//just for debug

	if (isDebug) {
		const auto mapToLidar = frameTracker_->getTransformMapToRangeSensor(t);
		transformStamped = toRos(mapToLidar, t, "map", "rs_check");
		tfBroadcaster_.sendTransform(transformStamped);


		const auto lidarToCam =
				(frameTracker_->getTransformOdomSourceToRangeSensor(t)).inverse();
		geometry_msgs::TransformStamped transformStamped = toRos(lidarToCam, t,
				"rs_check", "os_check");
		tfBroadcaster_.sendTransform(transformStamped);


		const auto check = mapToLidar * lidarToCam;
		transformStamped = toRos(check, t, "map", "rs_os_check");
		tfBroadcaster_.sendTransform(transformStamped);
	}

}

void TfPublisher::odometryCallback(const nav_msgs::Odometry &msg) {
	Eigen::Vector3d t;
	Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg.pose.pose.orientation, q);
	tf::pointMsgToEigen(msg.pose.pose.position, t);
	TimestampedTransform odomToTracking;
	odomToTracking.transform_ = Rigid3d(t, q);
	odomToTracking.time_ = fromRos(msg.header.stamp);
	frameTracker_->setTransformOdomToOdomSource(odomToTracking);

	if (isProvideOdomFrame_) {
		geometry_msgs::TransformStamped transformStamped = toRos(
				odomToTracking.transform_, odomToTracking.time_, "odom",
				"odom_source");
		tfBroadcaster_.sendTransform(transformStamped);

		const auto cameraToLidar =
				frameTracker_->getTransformOdomSourceToRangeSensor(
						odomToTracking.time_);
		transformStamped = toRos(cameraToLidar,
				odomToTracking.time_, "odom_source", "range_sensor");
		tfBroadcaster_.sendTransform(transformStamped);

		// just visualize raw odometry
		const Rigid3d initPose(initPosition_,initOrientation_);
		const Rigid3d lidarToOdomSource = frameTracker_->getTransformOdomSourceToRangeSensor(Time(fromSeconds(0))).inverse();
		transformStamped = toRos(initPose * lidarToOdomSource,
				odomToTracking.time_, "map", "init_odom_source");
		tfBroadcaster_.sendTransform(transformStamped);
		transformStamped = toRos(
						odomToTracking.transform_, odomToTracking.time_, "init_odom_source",
						"raw_odometry_source");
		tfBroadcaster_.sendTransform(transformStamped);
	}

}

void TfPublisher::imuCallback(const sensor_msgs::Imu &msg) {
	TimestampedImuReading imuReading = fromRos(msg);
	imuTracker_->addReading(imuReading.time_, imuReading.imu_);
//  std::cout <<"Adding imu reading: \n" << imuReading.imu_.asString() << std::endl;
	const auto lidarToImu = frameTracker_->getTransformImuToRangeSensor(
			imuReading.time_).inverse();
	geometry_msgs::TransformStamped transformStamped = toRos(lidarToImu,
			imuReading.time_, "range_sensor", "inertial_sensor");
	tfBroadcaster_.sendTransform(transformStamped);

//	{
//	Rigid3d odomToOdomSource = imuTracker_->getLatestOdometry();
////	std::cout << "IMu odom: " << odomToOdomSource.asString() << std::endl;
//	geometry_msgs::TransformStamped transformStamped = toRos(odomToOdomSource,
//			imuReading.time_, "map", "odom_imu");
//	tfBroadcaster_.sendTransform(transformStamped);
//	}
	//todo I should fuse those two somehow
	if (isProvideOdomFrame_ && !isUseOdometry_) {


	}

//  const auto end = imuTracker_->getBuffer().latest_time();
//  const auto start = end - fromSeconds(1.0);
//  std::cout << "dXYZ: " << imuTracker_->getLienarPositionChange(start, end, Eigen::Vector3d::Ones()).transpose() << std::endl;
//  const auto imuOrientation = imuTracker_->orientation(end);
//  const auto q = imuTracker_->getOrientationChange(start, end) * imuOrientation.inverse();
//  ROS_INFO_STREAM_THROTTLE(1.0, "Imu orientation: " << 180. / M_PI * toRPY(imuOrientation).transpose());
//  std::cout << "d RPY: " << 180. / M_PI * toRPY(q).transpose() << "\n \n";
}

}  // namespace icp_loco
