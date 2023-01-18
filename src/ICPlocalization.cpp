/*
 * ICPlocalization.cpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */
#include "icp_localization/ICPlocalization.hpp"
#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/serialization.h"
#include "pointmatcher_ros/deserialization.h"
#include <pointmatcher_ros/StampedPointCloud.h>
#include "pointmatcher/IO.h"
#include <thread>
#include "icp_localization/helpers.hpp"
#include "icp_localization/RangeDataAccumulator.hpp"
#include "icp_localization/transform/TfPublisher.hpp"
#include "icp_localization/transform/FrameTracker.hpp"
#include "icp_localization/transform/ImuTracker.hpp"

namespace icp_loco {

namespace {
const double kRadToDeg = 180.0 / M_PI;
}

ICPlocalization::ICPlocalization(const ros::NodeHandle &nh) :
		nh_(nh), rangeDataAccumulator_(nh) {
	initializeInternal();
	auto callable = [this]() {
		icpWorker();
	};
	icpWorker_ = std::thread(callable);
}

ICPlocalization::~ICPlocalization() {
	icpWorker_.join();
	Rigid3d lastPose(lastPosition_, lastOrientation_);
	std::cout << "ICP_LOCO: Last transform map to range sensor: \n";
	std::cout << lastPose.asString() << "\n";
}

void ICPlocalization::setMapCloud(const Pointcloud &map) {
	mapCloud_ = map;
	refCloud_ = fromPCL(map);
	isMapSet_ = true;
	icp_.setMap(refCloud_);
	std::cout << "Map size is: " << mapCloud_.points.size() << std::endl;
}

void ICPlocalization::setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q) {
	std::cout << "Init pose set to, xyz: " << p.transpose();
	std::cout << ", q: " << q.coeffs().transpose() << ", rpy: " << kRadToDeg * icp_loco::toRPY(q).transpose()
			<< " deg \n";
	lastPosition_ = p;
	lastOrientation_ = q;
	imuTracker_->setInitialPose(p, q);
	tfPublisher_->setInitialPose(p, q);
}

void ICPlocalization::set2DPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
	try {
		geometry_msgs::Pose pose_received = msg->pose.pose;
		tf::pointMsgToEigen(msg->pose.pose.position, userSetPosition_);
		tf::quaternionMsgToEigen(pose_received.orientation, userSetQuaternion_);
		isSetPoseFromUser_ = true;
	} catch (const std::exception &e) {
		std::cerr << "Caught exception while setting 2D pose: " << e.what() << '\n';
	}
	std::cout << "User set pose to :" << Rigid3d(userSetPosition_,userSetQuaternion_).asString() << std::endl;

}

void ICPlocalization::initializeInternal() {

	imuTracker_ = std::make_shared<ImuTracker>();
	frameTracker_ = std::make_shared<FrameTracker>(imuTracker_);
	tfPublisher_ = std::make_shared<TfPublisher>(nh_, frameTracker_, imuTracker_);

	lastPosition_.setZero();
	lastOrientation_.setIdentity();

	registeredCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("registered_cloud", 1, true);
	posePub_ = nh_.advertise<geometry_msgs::PoseStamped>("range_sensor_pose", 1, true);
	icp_.setDefault();

	// Initialite tf listener
	tfBuffer_.reset(new tf2_ros::Buffer(ros::Duration(10)));
	tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
}

void ICPlocalization::initialize() {

	const std::string configFileIcp = nh_.param<std::string>("icp_config_path", "");
	try {
		std::ifstream in(configFileIcp);
		if (!in.is_open()) {
			throw std::runtime_error("config file icp opening failed");
		}
		icp_.loadFromYaml(in);
		in.close();
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
	}

	const std::string configFileFilters = nh_.param<std::string>("input_filters_config_path", "");
	try {
		std::ifstream in(configFileFilters);
		if (!in.is_open()) {
			throw std::runtime_error("config file filters opening failed");
		}
		inputFilters_ = PM::DataPointsFilters(in);
		in.close();
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
	}

	std::string rangeDataTopic = "";
	if (!nh_.param<std::string>("icp_localization/range_data_topic", rangeDataTopic, "")) {
		ROS_ERROR_STREAM("failed to load range data topic");
	}
	std::string imuDataTopic = "";
	if (!nh_.param<std::string>("icp_localization/imu_data_topic", imuDataTopic, "")) {
		ROS_ERROR_STREAM("failed to load imu data topic");
	}
	std::string odometryDataTopic = "";
	if (!nh_.param<std::string>("icp_localization/odometry_data_topic", odometryDataTopic, "")) {
		ROS_ERROR_STREAM("failed to load odometry data topic");
	}

	std::cout << "odometry data topic: " << odometryDataTopic << std::endl;
	std::cout << "imu data topic: " << imuDataTopic << std::endl;

	isUseOdometry_ = nh_.param<bool>("icp_localization/is_use_odometry", false);
	std::cout << "Is use odometry: " << std::boolalpha << isUseOdometry_ << "\n";

	tfPublisher_->setOdometryTopic(odometryDataTopic);
	tfPublisher_->setImuTopic(imuDataTopic);
	tfPublisher_->setIsProvideOdomFrame(isUseOdometry_);

	const double gravityVectorFilterTimeConstant = nh_.param<double>(
			"icp_localization/gravity_vector_filter_time_constant", 0.01);
	std::cout << "Gravity vector filter time constant: " << gravityVectorFilterTimeConstant << "\n";
	imuTracker_->setGravityVectorFilterTimeConstant(gravityVectorFilterTimeConstant);

	const std::string imuLidarPrefix = "calibration/imu_to_range_sensor/";
	const Rigid3d imuToRangeSensor = Rigid3d(getPositionFromParameterServer(nh_, imuLidarPrefix),
			getOrientationFromParameterServer(nh_, imuLidarPrefix));

	const std::string odometrySourceLidarPrefix = "calibration/odometry_source_to_range_sensor/";
	const Rigid3d odometrySourceToRangeSensor = Rigid3d(getPositionFromParameterServer(nh_, odometrySourceLidarPrefix),
			getOrientationFromParameterServer(nh_, odometrySourceLidarPrefix));
	frameTracker_->setTransformOdometrySourceToRangeSensor(odometrySourceToRangeSensor);
	frameTracker_->setTransformImuToRangeSensor(imuToRangeSensor);
	frameTracker_->setIsUseOdometryForRangeSensorPosePrediction(isUseOdometry_);

	const int minNumOdomMeasurements = nh_.param<int>("icp_localization/min_num_odom_msgs_before_ready", 300);

	frameTracker_->setMinNumOdomMeasurementsBeforeReady(minNumOdomMeasurements);
	std::cout << "Min num odom measurements before ready: " << minNumOdomMeasurements << std::endl;

	std::cout << "Calibration: \n";
	std::cout << "imu to range sensor: " << imuToRangeSensor.asString() << "\n\n";
	std::cout << "odometry source to range sensor: " << odometrySourceToRangeSensor.asString() << "\n\n";

	RangeDataAccumulatorParamRos rangeDataAccParam;
	rangeDataAccParam.numAccumulatedRangeData_ = nh_.param<int>("icp_localization/num_accumulated_range_data", 1);
	rangeDataAccParam.inputRangeDataTopic_ = nh_.param<std::string>("icp_localization/range_data_topic", "");

	std::cout << "range data parameters: \n";
	std::cout << "topic: " << rangeDataAccParam.inputRangeDataTopic_ << "\n";
	std::cout << "num range data accumulated: " << rangeDataAccParam.numAccumulatedRangeData_ << "\n \n";

	fixedFrame_ = nh_.param<std::string>("icp_localization/fixed_frame", "map");
	std::cout << "Setting fixed frame to: " << fixedFrame_ << std::endl;

	rangeDataAccumulator_.setParam(rangeDataAccParam);
	rangeDataAccumulator_.initialize();
	tfPublisher_->initialize();

	std::cout << "ICPlocalization: Initialized \n";

	// Set 2D pose subscribe
	initialPose_ = nh_.subscribe("/initialpose", 1, &ICPlocalization::set2DPoseCallback, this);
}

DP ICPlocalization::fromPCL(const Pointcloud &pcl) {
	//todo this can result in data loss???
	sensor_msgs::PointCloud2 ros;
	pcl::toROSMsg(pcl, ros);
	return pointmatcher_ros::rosMsgToPointMatcherCloud<float>(ros);
}

const std::string& ICPlocalization::getFixedFrame() const {
	return fixedFrame_;
}

void ICPlocalization::matchScans() {
	if (!icp_.hasMap()) {
		return;
	}

	Eigen::Vector3d initPosition = lastPosition_;
	Eigen::Quaterniond initOrientation = lastOrientation_;
	Rigid3d before(lastPosition_, lastOrientation_);
	if (!isFirstScanMatch_) {
		const Rigid3d lastPose(initPosition, initOrientation);
		const Rigid3d motionPoseChange = frameTracker_->getPoseChangeOfRangeSensorInMapFrame(
				lastOptimizedPoseTimestamp_, regCloudTimestamp_);
		const Rigid3d motionCorrectedPose = Rigid3d(initPosition, initOrientation) * motionPoseChange;
		initPosition = motionCorrectedPose.translation();
		initOrientation = motionCorrectedPose.rotation();
//    std::cout << "Prediction diff: " << motionPoseChange.asString() << std::endl;
	}

	PM::TransformationParameters initPose;
// Compute the transformation to express data in ref
	inputFilters_.apply(regCloud_);
//	optimizedPose_ = initPose;
	if (!isSetPoseFromUser_) {
		try {
			initPose = getTransformationMatrix<float>(toFloat(initPosition), toFloat(initOrientation));
			optimizedPose_ = icp_(regCloud_, initPose);
		} catch (const std::exception &e) {
			std::cerr << "Caught exception while scan matching: " << e.what() << std::endl;
			optimizedPose_ = initPose;
		}
	} else {
		optimizedPose_ = getTransformationMatrix<float>(toFloat(userSetPosition_), toFloat(userSetQuaternion_));
		isSetPoseFromUser_ = false;
	}

	optimizedPoseTimestamp_ = regCloudTimestamp_;
	getPositionAndOrientation<double>(toDouble(optimizedPose_), &lastPosition_, &lastOrientation_);

	//limit translation if too big
//  Rigid3d poseDiffIcp = before.inverse()*Rigid3d(lastPosition_, lastOrientation_);
//  auto &t = poseDiffIcp.translation();
//  const Eigen::Vector3d limitdXYZ(0.25,0.25,0.25);
//  t.x() = std::fabs(t.x()) > limitdXYZ.x() ? limitdXYZ.x() : t.x();
//  t.y() = std::fabs(t.y()) > limitdXYZ.y() ? limitdXYZ.y() : t.y();
//  t.z() = std::fabs(t.z()) > limitdXYZ.z() ? limitdXYZ.z() : t.z();
//  lastPosition_ = (before * poseDiffIcp).translation();

//  std::cout << "Ground truth diff: " << poseDiffIcp.asString() << "\n\n";

	frameTracker_->setTransformMapToRangeSensor(
			TimestampedTransform { optimizedPoseTimestamp_, Rigid3d(lastPosition_, lastOrientation_) });
	lastOptimizedPoseTimestamp_ = optimizedPoseTimestamp_;

	isFirstScanMatch_ = false;

}

void ICPlocalization::publishPose() const {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.pose = pointmatcher_ros::eigenMatrixToPoseMsg<float>(optimizedPose_);
	pose_msg.header.frame_id = fixedFrame_;
	pose_msg.header.stamp = toRos(optimizedPoseTimestamp_);
	pose_msg.header.seq = seq_++;
	posePub_.publish(pose_msg);

	if (isUseOdometry_) {
		tfPublisher_->publishMapToOdom(optimizedPoseTimestamp_);
	} else {
		tfPublisher_->publishMapToRangeSensor(optimizedPoseTimestamp_);
	}

}

void ICPlocalization::publishRegisteredCloud() const {
	DP data_out(icp_.getReadingFiltered());
	icp_.transformations.apply(data_out, optimizedPose_);
	sensor_msgs::PointCloud2 ros_msg = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(data_out, fixedFrame_,
			ros::Time::now());
	registeredCloudPublisher_.publish(ros_msg);
}

void ICPlocalization::icpWorker() {
	ros::Rate r(100);
	while (ros::ok()) {
		if (!rangeDataAccumulator_.isAccumulatedRangeDataReady() || !frameTracker_->isReady()) {
			r.sleep();
			continue;
		}
		regCloudTimestamp_ = rangeDataAccumulator_.getAccumulatedRangeDataTimestamp();
		regCloud_ = rangeDataAccumulator_.popAccumulatedRangeData().data_;
		namespace ch = std::chrono;
		const auto startTime = ch::steady_clock::now();
		matchScans();
		const auto endTime = ch::steady_clock::now();
		const unsigned int nUs = ch::duration_cast<ch::microseconds>(endTime - startTime).count();
		const double timeMs = nUs / 1000.0;
//    std::string infoStr = "Scan matching took: " + std::to_string(timeMs) + " ms \n";
//    ROS_INFO_STREAM(infoStr);

//    ROS_INFO_STREAM_THROTTLE(10.0, "Scan matching took: " << timeMs << " ms");

		publishPose();
		publishRegisteredCloud();

		r.sleep();
	}
}

}  // namespace icp_loco
