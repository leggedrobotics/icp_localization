/*
 * TwistIntegrationBuffer.cpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/TwistIntegrationBufferRos.hpp"
#include "icp_localization/common/assert.hpp"

#include <algorithm>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace icp_loco {

TwistIntegrationBufferRos::TwistIntegrationBufferRos(const ros::NodeHandle &nh)
    : nh_(nh)
{

  nh_.param<std::string>("odometry_topic", odometryTopic_, "");

}


void TwistIntegrationBufferRos::setOdometryTopic(const std::string &topic){
  odometryTopic_ = topic;
}

void TwistIntegrationBufferRos::initialize() {

  odomSubscriber_ = nh_.subscribe(odometryTopic_,1,&TwistIntegrationBufferRos::odometryCallback, this);
}

void TwistIntegrationBufferRos::odometryCallback(const nav_msgs::Odometry &msg)
{
  const auto twist = fromRos(msg.twist.twist);

  const auto time = fromRos(msg.header.stamp);

  push(time, twist);

}

}  // namespace icp_loco

