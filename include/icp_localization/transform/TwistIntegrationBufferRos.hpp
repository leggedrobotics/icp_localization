/*
 * TwistIntegrationBuffer.hpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization/transform/TwistIntegrationBuffer.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace icp_loco {

class TwistIntegrationBufferRos : public TwistIntegrationBuffer {

 public:

  TwistIntegrationBufferRos(const ros::NodeHandle &nh);

  void initialize();

  void setOdometryTopic(const std::string &topic);

 private:

  void odometryCallback(const nav_msgs::Odometry &msg);

  std::string odometryTopic_;
  ros::NodeHandle nh_;
  ros::Subscriber odomSubscriber_;

};



} // namespace icp_loco
