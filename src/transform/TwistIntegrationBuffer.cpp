/*
 * TwistIntegrationBuffer.cpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/TwistIntegrationBuffer.hpp"
#include "icp_localization/common/assert.hpp"

#include <algorithm>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace icp_loco {

namespace {
const int kBuffSize = 5000;
}

TwistIntegrationBuffer::TwistIntegrationBuffer()
{
  setSizeLimit(kBuffSize);
}

void TwistIntegrationBuffer::push(const Time &time, const Twist3d& twist)
{
  //this relies that they will be pushed in order!!!
  if (!twists_.empty()) {
    if (time < earliest_time()) {
      std::cerr << "you are trying to push something earlier than the earliest measurement, this should not happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << time << std::endl;
      std::cerr << "earliest time: " << earliest_time() << std::endl;
//      throw std::runtime_error("your desired twist is ");
      return;
    }

    if (time < latest_time()) {
      std::cerr << "you are trying to push something out of order, this should not happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << time << std::endl;
      std::cerr << "latest time: " << latest_time() << std::endl;
      return;
    }
  }

  twists_.push_front(TimestampedTwist { time, twist });
  removeOldMeasurementsIfNeeded();
}

void TwistIntegrationBuffer::setSizeLimit(const size_t buffer_size_limit)
{
  bufferSizeLimit_ = buffer_size_limit;
  removeOldMeasurementsIfNeeded();
}

void TwistIntegrationBuffer::clear()
{
  twists_.clear();
}

bool TwistIntegrationBuffer::has(const Time &time) const
{
  if (twists_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

Twist3d TwistIntegrationBuffer::lookup(const Time &time) const
{
  if (!has(time)) {
    throw std::runtime_error("Missing transform for: " + toString(time));
  }
  const auto end = std::lower_bound(twists_.begin(), twists_.end(), time,
                                    [](const TimestampedTwist& twist,
                                        const Time time) {
                                      return twist.time_ < time;
                                    });
  if (end->time_ == time) {
    return end->twist_;
  }
  const auto start = std::prev(end);
  return interpolate(*start, *end, time).twist_;
}

Rigid3d TwistIntegrationBuffer::integrate(const Time &startTime, const Time &endTime)
{
  if (empty()) {
    throw std::runtime_error("Empty buffer");
  }

  assert_ge(toUniversal(endTime), toUniversal(startTime));

  if (!has(startTime)) {
    std::cout << "requested start time at: " << startTime << std::endl;
    std::cout << "earliest time at: " << earliest_time() << std::endl;
    throw std::runtime_error("Start time is too much in the past");
  }

  // todo compute the contribution from start Time to first mesurement

  const auto lb = std::lower_bound(twists_.begin(), twists_.end(), startTime,
                                   [](const TimestampedTwist& twist,
                                       const Time &time) {
                                     return twist.time_ < time;
                                   });
  const auto ub = std::upper_bound(twists_.begin(), twists_.end(), endTime,
                                   [](const Time& time, const TimestampedTwist& twist) {
                                     return twist.time_ < time;
                                   });

  Eigen::Vector3d dLinear(0., 0., 0.);
  Eigen::Vector3d dAngular(0., 0., 0.);

  //contribution from last measurement until the endTime
  {
    const auto firstMeasurement = lb;
    const double dt = toSeconds(firstMeasurement->time_ - startTime);
    assert_ge(dt, 0.0);
    dLinear += dt * firstMeasurement->twist_.linear();
    dAngular += dt * firstMeasurement->twist_.angular();
  }

  for (auto it = lb; it != ub; ++it) {
    const auto left = it;
    const auto right = std::next(left);
    const double dt = toSeconds(right->time_ - left->time_);
    assert_ge(dt, 0.);
    dLinear += dt * left->twist_.linear();
    dAngular += dt * right->twist_.angular();
  }

  //contribution from last measurement until the endTime
  {
    const auto lastMeasurement = std::prev(ub);
    const double dt = toSeconds(endTime - lastMeasurement->time_);
    assert_ge(dt, 0.0);
    dLinear += dt * lastMeasurement->twist_.linear();
    dAngular += dt * lastMeasurement->twist_.angular();
  }

  return Rigid3d(dLinear, fromRPY(dAngular.x(), dAngular.y(), dAngular.z()));

}

void TwistIntegrationBuffer::removeOldMeasurementsIfNeeded()
{
  while (twists_.size() > bufferSizeLimit_) {
    twists_.pop_back();
  }
}

Time TwistIntegrationBuffer::earliest_time() const
{
  if (empty()) {
    throw std::runtime_error("Empty buffer");
  }
  return twists_.back().time_;
}

Time TwistIntegrationBuffer::latest_time() const
{
  if (empty()) {
    throw std::runtime_error("Empty buffer");
  }
  return twists_.front().time_;
}

bool TwistIntegrationBuffer::empty() const
{
  return twists_.empty();
}

size_t TwistIntegrationBuffer::size_limit() const
{
  return bufferSizeLimit_;
}

size_t TwistIntegrationBuffer::size() const
{
  return twists_.size();
}

}  // namespace icp_loco

