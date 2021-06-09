/*
 * ImuInterpolationBuffer.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#include "icp_localization/transform/ImuInterpolationBuffer.hpp"
#include "icp_localization/common/assert.hpp"

#include <algorithm>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace icp_loco {

namespace {
const int kBuffSize = 1500; //5000;
}

ImuInterpolationBuffer::ImuInterpolationBuffer()
{
  setSizeLimit(kBuffSize);
}

void ImuInterpolationBuffer::push(const Time &time, const ImuReadingd& m)
{

  //this relies that they will be pushed in order!!!
  if (!measurements_.empty()) {
    if (time < earliest_time()) {
      std::cerr
          << "ImuInterpolationBuffer:: you are trying to push something earlier than the earliest measurement, this should not happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << time << std::endl;
      std::cerr << "earliest time: " << earliest_time() << std::endl;
      return;
    }

    if (time < latest_time()) {
      std::cerr << "ImuInterpolationBuffer:: you are trying to push something out of order, this should not happen \n";
      std::cerr << "ingnoring the mesurement \n";
      std::cerr << "Time: " << time << std::endl;
      std::cerr << "latest time: " << latest_time() << std::endl;
      return;
    }
  }
  measurements_.push_back( { time, m });
  removeOldMeasurementsIfNeeded();
}

void ImuInterpolationBuffer::setSizeLimit(const size_t buffer_size_limit)
{

  bufferSizeLimit_ = buffer_size_limit;
  removeOldMeasurementsIfNeeded();
}

void ImuInterpolationBuffer::clear()
{

  measurements_.clear();
}

bool ImuInterpolationBuffer::has(const Time &time) const
{
  if (measurements_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

ImuReadingd ImuInterpolationBuffer::lookup(const Time &time) const
{

  if (!has(time)) {
    throw std::runtime_error("ImuInterpolationBuffer:: Missing measurement for: " + toString(time));
  }

  if (size() == 1) {
    return measurements_.front().imu_;
  }

  //just return the closest
  const auto gtMeasurement = std::find_if(measurements_.begin(), measurements_.end(),
                                          [&time](const TimestampedImuReading& m) {
                                            return time <= m.time_;
                                          });
  const bool isIteratorValid = gtMeasurement != measurements_.end();
  if (isIteratorValid && gtMeasurement->time_ == time) {
    return gtMeasurement->imu_;
  }
  const auto start = std::prev(gtMeasurement);

  return interpolate(*start, *gtMeasurement, time).imu_;
}

void ImuInterpolationBuffer::removeOldMeasurementsIfNeeded()
{
  while (measurements_.size() > bufferSizeLimit_) {
    measurements_.pop_front();
  }
}

Time ImuInterpolationBuffer::earliest_time() const
{
  if (empty()) {
    throw std::runtime_error("ImuInterpolationBuffer:: Empty buffer");
  }
  return measurements_.front().time_;
}

Time ImuInterpolationBuffer::latest_time() const
{
  if (empty()) {
    throw std::runtime_error("ImuInterpolationBuffer:: Empty buffer");
  }
  return measurements_.back().time_;
}

bool ImuInterpolationBuffer::empty() const
{
  return measurements_.empty();
}

size_t ImuInterpolationBuffer::size_limit() const
{
  return bufferSizeLimit_;
}

const TimestampedImuReading &ImuInterpolationBuffer::latest_measurement(int n) const {
  if (empty()) {
    throw std::runtime_error("ImuInterpolationBuffer:: latest_measurement: Empty buffer");
  }
  return *(std::prev(measurements_.end(),n));
}

TimestampedImuReading &ImuInterpolationBuffer::latest_measurement(int n) {
  if (empty()) {
    throw std::runtime_error("ImuInterpolationBuffer:: latest_measurement: Empty buffer");
  }
  return *(std::prev(measurements_.end(),n));
}

size_t ImuInterpolationBuffer::size() const
{
  return measurements_.size();
}

void ImuInterpolationBuffer::printTimesCurrentlyInBuffer() const
{
  for (auto it = measurements_.cbegin(); it != measurements_.cend(); ++it) {
    std::cout << readable(it->time_) << std::endl;
  }
}

void ImuInterpolationBuffer::getRawReadings(const Time &start, const Time &end,
                                            std::vector<TimestampedImuReading> *v) const
{
  if (!has(start)) {
    throw std::runtime_error("ImuInterpolationBuffer::getRawReadings: Missing measurement for: " + toString(start));
  }

  if (!has(end)) {
    throw std::runtime_error("ImuInterpolationBuffer::getRawReadings: Missing measurement for: " + toString(end));
  }

  assert_ge(toUniversal(end), toUniversal(start));

  auto isInbetween = [&start, &end](const TimestampedImuReading& m) {
    return m.time_ >= start && m.time_ <=end;
  };
  v->clear();
  std::copy_if(measurements_.begin(), measurements_.end(),std::back_inserter(*v),isInbetween);
}

}  // namespace icp_loco

