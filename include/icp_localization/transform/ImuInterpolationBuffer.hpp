/*
 * ImuInterpolationBuffer.hpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#pragma once
#include <deque>
#include <limits>
#include <mutex>

#include "icp_localization/common/time.hpp"
#include "icp_localization/transform/ImuReading.hpp"

namespace icp_loco {



// A time-ordered buffer of transforms that supports interpolated lookups.
// Unless explicitly set, the buffer size is unlimited.
class ImuInterpolationBuffer {
 public:
  ImuInterpolationBuffer();
  virtual ~ImuInterpolationBuffer() = default;

  void setSizeLimit(size_t bufferSizeLimit);

  void push(const Time &time, const ImuReadingd& m);

  // Clears the transform buffer.
  void clear();

  // Returns true if an interpolated transform can be computed at 'time'.
  bool has(const Time &time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  ImuReadingd lookup(const Time &time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty. Earliest time is the one that is the closest to Jan 1,1,00
  Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty. Latest time is the one that is the farthest from Jan 1,1,00
  Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

  // Returns the maximum allowed size of the transform buffer.
  size_t size_limit() const;

  // Returns the current size of the transform buffer.
  size_t size() const;

  //follows the convention of std::prev
  const TimestampedImuReading &latest_measurement(int n = 1) const;
  TimestampedImuReading &latest_measurement(int n = 1);

  void printTimesCurrentlyInBuffer() const;

  void getRawReadings(const Time &start, const Time &end, std::vector<TimestampedImuReading> *v) const;

 private:
  void removeOldMeasurementsIfNeeded();
  static constexpr size_t kUnlimitedBufferSize = std::numeric_limits<size_t>::max();

  std::deque<TimestampedImuReading> measurements_;
  size_t bufferSizeLimit_ = kUnlimitedBufferSize;
};

} // namespace icp_loco
