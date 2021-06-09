/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "icp_localization/transform/RigidTransform.hpp"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <glog/logging.h>

namespace icp_loco {

TimestampedTransform interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const Time &time) {

  if (time > end.time_ || time < start.time_){
    std::cout << "Interpolator: \n";
    std::cout << "Start time: " << start.time_ << std::endl;
    std::cout << "End time: " << end.time_ << std::endl;
    std::cout << "Query time: " << time << std::endl;
    throw std::runtime_error("transform interpolate:: query time is not between start and end time");
  }

  const double duration = toSeconds(end.time_ - start.time_);
  const double factor = toSeconds(time - start.time_) / duration;
  const Eigen::Vector3d origin =
      start.transform_.translation() +
      (end.transform_.translation() - start.transform_.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform_.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform_.rotation()));
  return TimestampedTransform{time,Rigid3d(origin, rotation)};
}



}  // namespace icp_loco
