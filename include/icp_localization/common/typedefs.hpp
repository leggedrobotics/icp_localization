/*
 * typedefs.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once
#include "pointmatcher/PointMatcher.h"
#include <pcl/common/common.h>

namespace icp_loco{

using PointMatcher = PointMatcher<float>;
using PM = PointMatcher;
using DataPoints = PM::DataPoints;
using DP = DataPoints;

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

} // namespace icp_loco
