/*
 * Twist.hpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include "icp_localization/common/time.hpp"

#include <geometry_msgs/Twist.h>

namespace icp_loco {

template<typename FloatType>
class Twist3
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vector = Eigen::Matrix<FloatType, 3, 1>;

  Twist3()
      : linear_(Vector::Zero()),
        angular_(Vector::Zero())
  {
  }
  Twist3(const Vector& lin, const Vector& ang)
      : linear_(lin),
        angular_(ang)
  {
  }

  template<typename OtherType>
  Twist3<OtherType> cast() const
  {
    return Twist3<OtherType>(linear_.template cast<OtherType>(), angular_.template cast<OtherType>());
  }

  const Vector& linear() const
  {
    return linear_;
  }
  const Vector& angular() const
  {
    return angular_;
  }

 private:
  Vector linear_;
  Vector angular_;
};

using Twist3d = Twist3<double>;
using Twist3f = Twist3<float>;

struct TimestampedTwist {
  Time time_;
  Twist3d twist_;
};


TimestampedTwist interpolate(const TimestampedTwist& start,
                                 const TimestampedTwist& end,
                                 const Time &time);

Twist3d fromRos(const geometry_msgs::Twist &msg);

}  // namespace icp_loco
