/*
 * Parameters.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <string>
#include <boost/concept_check.hpp>

namespace icp_loco {

struct BaseParam
{
  template<class T>
  const T* as() const
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, BaseParam*>));
    return static_cast<const T*>(this);
  }

  template<class T>
  T* as()
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, BaseParam*>));
    return static_cast<T*>(this);
  }

 protected:
  BaseParam() = default;
  virtual ~BaseParam() = default;
};

struct RangeDataAccumulatorParam : public BaseParam
{
  int numAccumulatedRangeData_ = 32;
};

struct RangeDataAccumulatorParamRos : public RangeDataAccumulatorParam
{
  std::string inputRangeDataTopic_ = "";
  std::string accumulatedRangeDataTopic_ = "assembled_scans";
};

}  // namespace icp_loco
