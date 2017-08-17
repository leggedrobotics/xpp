/**
 @file    q_hyq.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_

#include <map>

#include <xpp/joint_values.h>

namespace xpp {
namespace mono {

static constexpr int kNumJoints = 3;
static constexpr int kNumEE = 1;
static constexpr int kNumJointsPerLeg = 3;

enum MonopedJointID {HAA, HFE, KFE};

/** @brief relationship between monoped joints and generic joints. */
static const std::map<MonopedJointID, JointID> kMapMonoToXpp {
  { HAA,  J0 },
  { HFE,  J1 },
  { KFE,  J2 },
};

} /* namespace mono */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
