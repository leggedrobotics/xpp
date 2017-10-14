/**
 @file    q_hyq.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_

#include <map>

#include <xpp_states/endeffectors.h>

namespace xpp {
namespace mono {

// joint order for 3DoF leg
enum MonopedJointID {HAA, HFE, KFE, KNumJoints};

//static const std::map<EndeffectorID, std::vector<MonopedJointID> > kMapMonoEEToJoints {
//  { E0, {HAA, HFE, KFE} },
//};

} /* namespace mono */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
