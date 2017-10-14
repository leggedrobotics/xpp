/**
 @file    q_hyq.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_H_

#include <xpp_states/endeffectors.h>

namespace xpp {
namespace quad {

// this is wrong!
// always access by leg id, joint id
enum QuadJointID {
  LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,
  LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE,
};

static const std::map<EndeffectorID, std::vector<QuadJointID> > kMapQuadEEToJoints {
  { kMapIDToEE.at(LF),  {LF_HAA, LF_HFE, LF_KFE} },
  { kMapIDToEE.at(LH),  {LH_HAA, LH_HFE, LH_KFE} },
  { kMapIDToEE.at(RF),  {RF_HAA, RF_HFE, RF_KFE} },
  { kMapIDToEE.at(RH),  {RH_HAA, RH_HFE, RH_KFE} },
};

} /* namespace quad */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_HYQ_H_ */
