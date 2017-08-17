/**
 @file    q_hyq.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_H_

#include <xpp/endeffectors.h>

namespace xpp {
namespace quad {

enum QuadJointID {
  LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE,
  RH_HAA,RH_HFE,RH_KFE,RF_HAA,RF_HFE,RF_KFE,
};

static const std::map<EndeffectorID, std::vector<QuadJointID> > kMapQuadEEToJoints {
  { Reverse(kMapOptToQuad).at(LF),  {LF_HAA, LF_HFE, LF_KFE} },
  { Reverse(kMapOptToQuad).at(LH),  {LH_HAA, LH_HFE, LH_KFE} },
  { Reverse(kMapOptToQuad).at(RF),  {RF_HAA, RF_HFE, RF_KFE} },
  { Reverse(kMapOptToQuad).at(RH),  {RH_HAA, RH_HFE, RH_KFE} },
};

} /* namespace quad */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_QUAD_JOINTS_HYQ_H_ */
