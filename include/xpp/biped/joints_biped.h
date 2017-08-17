/**
 @file    joints_biped.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_BIPED_BIPED_JOINTS_H_
#define XPP_BIPED_BIPED_JOINTS_H_

#include <xpp/endeffectors.h>

namespace xpp {
namespace biped {

enum BipedJointID {
  L_HAA,L_HFE,L_KFE,
  R_HAA,R_HFE,R_KFE,
};

static const std::map<EndeffectorID, std::vector<BipedJointID> > kMapBipedEEToJoints {
  { Reverse(kMapOptToBiped).at(L),  {L_HAA, L_HFE, L_KFE} },
  { Reverse(kMapOptToBiped).at(R),  {R_HAA, R_HFE, R_KFE} },
};

} /* namespace biped */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
