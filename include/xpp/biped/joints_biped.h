/**
 @file    joints_biped.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_BIPED_BIPED_JOINTS_H_
#define XPP_BIPED_BIPED_JOINTS_H_

#include <xpp/endeffectors4.h>
#include <xpp/joint_values.h>

namespace xpp {
namespace biped {

static constexpr int kNumJoints = 6;
static constexpr int kNumEE = 2;
static constexpr int kNumJointsPerLeg = 3;

enum BipedJointID {
  L_HAA,L_HFE,L_KFE,
  R_HAA,R_HFE,R_KFE,
};

static const std::map<FootID, std::vector<BipedJointID> > kMapBipedEEToJoints {
  { L,  {L_HAA, L_HFE, L_KFE} },
  { R,  {R_HAA, R_HFE, R_KFE} },
};

/** @brief Constructs the relationship between hyq joints and generic joints.
  *
  * The joints are ordered according to the the enum xpp::hyq::LegID.
  */
static const std::map<BipedJointID, JointID> kMapBipedToXpp = []
{
    std::map<BipedJointID, JointID> map;

    int j = 0;
    for (int i=0; i<kNumEE; ++i) {
      auto foot = kMapOptToBiped.at(kEEOrder.at(i));
      for (auto biped_joint : kMapBipedEEToJoints.at(foot)) {
        auto xpp_joint = static_cast<JointID>(j++);
        map[biped_joint] = xpp_joint;
      }
    }

    return map;
}();

} /* namespace biped */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
