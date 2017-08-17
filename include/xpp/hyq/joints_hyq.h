/**
 @file    q_hyq.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_

#include <xpp/endeffectors4.h>
#include <xpp/joint_values.h>

namespace xpp {
namespace hyq {

static constexpr int kNumJoints = 12;
static constexpr int kNumEE = 4;
static constexpr int kNumJointsPerLeg = 3;

enum HyqJointID {
  LF_HAA,LF_HFE,LF_KFE,LH_HAA,LH_HFE,LH_KFE,
  RH_HAA,RH_HFE,RH_KFE,RF_HAA,RF_HFE,RF_KFE,
};

static const std::map<LegID, std::vector<HyqJointID> > kMapHyqEEToJoints {
  { LF,  {LF_HAA, LF_HFE, LF_KFE} },
  { LH,  {LH_HAA, LH_HFE, LH_KFE} },
  { RF,  {RF_HAA, RF_HFE, RF_KFE} },
  { RH,  {RH_HAA, RH_HFE, RH_KFE} },
};

/** @brief Constructs the relationship between hyq joints and generic joints.
  *
  * The joints are ordered according to the the enum xpp::hyq::LegID.
  */
// zmp_ only used in URDF visualizer, so maybe make function and reuse for biped
static const std::map<HyqJointID, JointID> kMapHyqToXpp = []
{
    std::map<HyqJointID, JointID> map;

    int j = 0;
    for (int i=0; i<kNumEE; ++i) {
      LegID leg = kMapOptToQuad.at(kEEOrder.at(i));
      for (HyqJointID hyq_joint : kMapHyqEEToJoints.at(leg)) {
        auto xpp_joint = static_cast<JointID>(j++);
        map[hyq_joint] = xpp_joint;
      }
    }

    return map;
}();

} /* namespace hyq */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
