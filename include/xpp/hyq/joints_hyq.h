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

/** @brief Holds joint values for HyQ and allow conversions from/to Eigen
  *        in correct order.
  */
class JointsHyq {
public:
  using Vector3d = Eigen::Vector3d;
  using JointVec = Eigen::Matrix<double,kNumJoints,1>;

  explicit JointsHyq (double val = 0.0);
  explicit JointsHyq (const JointValues& q_xpp);
  virtual ~JointsHyq ();

  /** @brief Converts joints angles to an Eigen vector sorted according to
    * endeffector order in enum xpp::hyq::LegID.
    */
  JointVec ToHyqVec() const;

  /** @brief Sets joint values, assuming hyq order according to
    * @param q  Joint values interpreted as order as enum xpp::hyq::LegID.
    */
  void SetFromHyqVec(const JointVec& q);

  /** @returns the joint value of HyQ joint \c joint.
    */
  double& At(HyqJointID joint);
  double At(HyqJointID joint) const;

  /** @brief Gets/Sets the joint values of Leg leg.
    * @param leg  The leg to which the joint values apply.
    * @param q_leg The leg's joint values ordered as in kMapHyqEEToJoints.
    */
  void SetLeg(LegID leg, const Vector3d& q_leg);
  Vector3d GetLeg(LegID leg) const;

  JointValues ToXpp() const;

private:
  JointValues joint_values_;
};

} /* namespace hyq */
} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_HYQ_JOINTS_HYQ_H_ */
