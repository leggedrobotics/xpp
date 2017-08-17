/**
 @file    q_hyq.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#include <xpp/hyq/joints_hyq.h>
#include <map>

namespace xpp {
namespace hyq {

static const std::array<LegID, 4> kHyqLegOrder =
{
  static_cast<LegID>(0),
  static_cast<LegID>(1),
  static_cast<LegID>(2),
  static_cast<LegID>(3)
};

static const std::vector<EndeffectorID> kHyqEEOrder =
{
  kMapQuadToOpt.at(kHyqLegOrder.at(0)),
  kMapQuadToOpt.at(kHyqLegOrder.at(1)),
  kMapQuadToOpt.at(kHyqLegOrder.at(2)),
  kMapQuadToOpt.at(kHyqLegOrder.at(3)),
};

JointsHyq::JointsHyq (double angle) : joint_values_(kNumEE, kNumJointsPerLeg, angle)
{
}

JointsHyq::JointsHyq (const JointValues& q_xpp) : joint_values_(q_xpp)
{
}

JointsHyq::~JointsHyq ()
{
  // TODO Auto-generated destructor stub
}

JointsHyq::JointVec
JointsHyq::ToHyqVec () const
{
  return joint_values_.ToVec(kHyqEEOrder);
}

void
JointsHyq::SetFromHyqVec (const JointVec& q_hyq)
{
  joint_values_.SetFromVec(q_hyq, kHyqEEOrder);
}

JointValues
JointsHyq::ToXpp () const
{
  return joint_values_;
}

JointsHyq::Vector3d
JointsHyq::GetLeg (LegID leg) const
{
  EndeffectorID ee = kMapQuadToOpt.at(leg);
  return joint_values_.At(ee);
}

void
JointsHyq::SetLeg (LegID leg, const Vector3d& q_hyq)
{
  EndeffectorID ee = kMapQuadToOpt.at(leg);
  joint_values_.At(ee) = q_hyq;
}

double&
JointsHyq::At (HyqJointID joint_hyq)
{
  JointID id = kMapHyqToXpp.at(joint_hyq);
  return joint_values_.At(id);
}

double
JointsHyq::At (HyqJointID joint_hyq) const
{
  JointID id = kMapHyqToXpp.at(joint_hyq);
  return joint_values_.At(id);
}

} /* namespace hyq */
} /* namespace xpp */


