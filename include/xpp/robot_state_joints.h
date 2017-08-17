/**
@file    articulated_robot_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#ifndef XPP_XPP_OPT_ARTICULATED_ROBOT_STATE_H_
#define XPP_XPP_OPT_ARTICULATED_ROBOT_STATE_H_

#include <xpp/a_inverse_kinematics.h>
//#include <xpp/exe/robot_kinematics.h>
#include <xpp/robot_state_cartesian.h>

#include <memory>

namespace xpp {

/** Full-body robot state expressed with joints instead of Cartesian
  * endeffector positions.
  */
class RobotStateJoints : public RobotDataHolder  {
public:
  using StateC       = RobotStateCartesian;
  using StateCVec    = std::vector<StateC>;
  using StateJVec    = std::vector<RobotStateJoints>;

//  using ForKinPtr    = std::shared_ptr<RobotKinematics>;
  using InvKinPtr    = std::shared_ptr<AInverseKinematics>;

  using PosEE        = EndeffectorsPos;
  using VelEE        = EndeffectorsPos;
  using EEID         = EndeffectorID;

public:
  RobotStateJoints(int n_ee, int n_joints_per_ee);
  virtual ~RobotStateJoints();

  int GetJointsPerEE() const { return q.GetNumJointsPerEE(); };
  int GetJointCount()  const { return q.GetNumJoints(); };

  JointValues GetJointPos() const { return q; };
  JointValues GetJointVel() const { return qd; };
  JointValues GetJointAcc() const { return qdd; };

  void SetJointPos(const JointValues& pos) { q = pos; };
  void SetJointVel(const JointValues& vel) { qd = vel; };
  void SetJointAcc(const JointValues& acc) { qdd = acc; };


  void SetJointAngles(const PosEE& ee_W, const InvKinPtr&);
//  StateC ConvertToCartesian(const ForKinPtr&) const;

  static RobotStateJoints FromCartesian(const StateC&, const InvKinPtr&);
  static StateJVec BuildWholeBodyTrajectory(const StateCVec& trajectory_in, const InvKinPtr& ik);

private:
  JointValues q, qd, qdd;

//  PosEE GetEEInWorld(const ForKinPtr&) const;
//  VelEE GetEEInVelWorld(const ForKinPtr&) const;
};

} // namespace xpp

#endif // XPP_XPP_OPT_ARTICULATED_ROBOT_STATE_H_
