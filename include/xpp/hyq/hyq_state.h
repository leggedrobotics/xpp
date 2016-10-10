/**
@file    hyq_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#ifndef _XPP_HYQ_STATE_H_
#define _XPP_HYQ_STATE_H_

#include <xpp/hyq/foothold.h>
#include <xpp/hyq/declarations.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/utils/base_state.h>

namespace xpp {
namespace hyq {


/** Captures the full state of the robot (body, feet)
  */
class HyqState {
public:
  typedef utils::BaseState Pose;
  typedef utils::BaseLin3d Point3d;
  typedef Eigen::Vector3d Vector3d;
  typedef std::vector<Foothold> VecFoothold;

  HyqState();
  virtual ~HyqState();

  LegDataMap< bool > swingleg_;
  LegDataMap<Point3d> feet_;
  Pose base_; // geometric center of mass, vel, acc

  LegDataMap< Foothold > FeetToFootholds() const;
  Foothold FootToFoothold(LegID leg) const;
  VecFoothold GetStanceLegs() const;

  const LegDataMap<Vector3d> GetFeetPosOnly();

  void SetSwingleg(LegID leg);
  std::array<Vector3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;
  void ZeroVelAcc();
  int SwinglegID() const;
};

class HyqStateStamped : public HyqState {
public:
  double t_;
};

/** State of HyQ represented by base state and joint angles
  *
  */
class HyQStateJoints {
public:
  using State3d    = xpp::utils::BaseState;
  using JointState = xpp::hyq::JointState;

  JointState joints_;
  State3d base_;
};


inline std::ostream& operator<<(std::ostream& out, const HyqState& hyq)
{
  out << "base: " << hyq.base_ << "\n"
      << "feet: " << "\tLF = " <<  hyq.feet_[LF] << "\n"
                  << "\tRF = " <<  hyq.feet_[RF] << "\n"
                  << "\tLH = " <<  hyq.feet_[LH] << "\n"
                  << "\tRH = " <<  hyq.feet_[RH] << "\n"
      << "swing:\t" << hyq.swingleg_ << "\n";
   return out;
}


} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_STATE_H_
