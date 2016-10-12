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
// inv_kin think about merging this with HyQStateJoints (DRY)
// and using inverse/forward kinematics as mediator.
class HyqState {
public:
  typedef utils::BaseState Pose;

  HyqState();
  virtual ~HyqState();

  LegDataMap< bool > swingleg_;
  Pose base_; // geometric center of mass, vel, acc

  void ZeroVelAcc();
  int SwinglegID() const;
  void SetSwingleg(LegID leg);
};

class HyqStateEE : public HyqState {
public:
  using Vector3d = Eigen::Vector3d;
  using VecFoothold = std::vector<Foothold>;
  using Point3d = xpp::utils::BaseLin3d;

  // inv_kin only save position, not vel and acc (disregarding anyway)
  LegDataMap<Point3d> feet_;

  LegDataMap< Foothold > FeetToFootholds() const;
  Foothold FootToFoothold(LegID leg) const;
  VecFoothold GetStanceLegs() const;
  const LegDataMap<Vector3d> GetFeetPosOnly();
  std::array<Vector3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;
};


/** State of HyQ represented by base state and joint angles
  */
class HyqStateJoints : public HyqState {
public:
  using JointState = xpp::hyq::JointState;
  using VecFoothold = std::vector<Foothold>;

  HyqStateJoints();
  virtual ~HyqStateJoints();

  VecFoothold GetStanceLegsInWorld() const;

  JointState q, qd, qdd;

private:
  /** Forward kinematics.
    */
  VecFoothold GetStanceLegsInBase() const;
};



inline std::ostream& operator<<(std::ostream& out, const HyqStateEE& hyq)
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
