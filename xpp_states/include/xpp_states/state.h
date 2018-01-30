/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef _XPP_STATES_STATE_H_
#define _XPP_STATES_STATE_H_

#include <iostream>
#include <Eigen/Dense>

#include <xpp_states/cartesian_declarations.h>

namespace xpp {

class StateLin1d;
class StateLin2d;
class StateLin3d;
class StateAng3d;
class State3d;
class State3dEuler;

using Vector2d    = Eigen::Vector2d;
using Vector3d    = Eigen::Vector3d;
using Vector6d    = Eigen::Matrix<double,6,1>;
using VectorXd    = Eigen::VectorXd;
using Quaterniond = Eigen::Quaterniond;

enum MotionDerivative { kPos=0, kVel, kAcc, kJerk };

/**
 * @brief Represents position, velocity and acceleration in x-dimensions.
 */
class StateLinXd {
public:
  VectorXd p_, v_, a_; ///< position, velocity and acceleration

  /**
   * @brief  Constructs an object of dimensions _dim.
   *
   * Be careful of default value, as zero dimensional object is bound to
   * cause seg-fault at some point.
   */
  explicit StateLinXd(int _dim = 0);

  /**
   * @brief Constructs object with specific position, velocity and acceleration.
   * @param  p  Position of the state.
   * @param  v  Velocity of the state.
   * @param  a  Acceleration of the state.
   *
   * The dimensions are set to the number of rows of p.
   */
  explicit StateLinXd(const VectorXd& p, const VectorXd& v, const VectorXd& a);

  /**
   * @brief Constructs object with position p, zeroing velocity and acc.
   */
  StateLinXd(const VectorXd& p);
  virtual ~StateLinXd() = default;

  /**
   * @brief  Read either position, velocity of acceleration by index.
   * @param  deriv  Index for that specific derivative (pos=0, vel=1, acc=2).
   * @return  Read only n-dimensional position, velocity or acceleration.
   */
  const VectorXd GetByIndex(MotionDerivative deriv) const;

  /**
   * @brief  Read and write either position, velocity of acceleration by index.
   * @param  deriv  Index for that specific derivative (pos=0, vel=1, acc=2).
   * @return  Read/write n-dimensional position, velocity or acceleration.
   */
  VectorXd& GetByIndex(MotionDerivative deriv);

  /**
   * @brief Returns true if this state has all same pos,vel and acc as other.
   */
  bool operator==(const StateLinXd& other) const;

  /**
   * @brief Returns true if just one value in this state differs from other.
   */
  bool operator!=(const StateLinXd& other) const;

  int kNumDim = 0;  ///< the number of dimenions this state represents.
};


// some state classes of explicit dimensions for type safety as well as
// conversions from the general base class.
class StateLin1d  : public StateLinXd {
public:
  StateLin1d() : StateLinXd(1) {};
  virtual ~StateLin1d() {};
};

class StateLin2d  : public StateLinXd {
public:
  StateLin2d() : StateLinXd(2) {};
  virtual ~StateLin2d() {};
};

class StateLin3d : public StateLinXd {
public:
  StateLin3d() : StateLinXd(3) {};
  StateLin3d(const StateLinXd&);
  virtual ~StateLin3d() {};

  /**
   * @brief Extracts only the 2-dimensional part (x,y) from this 3-D state.
   */
  StateLin2d Get2D() const;
};


/**
 * @brief Angular state of an object in 3-dimensional space.
 *
 * The orientation is expressed as a Quaternion, whereas velocities and
 * acceleration are 3D-vectors.
 */
class StateAng3d {
public:
  Quaterniond q;  ///< orientation expressed as Quaternion.
  Vector3d    w;  ///< angular velocity (omega).
  Vector3d   wd;  ///< angular acceleration (omega dot).

  /**
   * @brief Builds an angular state.
   * @param  _q   orientation expressed as Quaternion.
   * @param  _w   angular velocity (omega).
   * @param  _wd  angular acceleration (omega dot).
   *
   * The conventions (Quaterion should map world to base or vice versa,
   * angular vel/acc expressed in base or world frame) can be chosen by the
   * user.
   */
  explicit StateAng3d(Quaterniond _q = Quaterniond(1.0, 0.0, 0.0, 0.0),
                      Vector3d   _w  = Vector3d::Zero(),
                      Vector3d   _wd = Vector3d::Zero())
  : q(_q), w(_w), wd(_wd) {}
};


/**
 * @brief 6D-State (linear+angular) of an object in 3-dimensional space,
 * where the angular part is expressed by a Quaternion.
 */
class State3d {
public:
  StateLin3d lin; ///< linear position, velocity and acceleration
  StateAng3d ang; ///< Quaternion, velocity and acceleration

  Vector6d Get6dVel() const;
  Vector6d Get6dAcc() const;
};

/**
 * @brief  6D-state (linear+angular) of an object in 3-dimensional space,
 * where the angular part is expressed by Euler angles.
 */
class State3dEuler {
public:
  StateLin3d lin; ///< linear position, velocity and acceleration
  StateLin3d ang; ///< roll-pitch-yaw Euler angles, rates- and rate derivatives.
};



// Conversions between Euler angles and Quaternions
/**
 * @brief Converts an orientation to Euler ZY'X'' convention.
 *
 * First rotate around z-axis,
 * then around new y' axis,
 * finally around newest x'' axis.
 *
 * @param  q  Quaternion expressing current orientation.
 * @return first element is roll angle in radians, then pitch, then yaw
 */
static Vector3d GetEulerZYXAngles(const Quaterniond& q)
{
  Vector3d yaw_pitch_roll = q.normalized().toRotationMatrix().eulerAngles(Z, Y, X);
  return yaw_pitch_roll.reverse();
}

/**
 * @brief Converts an orientation to Quaternion from Euler ZY'X'' convention.
 *
 * First rotate around z-axis,
 * then around new y' axis,
 * finally around newest x'' axis.
 */
static Quaterniond GetQuaternionFromEulerZYX(double yaw, double pitch, double roll)
{
  using namespace Eigen;
  Quaterniond q;

  q =   AngleAxisd(yaw,   Vector3d::UnitZ())
      * AngleAxisd(pitch, Vector3d::UnitY())
      * AngleAxisd(roll,  Vector3d::UnitX());

  return q;
}



// convenience functions for easy readability
inline std::ostream& operator<<(std::ostream& out, const StateLinXd& pos)
{
  out << "p=" << pos.p_.transpose() << "  "
      << "v=" << pos.v_.transpose() << "  "
      << "a=" << pos.a_.transpose();
  return out;
}

inline StateLinXd operator+(const StateLinXd& lhs, const StateLinXd& rhs)
{
  StateLinXd ret(lhs.kNumDim);
  ret.p_ = lhs.p_ + rhs.p_;
  ret.v_ = lhs.v_ + rhs.v_;
  ret.a_ = lhs.a_ + rhs.a_;
  return ret;
}

inline StateLinXd operator*(double mult, const StateLinXd& rhs)
{
  StateLinXd ret(rhs.kNumDim);
  ret.p_ = mult * rhs.p_;
  ret.v_ = mult * rhs.v_;
  ret.a_ = mult * rhs.a_;
  return ret;
}

inline bool StateLinXd::operator==(const StateLinXd &other) const
{
  bool all_equal = (p_==other.p_
                 && v_==other.v_
                 && a_==other.a_);
  return all_equal;
}

inline bool StateLinXd::operator!=(const StateLinXd &other) const
{
  return !(*this == other);
}

inline std::ostream& operator<<(std::ostream& out, const StateAng3d& ori)
{
  Vector3d rpy_rad;
  rpy_rad = GetEulerZYXAngles(ori.q);
  out << "rpy=" << rpy_rad.transpose() << "  "
      << "v="   << ori.w.transpose() << "  "
      << "a="   << ori.wd.transpose();
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const State3d& pose)
{
  out << "\tPos: " << pose.lin << "\n"
      << "\tOri: " << pose.ang;
  return out;
}

} // namespace xpp

#endif // _XPP_STATES_STATE_H_
