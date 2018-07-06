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

#ifndef _XPP_STATES_ENDEFFECTORS_H_
#define _XPP_STATES_ENDEFFECTORS_H_

#include <deque>
#include <iostream>
#include <vector>

#include <xpp_states/state.h>

namespace xpp {

using EndeffectorID = uint;

/**
 * @brief Data structure to assign values to each endeffector.
 *
 * Common values are xyz-positions (Vector3d), contact-flags (bool), or joints
 * angles of each leg.
 *
 * This gives a unified interface, and also holds for boolean values (which
 * is not possible using an std::vector). It allows to restrict default
 * constructors, define addition and subtraction of all endeffectors and
 * other convenience functions associated specifically to endeffectors.
 *
 * The idea is that this class is an enhanced STL container, but complies to the
 * same interface, (e.g at()). However, in case this unified interface is
 * burdensome, you can always access the underlying STL-deque container directly.
 */
template<typename T>
class Endeffectors {
public:
  using Container     = std::deque<T>;   // only to avoid faulty std::vector<bool>
  using EndeffectorsT = Endeffectors<T>;

  Endeffectors (int n_ee = 0);
  virtual ~Endeffectors () = default;

  /**
   * @brief Sets the number of endeffectors.
   */
  void SetCount(int n_ee);

  /**
   * @brief Sets each endeffector to the same value.
   */
  void SetAll(const T& value);

  /**
   * @returns Number of endeffectors this structure holds.
   */
  int GetEECount() const;

  /**
   * @returns All endeffector IDs from 0 to the number of endeffectors.
   */
  std::vector<EndeffectorID> GetEEsOrdered() const;

  /**
   * @brief Read/write access to the endeffector stored at index ee.
   * @param  ee  Endeffector index/position.
   */
  T& at(EndeffectorID ee);

  /**
   * @brief Read access to the endeffector stored at index ee.
   * @param  ee  Endeffector index/position.
   */
  const T& at(EndeffectorID ee) const;

  const EndeffectorsT operator-(const EndeffectorsT& rhs) const;
  const EndeffectorsT operator/(double scalar) const;

  /**
   * @returns true if only one element differs from other.
   */
  bool operator!=(const Endeffectors& other) const;

  /**
   * @returns a returns a copy(!) of the underlying STL-deque container.
   */
  Container ToImpl() const;

private:
  Container ee_;
};

// convenience typedefs, can also be extended to derived classes if desired.
using EndeffectorsPos  = Endeffectors<Eigen::Vector3d>;
using EndeffectorsVel  = Endeffectors<Eigen::Vector3d>;
using EndeffectorsAcc  = Endeffectors<Eigen::Vector3d>;

/**
 * @brief Bundles the position, velocity and acceleration of all endeffectors.
 * as well as appending a EndeffectorMotion specific convenience function.
 */
class EndeffectorsMotion : public Endeffectors<StateLin3d> {
public:
  /**
   * @brief  Extract only either the pos, vel or acc from all endeffectors.
   * @param  deriv  Derivative being either position, velocity or acceleration.
   */
  Endeffectors<Vector3d> Get (MotionDerivative deriv) const
  {
    Endeffectors<Vector3d> val(GetEECount());
    for (auto ee : GetEEsOrdered())
      val.at(ee) = at(ee).GetByIndex(deriv);

    return val;
  }
};


/**
 * @brief Bundles the contact state of all endeffectors.
 *
 * Says if an endeffector is currently touching the environment or not.
 * This is often an important criteria for motion planning, as only in the
 * contact state can forces be exerted that move the body.
 */
class EndeffectorsContact : public Endeffectors<bool> {
public:
  /**
   * @brief Constructs a state, the default being 0 feet, none in contact.
   * @param  n_ee  Number of endeffectors.
   * @param  in_contact  True if all legs should be in contact, false otherwise.
   */
  EndeffectorsContact (int n_ee=0, bool in_contact=false)
      :Endeffectors(n_ee) { SetAll(in_contact);};

  /**
   * @brief The number of endeffectors in contact with the environment.
   */
  int GetContactCount() const
  {
    int count = 0;
    for (auto ee : GetEEsOrdered())
      if (at(ee))
        count++;

    return count;
  }
};


// implementations
template<typename T>
Endeffectors<T>::Endeffectors (int n_ee)
{
  SetCount(n_ee);
}

template<typename T>
void
Endeffectors<T>::SetCount (int n_ee)
{
  ee_.resize(n_ee);
}

template<typename T>
void
Endeffectors<T>::SetAll (const T& value)
{
  std::fill(ee_.begin(), ee_.end(), value);
}

template<typename T>
T&
Endeffectors<T>::at (EndeffectorID idx)
{
  return ee_.at(idx);
}

template<typename T>
const T&
Endeffectors<T>::at (EndeffectorID idx) const
{
  return ee_.at(idx);
}

template<typename T>
int
Endeffectors<T>::GetEECount () const
{
  return ee_.size();
}

template<typename T>
typename Endeffectors<T>::Container
Endeffectors<T>::ToImpl () const
{
  return ee_;
}

template<typename T>
std::vector<EndeffectorID>
Endeffectors<T>::GetEEsOrdered () const
{
  std::vector<EndeffectorID> vec;
  for (int i=0; i<ee_.size(); ++i)
    vec.push_back(i);

  return vec;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator - (const EndeffectorsT& rhs) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.at(i) = ee_.at(i) - rhs.at(i);

  return result;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator / (double scalar) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.at(i) = ee_.at(i)/scalar;

  return result;
}

template <typename T>
std::ostream& operator<<(std::ostream& stream, Endeffectors<T> endeffectors)
{
  for (EndeffectorID ee : endeffectors.GetEEsOrdered())
    stream << endeffectors.at(ee) << ", ";

  return stream;
}

template<typename T>
bool
Endeffectors<T>::operator!=(const Endeffectors& other) const
{
  for (auto ee : GetEEsOrdered()) {
    if (ee_.at(ee) != other.at(ee))
      return true;
  }
  return false;
}

} /* namespace xpp */

#endif /* _XPP_STATES_ENDEFFECTORS_H_ */
