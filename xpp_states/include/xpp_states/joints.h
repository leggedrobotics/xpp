/**
 @file    joint_values.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_UTILS_JOINT_VALUES_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_UTILS_JOINT_VALUES_H_

#include <map>
#include <vector>
#include <Eigen/Dense>

#include <xpp_states/endeffectors.h>

namespace xpp {

enum JointID { J0=0, J1, J2, J3, J4, J5, J6, J7, J8, J9, J10, J11, J12, J13, J14, BaseJoint };

/** @brief Container to access values associated to joints of endeffectors.
  */
class Joints : public Endeffectors<Eigen::VectorXd> {
public:
  using VectorXd = Eigen::VectorXd;
  using Base = Endeffectors<VectorXd>;
  using Base::GetCount;
  using Base::At;
  using EEOrder = std::vector<EndeffectorID>;

  /** @param n_ee             total number of endeffectors.
    * @param n_joints_per_ee  number of joints for each endeffector.
    * @param value            default joint value
    */
  explicit Joints (int n_ee, int n_joints_per_ee, double value = 0.0);
  explicit Joints (const std::vector<VectorXd>& joints);
  explicit Joints (const VectorXd& q, const EEOrder& ee_order);
  virtual ~Joints ();

  /** @brief Converts joint values to Eigen vector.
   *
   * The endeffectors are processed from E0,E1,.. and for each endeffector
   * the joints are appended.
   */
  VectorXd ToVec() const;

  /** @brief Converts joint values to Eigen vector according to specific order.
   *  @param ee_order  The order in which the Endeffectors are appended.
   */
  VectorXd ToVec(const EEOrder& ee_order) const;

  /** @brief Sets joints values from Eigen vector.
   *  @param q The Eigen Vector of joint values.
   *
   * The vector q is interpreted to be ordered from E0,E1 with the correct
   * number of endeffectors and joints per endeffector.
   */
  void SetFromVec(const VectorXd& q);

  /** @brief Sets joint values from Eigen vector in specific order.
    * @param q         The Eigen Vector of joint values.
    * @param ee_order  Describes the internal order of q.
    */
  void SetFromVec(const VectorXd& q, const EEOrder& ee_order);

  /** @returns read/write access of the joint value at joint.
    */
  double& At(JointID joint);
  double At(JointID joint) const;

  int GetNumJoints() const;

  const Joints operator + (const Joints& rhs) const;
  const Joints operator * (double scalar) const;

  int GetNumJointsPerEE() const;

protected:
  int n_joints_per_leg_;

private:
  int n_joints_;
};


// spring_clean_ this seems hacky, double check this
template<typename T>
std::map<T, JointID> GetMap(const std::map<EndeffectorID, std::vector<T> >& map_ee_to_joints) {

  std::map<T, JointID> map;

  int j = 0;
  for (int i=0; i<map_ee_to_joints.size(); ++i) {
    auto ee = static_cast<EndeffectorID>(i);
    for (T hyq_joint : map_ee_to_joints.at(ee)) {
      auto xpp_joint = static_cast<JointID>(j++);
      map[hyq_joint] = xpp_joint;
    }
  }

  return map;
}

} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_UTILS_JOINT_VALUES_H_ */
