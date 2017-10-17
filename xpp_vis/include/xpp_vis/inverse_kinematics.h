
#ifndef XPP_VIS_INVERSE_KINEMATICS_H_
#define XPP_VIS_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <memory>

#include <xpp_states/endeffectors.h>
#include <xpp_states/joints.h>

namespace xpp {

/**
 *  @brief  Converts Cartesian endeffector positions into joint angles.
 *
 *  This class is responsible for calculating the joint angles of a robot
 *  given an endeffector position (=inverse kinematics).
 *  Base class that every inverse dynamics class must conform with.
 */
class InverseKinematics {
public:
  using Ptr      = std::shared_ptr<InverseKinematics>;
  using Vector3d = Eigen::Vector3d;

  InverseKinematics () {};
  virtual ~InverseKinematics () {};

  /**
    * @brief  Calculates the joint angles to reach a position @ pos_b.
    * @param  pos_b  3D-position of the endeffector expressed in base frame.
    * @return Joints angles of the robot.
    */
  virtual Joints GetAllJointAngles(const EndeffectorsPos& pos_b) const = 0;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  virtual int GetEECount() const = 0;
};

} /* namespace xpp */

#endif /* XPP_VIS_INVERSE_KINEMATICS_H_ */
