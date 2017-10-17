
#include <xpp_states/robot_state_joint.h>

namespace xpp {

RobotStateJoint::RobotStateJoint (int n_ee, int n_joints_per_ee)
    :q_(n_ee, n_joints_per_ee),
     qd_(n_ee, n_joints_per_ee),
     qdd_(n_ee, n_joints_per_ee),
     torques_(n_ee, n_joints_per_ee)
{
  ee_contact_.SetCount(n_ee);
  ee_contact_.SetAll(true);
  t_global_ = 0.0;
}

RobotStateJoint::~RobotStateJoint ()
{
}

} /* namespace xpp */
