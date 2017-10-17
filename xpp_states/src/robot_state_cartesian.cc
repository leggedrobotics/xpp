
#include <xpp_states/robot_state_cartesian.h>

namespace xpp {

RobotStateCartesian::RobotStateCartesian(int n_ee)
{
  ee_motion_.SetCount(n_ee);
  ee_forces_.SetCount(n_ee);
  ee_contact_.SetCount(n_ee);
  ee_contact_.SetAll(true);
  t_global_ = 0.0;
};

} /* namespace xpp */

