/*
 * 	rewrite of original SL inverse kinematics functions
 *
 *  Created on: Jan, 2016
 *      Author: Manuel Lussi <mlussi@student.ethz.ch>
 */

#include <xpp/mono/monoped_inverse_kinematics.h>

#include <cmath>
#include <iostream>

#include <xpp/endeffectors.h>

namespace xpp {
namespace mono {


MonopedInverseKinematics::MonopedInverseKinematics ()
{
}

int
MonopedInverseKinematics::GetJointsPerEE () const
{
  return kNumJointsPerLeg;
}

JointValues
MonopedInverseKinematics::GetAllJointAngles(const EndeffectorsPos& x_B) const
{
	double q_HAA_bf, q_HAA_br, q_HFE_br; // rear bend of knees
	double q_HFE_bf, q_KFE_br, q_KFE_bf; // forward bend of knees

	Eigen::Vector3d xr;
	Eigen::Matrix3d R;

	// translate to the local coordinate of the attachment of the leg
	// and flip coordinate signs such that all computations can be done
	// for the front-left leg
	Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
  xr = x_B.At(E0) + offset_base_to_hip;

	// compute the HAA angle
	q_HAA_bf = q_HAA_br = -atan2(xr[Y],-xr[Z]);

	// rotate into the HFE coordinate system (rot around X)
	R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA_bf), -sin(q_HAA_bf), 0.0, sin(q_HAA_bf), cos(q_HAA_bf);

	xr = (R * xr).eval();

	// translate into the HFE coordinate system (along Z axis)
	xr[Z] += 0.08;  //distance of HFE to HAA in z direction

	// compute square of length from HFE to foot
	double tmp1 = pow(xr[X],2)+pow(xr[Z],2);


	// compute temporary angles (with reachability check)
	double lu = 0.35;  // length of upper leg
	double ll = 0.33;  // length of lower leg
	double alpha = atan2(-xr[Z],xr[X]) - 0.5*M_PI;  //  flip and rotate to match HyQ joint definition


	double some_random_value_for_beta = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1)); // this must be between -1 and 1
	if (some_random_value_for_beta > 1) {
		some_random_value_for_beta = 1;
	}
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
	double beta = acos(some_random_value_for_beta);

	// compute Hip FE angle
	q_HFE_bf = q_HFE_br = alpha + beta;


	double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
	// law of cosines give the knee angle
	if (some_random_value_for_gamma > 1) {
		some_random_value_for_gamma = 1;
	}
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
	double gamma  = acos(some_random_value_for_gamma);


	q_KFE_bf = q_KFE_br = gamma - M_PI;

	// forward knee bend
	EnforceLimits(q_HAA_bf, HAA);
	EnforceLimits(q_HFE_bf, HFE);
	EnforceLimits(q_KFE_bf, KFE);

	// backward knee bend
  EnforceLimits(q_HAA_br, HAA);
  EnforceLimits(q_HFE_br, HFE);
  EnforceLimits(q_KFE_br, KFE);


  JointValues q_mono(kNumEE, kNumJointsPerLeg, 0.0);
  q_mono.At(E0) << q_HAA_bf, q_HFE_bf, q_KFE_bf;

  return q_mono;
}

void
MonopedInverseKinematics::EnforceLimits (double& val, MonopedJointID joint) const
{
  // real joint angle limits
  const static double haa_min = -90;
  const static double haa_max = +30;

  const static double hfe_min = -50;
  const static double hfe_max =  70;

  const static double kfe_min = -140;
  const static double kfe_max =  -20;

  // reduced joint angles for optimization
  static const std::map<MonopedJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<MonopedJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

MonopedInverseKinematics::~MonopedInverseKinematics ()
{
}

} /* namespace mono */
} /* namespace xpp */


