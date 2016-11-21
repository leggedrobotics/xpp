/**
 @file    hyqb_vis.hpp
 @author  Diego Pardo (depardo@ethz.ch)
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 24, 2015
 @brief   Brief description here.
 */

#ifndef HYQB_VIS_HPP
#define HYQB_VIS_HPP

#include "robotVisBase.hpp"

static const std::size_t hyqJoints = 12;

static const std::array<std::string, hyqJoints> hyq_joint_names =
{
		"lf_haa_joint",
		"lf_hfe_joint",
		"lf_kfe_joint",

		"rf_haa_joint",
		"rf_hfe_joint",
		"rf_kfe_joint",

		"lh_haa_joint",
		"lh_hfe_joint",
		"lh_kfe_joint",

		"rh_haa_joint",
		"rh_hfe_joint",
		"rh_kfe_joint",

};

namespace xpp {
namespace vis {

class hyqb_vis : public robotVisBase<hyqJoints>
{
public:
	hyqb_vis()
      :robotVisBase("hyq",hyq_joint_names)
  {};
	virtual ~hyqb_vis() {};

};

} // namespace vis
} // namespace xpp

#endif /* HYQB_VIS_HPP */
