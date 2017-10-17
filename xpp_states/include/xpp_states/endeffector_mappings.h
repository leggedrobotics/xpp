/**
 * @file endeffector_mappings.h
 *
 * Assigning some semantic information (e.g. name of the foot) to endeffector
 * indices.
 */

#ifndef XPP_STATES_ENDEFFECTOR_MAPPINGS_H_
#define XPP_STATES_ENDEFFECTOR_MAPPINGS_H_

namespace xpp {

namespace biped {
enum FootIDs { L=0, R };
}

namespace quad {
enum FootIDs { LF=0, RF, LH, RH };
}

namespace quad_rotor {
enum RotoIDs { L=0, R, F, H };
}

} // namespace xpp

#endif /* XPP_STATES_ENDEFFECTOR_MAPPINGS_H_ */
