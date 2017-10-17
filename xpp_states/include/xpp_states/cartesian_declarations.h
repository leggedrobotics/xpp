/**
 * @file cartesian_declartions.h
 *
 * Defines common conventions and index values to be used in Cartesian
 * environments.
 */

#ifndef XPP_CARTESIAN_DECLARATIONS_H_
#define XPP_CARTESIAN_DECLARATIONS_H_

#include <cassert>

namespace xpp {

// 2-dimensional
static constexpr int kDim2d = 2;
enum Coords2D { X_=0, Y_};

// 3-dimensional
static constexpr int kDim3d = 3;
enum Coords3D { X=0, Y, Z };
static Coords2D To2D(Coords3D dim)
{
  assert(dim != Z);
  return static_cast<Coords2D>(dim);
};

// 6-dimensional
// 'A' stands for angular, 'L' for linear.
static constexpr int kDim6d = 6; // X,Y,Z, roll, pitch, yaw
enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
static const Coords6D AllDim6D[] = {AX, AY, AZ, LX, LY, LZ};

} // namespace xpp

#endif /* XPP_CARTESIAN_DECLARATIONS_H_ */
