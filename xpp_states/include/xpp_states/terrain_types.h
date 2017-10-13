/**
@file    terrain_types.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 13, 2017
@brief   Brief description
 */

#ifndef XPP_STATES_INCLUDE_XPP_STATES_TERRAIN_TYPES_H_
#define XPP_STATES_INCLUDE_XPP_STATES_TERRAIN_TYPES_H_

namespace xpp {

enum TerrainID { FlatID=0,
                 BlockID,
                 StairsID,
                 GapID,
                 SlopeID,
                 ChimneyID,
                 ChimneyLRID,
                 K_TERRAIN_COUNT };

} // namespace xpp


#endif /* XPP_STATES_INCLUDE_XPP_STATES_TERRAIN_TYPES_H_ */
