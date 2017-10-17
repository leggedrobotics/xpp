
#ifndef XPP_STATES_TERRAIN_TYPES_H_
#define XPP_STATES_TERRAIN_TYPES_H_

namespace xpp {

/**
 * @brief Terrains IDs corresponding to a draw function in xpp_vis and a
 * detailed (gradient supplying) function in the optimizer.
 */
enum TerrainID { FlatID=0,
                 BlockID,
                 StairsID,
                 GapID,
                 SlopeID,
                 ChimneyID,
                 ChimneyLRID,
                 K_TERRAIN_COUNT };

} // namespace xpp


#endif /* XPP_STATES_TERRAIN_TYPES_H_ */
