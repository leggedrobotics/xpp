
#ifndef XPP_RVIZ_TERRAIN_BUILDER_H_
#define XPP_RVIZ_TERRAIN_BUILDER_H_

#include <Eigen/Dense>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <xpp_msgs/StateLin3d.h>


namespace xpp {

/**
 * @brief  Constructs RVIZ markers that show different terrains.
 *
 * This class shows simple terrains (flat, sloped, stairs, ...) through
 * basic RVIZ shapes.
 */
class RvizTerrainBuilder {
public:
  using Marker       = visualization_msgs::Marker;
  using MarkerArray  = visualization_msgs::MarkerArray;
  using Vector3d     = Eigen::Vector3d;
  using Quat         = Eigen::Quaterniond;

  /**
   * @brief  Creates a default terrain builder object.
   */
  RvizTerrainBuilder () {};
  virtual ~RvizTerrainBuilder () {};


  /**
   * @brief  Constructs the rviz markers for a specific terrain.
   * @param  terrain_id  The identifier for a specific terrain.
   */
  MarkerArray BuildTerrain(int terrain_id);

  /**
   * @brief Visualizes a 6D pose in rviz
   * @param pos  The linear position expressed in world frame (W).
   * @param orientation  In Euler angles (ZYX-convention).
   */
  geometry_msgs::PoseStamped BuildPose(const geometry_msgs::Point pos_W,
                                       xpp_msgs::StateLin3d orientation) const;

private:
  MarkerArray BuildTerrainFlat()      const;
  MarkerArray BuildTerrainBlock()     const;
  MarkerArray BuildTerrainStairs()    const;
  MarkerArray BuildTerrainGap()       const;
  MarkerArray BuildTerrainSlope()     const;
  MarkerArray BuildTerrainChimney()   const;
  MarkerArray BuildTerrainChimneyLR() const;

  Marker BuildTerrainBlock(const Vector3d& pos,
                           const Vector3d& edge_length,
                           const Quat& ori = Quat::Identity()) const;

  const double eps_ = 0.02;           // for lowering of terrain.
  const int terrain_ids_start_ = 50;  // to not overwrite other RVIZ markers.
  std::string rviz_frame_ = "world";  // the name of the frame set in RVIZ.
};

} /* namespace xpp */

#endif /* XPP_RVIZ_TERRAIN_BUILDER_H_ */
