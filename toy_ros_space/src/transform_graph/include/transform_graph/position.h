#ifndef _TRANSFORM_GRAPH_POSITION_H_
#define _TRANSFORM_GRAPH_POSITION_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"

namespace transform_graph {
/// \brief Position provides conversions from common position types.
///
/// By default it is (0, 0, 0).
/// To get the origin/vector, call Position::vector().
class Position {
 public:
  /// Default constructor, (0, 0, 0).
  Position();
  /// x/y/z constructor.
  Position(double x, double y, double z);
  /// Eigen::Vector3d constructor.
  Position(const Eigen::Vector3d& v);
  /// geometry_msgs point constructor.
  Position(const geometry_msgs::Point& p);
  /// geometry_msgs vector constructor.
  Position(const geometry_msgs::Vector3& v);
  /// PCL point constructor.
  Position(const pcl::PointXYZ& p);
  /// tf vector constructor.
  Position(const tf::Vector3& v);

  // TODO: Add more PCL point types.

  /// \brief Returns this translation as an Eigen::Vector3d.
  /// \returns The translation, as an Eigen::Vector3d.
  Eigen::Vector3d vector() const;

 private:
  Eigen::Vector3d vector_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_POSITION_H_
