#include "transform_graph/position.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"

namespace transform_graph {
Position::Position() : vector_() { vector_ << 0, 0, 0; }

Position::Position(double x, double y, double z) : vector_() {
  vector_ << x, y, z;
}

Position::Position(const Eigen::Vector3d& v) : vector_() { vector_ = v; }

Position::Position(const geometry_msgs::Point& p) : vector_() {
  vector_ << p.x, p.y, p.z;
}

Position::Position(const geometry_msgs::Vector3& v) : vector_() {
  vector_ << v.x, v.y, v.z;
}

Position::Position(const pcl::PointXYZ& p) : vector_() {
  vector_ << p.x, p.y, p.z;
}

Position::Position(const tf::Vector3& v) : vector_() {
  vector_ << v.x(), v.y(), v.z();
}

Eigen::Vector3d Position::vector() const { return vector_; }
}  // namespace transform_graph
