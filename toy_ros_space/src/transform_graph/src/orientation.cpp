#include "transform_graph/orientation.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

namespace transform_graph {
Orientation::Orientation() : matrix_() {
  Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
  matrix_ = q.matrix();
}

Orientation::Orientation(double w, double x, double y, double z) : matrix_() {
  Eigen::Quaterniond q(w, x, y, z);
  matrix_ = q.matrix();
}

Orientation::Orientation(const Eigen::Matrix3d& m) : matrix_() { matrix_ = m; }

Orientation::Orientation(const Eigen::Quaterniond& q) : matrix_() {
  matrix_ = q.matrix();
}

Orientation::Orientation(const geometry_msgs::Quaternion& q) : matrix_() {
  Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
  matrix_ = eq.matrix();
}

Orientation::Orientation(const tf::Quaternion& q) : matrix_() {
  Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
  matrix_ = eq.matrix();
}

Orientation::Orientation(const tf::Matrix3x3& m) : matrix_() {
  const tf::Vector3& row0 = m.getRow(0);
  const tf::Vector3& row1 = m.getRow(1);
  const tf::Vector3& row2 = m.getRow(2);
  // clang-format off
  matrix_ << row0.x(), row0.y(), row0.z(),
             row1.x(), row1.y(), row1.z(),
             row2.x(), row2.y(), row2.z();
  // clang-format on
}

Eigen::Matrix3d Orientation::matrix() const { return matrix_; }

Eigen::Quaterniond Orientation::quaternion() const {
  return Eigen::Quaterniond(matrix_);
}
}  // namespace transform_graph
