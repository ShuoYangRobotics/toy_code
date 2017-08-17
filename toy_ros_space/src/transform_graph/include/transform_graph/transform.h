#ifndef _TRANSFORM_GRAPH_TRANSFORM_H_
#define _TRANSFORM_GRAPH_TRANSFORM_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

#include "transform_graph/orientation.h"
#include "transform_graph/position.h"

namespace transform_graph {
/// \brief Transform describes a combination of a rotation and a translation.
///
/// By default it is the identity transform.
/// To get the homogeneous transform matrix, call Transform::matrix().
/// The top left 3x3 submatrix is the rotation matrix.
/// The columns of the rotation matrix describe the principal axes of the local
/// frame relative to the reference frame.
/// The top right 3x1 vector is the origin of the local frame relative to the
/// reference frame.
class Transform {
 public:
  /// Default, identity constructor.
  Transform();
  /// Constructor from other transform_graph types (or implicitly convertible
  /// types).
  Transform(const transform_graph::Position& position,
            const transform_graph::Orientation& orientation);
  /// Constructor from a tf Transform.
  Transform(const tf::Transform& st);
  /// Constructor from a pose.
  Transform(const geometry_msgs::Pose& p);
  /// Constructor from an Eigen Affine matrix.
  Transform(const Eigen::Affine3d& a);
  /// Constructor from an Eigen matrix.
  Transform(const Eigen::Matrix4d& m);

  /// Returns the identity transform.
  static Transform Identity();

  /// \brief Returns this transform's homogeneous transform matrix.
  /// \returns This transform's homogeneous transform matrix.
  Eigen::Matrix4d matrix() const;

  /// \brief Returns this transform as a geometry_msgs::Pose.
  /// \returns This transform as a Pose.
  void ToPose(geometry_msgs::Pose* pose) const;

  /// \brief Returns the inverse of this transform.
  /// \returns The inverse of this transform.
  transform_graph::Transform inverse() const;

 private:
  Eigen::Affine3d transform_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_TRANSFORM_H_
