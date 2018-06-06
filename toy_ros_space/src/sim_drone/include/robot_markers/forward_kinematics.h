#ifndef _ROBOT_MARKERS_FORWARD_KINEMATICS_H_
#define _ROBOT_MARKERS_FORWARD_KINEMATICS_H_

#include <string>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "kdl/frames.hpp"
#include "kdl/segment.hpp"
#include "kdl/tree.hpp"
#include "urdf/model.h"

namespace robot_markers {
class SegmentPair {
 public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root,
              const std::string& p_tip);

  KDL::Segment segment;
  std::string root, tip;
};

/// \brief Computes forward kinematics for the robot.
///
/// Based on code from robot_state_publisher.
class ForwardKinematics {
 public:
  /// Constructor.
  ///
  /// \param[in] model The URDF model.
  explicit ForwardKinematics(const urdf::Model& model);

  /// Initializes the forward kinematics.
  ///
  /// This should be called before calling any other methods.
  void Init();

  /// Outputs the transforms for each set of joint angles passed in.
  ///
  /// If a joint is not part of the URDF, then it is skipped.
  ///
  /// \param[in] joint_positions The joint angles to query.
  /// \param[out] transforms The transforms, one for each joint angle.
  void GetTransforms(
      const std::map<std::string, double>& joint_positions,
      std::vector<geometry_msgs::TransformStamped>* transforms) const;

 private:
  urdf::Model model_;
  KDL::Tree tree_;
  std::map<std::string, SegmentPair> segments_;

  void AddChildren(const KDL::SegmentMap::const_iterator segment);
};
}  // namespace robot_markers

#endif  // _ROBOT_MARKERS_FORWARD_KINEMATICS_H_
