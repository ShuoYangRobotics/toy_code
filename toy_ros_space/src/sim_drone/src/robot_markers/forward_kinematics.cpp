#include "robot_markers/forward_kinematics.h"

#include <string>
#include <vector>

#include "kdl/frames_io.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "ros/ros.h"
#include "ros/time.h"
#include "tf2_kdl/tf2_kdl.h"
#include "urdf/model.h"

namespace robot_markers {
SegmentPair::SegmentPair(const KDL::Segment& p_segment,
                         const std::string& p_root, const std::string& p_tip)
    : segment(p_segment), root(p_root), tip(p_tip) {}

ForwardKinematics::ForwardKinematics(const urdf::Model& model)
    : model_(model), tree_(), segments_() {}

void ForwardKinematics::Init() {
  kdl_parser::treeFromUrdfModel(model_, tree_);
  AddChildren(tree_.getRootSegment());
}

void ForwardKinematics::GetTransforms(
    const std::map<std::string, double>& joint_positions,
    std::vector<geometry_msgs::TransformStamped>* transforms) const {
  // Moving joints
  for (std::map<std::string, double>::const_iterator jnt =
           joint_positions.begin();
       jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg =
        segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform =
          tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.frame_id = seg->second.root;
      tf_transform.child_frame_id = seg->second.tip;
      transforms->push_back(tf_transform);
    } else {
      ROS_ERROR("Robot does not have joint \"%s\"", jnt->first.c_str());
    }
  }

  // Fixed joints. These are not needed for the purposes of drawing a marker
  // because the fixed joints are read by Builder::Init().
  // However, this may be useful code to repurpose elsewhere.
  //
  // for (std::map<std::string, SegmentPair>::const_iterator seg =
  //         segments_fixed_.begin();
  //     seg != segments_fixed_.end(); seg++) {
  //  geometry_msgs::TransformStamped tf_transform =
  //      tf2::kdlToTransform(seg->second.segment.pose(0));
  //  tf_transform.header.frame_id = seg->second.root;
  //  tf_transform.child_frame_id = seg->second.tip;
  //  transforms->push_back(tf_transform);
  //}
}

void ForwardKinematics::AddChildren(
    const KDL::SegmentMap::const_iterator segment) {
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children =
      GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root,
                  child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      // if (model_.getJoint(child.getJoint().getName()) &&
      //    model_.getJoint(child.getJoint().getName())->type ==
      //        urdf::Joint::FLOATING) {
      //} else {
      //   segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      //}
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
    }
    AddChildren(children[i]);
  }
}
}  // namespace robot_markers
