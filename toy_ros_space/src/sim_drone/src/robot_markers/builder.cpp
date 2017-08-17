#include "robot_markers/builder.h"

#include <map>
#include <string>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "transform_graph/transform_graph.h"
#include "urdf/model.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "Eigen/Eigen"

using transform_graph::RefFrame;
using transform_graph::Source;
using transform_graph::Target;
using transform_graph::Transform;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

typedef std::map<std::string, urdf::LinkSharedPtr> LinkMap;

namespace {
geometry_msgs::Pose ToGeometryPose(const urdf::Pose& urdf_pose) {
  geometry_msgs::Pose pose;
  pose.position.x = urdf_pose.position.x;
  pose.position.y = urdf_pose.position.y;
  pose.position.z = urdf_pose.position.z;
  pose.orientation.w = urdf_pose.rotation.w;
  pose.orientation.x = urdf_pose.rotation.x;
  pose.orientation.y = urdf_pose.rotation.y;
  pose.orientation.z = urdf_pose.rotation.z;
  return pose;
}

geometry_msgs::Pose ToGeometryPose(
    const transform_graph::Transform& transform) {
  Eigen::Matrix3d rot(transform.matrix().topLeftCorner(3, 3));
  Eigen::Quaterniond q(rot);
  geometry_msgs::Pose pose;
  pose.position.x = transform.matrix()(0, 3);
  pose.position.y = transform.matrix()(1, 3);
  pose.position.z = transform.matrix()(2, 3);
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}
}  // namespace

namespace robot_markers {
Builder::Builder(const urdf::Model& model)
    : model_(model),
      fk_(model),
      tf_graph_(),
      frame_id_(""),
      stamp_(0),
      ns_(""),
      pose_(),
      color_(),
      lifetime_(0),
      frame_locked_(false),
      has_initialized_(false) {
  pose_.orientation.w = 1;
}

void Builder::Init() {
  fk_.Init();

  // Build initial transform graph
  std::vector<urdf::JointSharedPtr> joints;
  std::vector<urdf::LinkSharedPtr> links;
  model_.getLinks(links);
  for (size_t i = 0; i < links.size(); ++i) {
    const urdf::LinkSharedPtr& link_p = links[i];
    const std::string& name = link_p->name;

    if (link_p->parent_joint) {
      geometry_msgs::Pose pose = ToGeometryPose(
          link_p->parent_joint->parent_to_joint_origin_transform);
      const std::string& parent_name = link_p->getParent()->name;
      tf_graph_.Add(NodeName(name), RefFrame(NodeName(parent_name)),
                    Transform(pose));
    }
  }
  has_initialized_ = true;
}

void Builder::SetJointPositions(
    const std::map<std::string, double> joint_positions) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  fk_.GetTransforms(joint_positions, &transforms);
  for (size_t i = 0; i < transforms.size(); ++i) {
    const geometry_msgs::TransformStamped& tf = transforms[i];
    transform_graph::Transform transform(tf.transform.translation,
                                         tf.transform.rotation);
    tf_graph_.Add(NodeName(tf.child_frame_id),
                  RefFrame(NodeName(tf.header.frame_id)), transform);
  }
}

void Builder::SetFrameId(const std::string& frame_id) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }

  frame_id_ = frame_id;
}
void Builder::SetTime(const ros::Time& stamp) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }

  stamp_ = stamp;
}
void Builder::SetNamespace(const std::string& ns) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }
  ns_ = ns;
}
void Builder::SetPose(const geometry_msgs::Pose& pose) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }
  pose_ = pose;
}
void Builder::SetColor(float r, float g, float b, float a) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }

  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;
}

void Builder::Build(MarkerArray* marker_array) {
  if (!has_initialized_) {
    ROS_ERROR(
        "You must call Init() before calling any other methods of "
        "robot_markers::Builder.");
    return;
  }

  const std::string& root_name = model_.getRoot()->name;

  tf_graph_.Add(NodeName(root_name), RefFrame(frame_id_), pose_);

  std::vector<urdf::LinkSharedPtr> links;
  model_.getLinks(links);
  for (size_t i = 0; i < links.size(); ++i) {
    const urdf::LinkSharedPtr& link_p = links[i];
    const std::string& name = link_p->name;

    if (!link_p->visual) {
      continue;
    }

    Marker marker;
    BuildMarker(*link_p, i, &marker);
    transform_graph::Transform transform_out;
    geometry_msgs::Pose visual_transform =
        ToGeometryPose(link_p->visual->origin);
    bool success =
        tf_graph_.DescribePose(visual_transform, Source(NodeName(name)),
                               Target(frame_id_), &transform_out);
    if (!success) {
      ROS_ERROR("No transform from %s to %s!", frame_id_.c_str(), name.c_str());
      continue;
    } else {
    }
    marker.pose = ToGeometryPose(transform_out);
    marker_array->markers.push_back(marker);
  }
}

void Builder::BuildMarker(const urdf::Link& link, int id, Marker* output) {
  output->header.frame_id = frame_id_;
  output->header.stamp = stamp_;
  output->ns = ns_;
  output->id = id;
  output->color = color_;
  output->lifetime = lifetime_;
  output->frame_locked = frame_locked_;

  if (link.visual->geometry->type == urdf::Geometry::BOX) {
    output->type = Marker::CUBE;
    const urdf::Box& box = dynamic_cast<urdf::Box&>(*link.visual->geometry);
    output->scale.x = box.dim.x;
    output->scale.y = box.dim.y;
    output->scale.z = box.dim.z;
  } else if (link.visual->geometry->type == urdf::Geometry::CYLINDER) {
    output->type = Marker::CYLINDER;
    const urdf::Cylinder& cylinder =
        dynamic_cast<urdf::Cylinder&>(*link.visual->geometry);
    output->scale.x = 2 * cylinder.radius;
    output->scale.y = 2 * cylinder.radius;
    output->scale.z = cylinder.length;
  } else if (link.visual->geometry->type == urdf::Geometry::MESH) {
    output->type = Marker::MESH_RESOURCE;
    const urdf::Mesh& mesh = dynamic_cast<urdf::Mesh&>(*link.visual->geometry);
    output->mesh_use_embedded_materials = true;
    if (mesh.scale.x == 0 && mesh.scale.y == 0 && mesh.scale.z == 0) {
      output->scale.x = 1;
      output->scale.y = 1;
      output->scale.z = 1;
    } else {
      output->scale.x = mesh.scale.x;
      output->scale.y = mesh.scale.y;
      output->scale.z = mesh.scale.z;
    }
    output->mesh_resource = mesh.filename;
  } else if (link.visual->geometry->type == urdf::Geometry::SPHERE) {
    output->type = Marker::SPHERE;
    const urdf::Sphere& sphere =
        dynamic_cast<urdf::Sphere&>(*link.visual->geometry);
    output->scale.x = 2 * sphere.radius;
    output->scale.y = 2 * sphere.radius;
    output->scale.z = 2 * sphere.radius;
  }
}

std::string Builder::NodeName(const std::string& name) {
  return "robot " + name;
}
}  // namespace robot_markers
