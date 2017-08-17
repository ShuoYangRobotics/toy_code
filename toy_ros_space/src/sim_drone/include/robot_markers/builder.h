#ifndef _ROBOT_MARKERS_BUILDER_H_
#define _ROBOT_MARKERS_BUILDER_H_

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "std_msgs/ColorRGBA.h"
#include "transform_graph/transform_graph.h"
#include "urdf/model.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "robot_markers/forward_kinematics.h"

namespace robot_markers {
/// \brief Builder creates a visualization of a robot, given its URDF model.
///
/// You can either get the visualization as a visualization_msgs::MarkerArray or
/// as a visualization_msgs::InteractiveMarker.
/// The visualization is based on the URDF and should represent the robot with
/// all joint positions set to 0.
/// However, you can set the joint positions before building the marker.
///
/// The API uses a builder pattern. You first create a builder with the URDF and
/// initialize it (Init). Then, you set the namespace (the marker IDs
/// start at 0 and count up), time stamp, frame ID, pose, color, etc. If no
/// color is provided or the color values are all 0, the mesh color will be
/// used.
///
/// Once you have set all the marker fields, you then call Build to get the
/// marker or markers representing the robot.
///
/// \b Example:
/// \code
///   // Robot 1: Default configuration.
///   visualization_msgs::MarkerArray robot1;
///   builder.SetNamespace("robot");
///   builder.SetFrameId("base_link");
///   builder.SetTime(ros::Time::now());
///   builder.Build(&robot1);
///   marker_arr_pub.publish(robot1);
///
///   // Robot 2: Different pose and joints changed.
///   visualization_msgs::MarkerArray robot2;
///   builder.SetNamespace("robot2");
///
///   std::map<std::string, double> joint_positions;
///   joint_positions["torso_lift_joint"] = 0.1;
///   joint_positions["head_tilt_joint"] = 0.5;
///   builder.SetJointPositions(joint_positions);
///
///   geometry_msgs::Pose pose;
///   pose.position.y = 1;
///   pose.orientation.w = 0.92387953;
///   pose.orientation.z = -0.38268343;
///   builder.SetPose(pose);
///   builder.Build(&robot2);
///   marker_arr_pub.publish(robot2);
/// \endcode
class Builder {
 public:
  /// Construct a Builder with the given URDF model.
  explicit Builder(const urdf::Model& model);

  /// \brief Initializes the Builder.
  ///
  /// Call Init() once before calling any other methods.
  /// This does NOT set any marker parameters (color, pose, etc.).
  void Init();

  /// \brief Set the joint angles on the robot.
  ///
  /// If the robot does not have a joint, an error will be logged and the
  /// visualization will be unchanged.
  ///
  /// \param[in] joint_positions The joint angles/positions to set. This method
  ///   does not check joint limits, so you must supply valid values.
  void SetJointPositions(const std::map<std::string, double> joint_positions);

  /// \brief Set the output frame ID.
  ///
  /// All the markers will be given in this frame.
  ///
  /// \param[in] frame_id The frame ID for all the markers.
  void SetFrameId(const std::string& frame_id);

  /// \brief Set the time stamp.
  ///
  /// All the markers will be given this time stamp.
  ///
  /// \param[in] stamp The time stamp for all the markers.
  void SetTime(const ros::Time& stamp);

  /// \brief Set the marker namespace.
  ///
  /// Note that you cannot set the IDs of the markers. To visualize multiple
  /// robots, give each their own namespace.
  ///
  /// \param[in] ns The namespace for all the markers.
  void SetNamespace(const std::string& ns);

  /// \brief Set the pose of the robot, in the frame ID given by SetFrameId.
  ///
  /// \param[in] pose The pose of the robot.
  void SetPose(const geometry_msgs::Pose& pose);

  /// \brief Sets the color of the robot.
  ///
  /// Set to 0, 0, 0, 0 to use the mesh colors if available. If the mesh
  /// contains color information, then setting a non-zero color will tint the
  /// robot in that color.
  ///
  /// \param[in] r The red value, between 0 and 1.
  /// \param[in] g The green value, between 0 and 1.
  /// \param[in] b The blue value, between 0 and 1.
  /// \param[in] a The alpha value, between 0 and 1.
  void SetColor(float r, float g, float b, float a);

  /// \brief Sets the lifetime of the robot markers.
  ///
  /// \param[in] lifetime The lifetime for the robot markers.
  void SetLifetime(const ros::Duration& lifetime);

  /// \brief Sets whether the marker should be locked to its frame ID.
  ///
  /// The frame ID is supplied via SetFrameId.
  ///
  /// \param[in] frame_locked True to lock the markers to the robot's frame ID,
  ///   false otherwise.
  void SetFrameLocked(bool frame_locked);

  /// \brief Builds a visualization of the robot model as a marker array.
  ///
  /// It sets marker properties according to the Set* methods in the class.
  /// You can build multiple sets of markers with the same builder, but you
  /// should be mindful of the history of property settings made beforehand.
  ///
  /// \param[out] marker_array The marker array message to append to.
  void Build(visualization_msgs::MarkerArray* marker_array);

 private:
  // Sets everything in the marker except the pose.
  void BuildMarker(const urdf::Link& link, int id,
                   visualization_msgs::Marker* output);
  std::string NodeName(const std::string& name);

  urdf::Model model_;
  ForwardKinematics fk_;
  transform_graph::Graph tf_graph_;

  std::map<std::string, double> joint_positions_;

  // Marker fields
  std::string frame_id_;
  ros::Time stamp_;
  std::string ns_;
  geometry_msgs::Pose pose_;
  std_msgs::ColorRGBA color_;
  ros::Duration lifetime_;
  bool frame_locked_;

  bool has_initialized_;
};
}  // namespace robot_markers

#endif  // _ROBOT_MARKERS_BUILDER_H_
