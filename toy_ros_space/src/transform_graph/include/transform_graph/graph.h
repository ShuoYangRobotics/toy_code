#ifndef _TRANSFORM_GRAPH_GRAPH_H_
#define _TRANSFORM_GRAPH_GRAPH_H_

#include <map>
#include <string>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "transform_graph/explicit_types.h"
#include "transform_graph/graph_internal.h"
#include "transform_graph/transform.h"

namespace transform_graph {
/// \brief Graph models static transformations between coordinate frames.
///
/// The vertices are coordinate frame IDs, given as strings. The edges are
/// directed and represent transformations between coordinate
/// frames.
///
/// Once the graph is specified, this class can be used to compute the
/// transformation between any two coordinate frames, as long as a weakly
/// connected path exists between them.
///
/// The graph may contain cycles. If it does, our shortest path
/// algorithm will find a path with the minimum number of edges.
///
/// The graph may be disconnected, in which case Graph may be used to find
/// transformations between two frames in the same connected component.
///
/// \b Example:
/// \code
///   transform_graph::Graph graph;
///   graph.Add("head_mount_kinect", transform_graph::RefFrame("base_link"),
///   head_pose);
///   graph.Add("wrist", transform_graph::RefFrame("base_link"), wrist_pose);
///   for (PointXYZ pt : cloud.points) {
///     transform_graph::Transform pt_in_wrist;
///     graph.DescribePosition(pt, transform_graph::Source("head_mount_kinect"),
///                            transform_graph::Target("wrist"), pt_in_wrist);
///     // pt_in_base is in wrist frame, do something with it
///   }
/// \endcode
///
/// In the example above, it may be more efficient to cache the transform
/// between head_mount_kinect and wrist:
/// \code
///   transform_graph::Transform kinect_in_wrist;
///   graph.ComputeDescription(transform_graph::LocalFrame("head_mount_kinect"),
///                            transform_graph::RefFrame("wrist"),
///                            &kinect_in_wrist);
///   graph.Add("head_mount_kinect", transform_graph::RefFrame("wrist"),
///             kinect_in_wrist);
/// \endcode
class Graph {
 public:
  /// Default constructor.
  Graph();

  /// \brief Adds or edits a transform (an edge) to the graph.
  ///
  /// If the transform (or its inverse) is already in the graph, this will
  /// replace it.
  ///
  /// You should only input transformations consisting of rotations and
  /// translations. Graph does not check if the matrix is well-formed or
  /// invertible.
  ///
  /// For the third parameter, you do not necessarily need to supply an
  /// transform_graph::Transform type. Many common transformation types will be
  /// implicitly converted, see transform.h to see a list.
  ///
  /// \param[in] name The name of the frame being added to the graph.
  /// \param[in] ref_frame_name The name of the reference frame that this
  ///   transform is expressed in.
  /// \param[in] transform The transform to add.
  ///
  /// \b Example: add a tooltip that is 10cm in front of the wrist, same
  /// direction as the wrist
  /// \code
  ///   geometry_msgs::Pose pose;
  ///   pose.position.x = 0.1;
  ///   pose.orientation.w = 1;
  ///   graph.Add("tooltip", transform_graph::RefFrame("wrist"), pose);
  /// \endcode
  void Add(const std::string& name, const RefFrame& ref_frame_name,
           const Transform& transform);

  /// \brief Returns whether an undirected path exists between two frames.
  ///
  /// \returns true if an undirected path exists between the given frames,
  /// false otherwise.
  bool CanTransform(const std::string& frame1_id,
                    const std::string& frame2_id) const;

  /// \brief Computes the transform describing the local frame relative to the
  /// reference frame.
  ///
  /// Multiplying this transform with a point expressed in the
  /// local frame results in the same point expressed in the reference frame.
  /// The position component of the returned homogeneous matrix is the origin of
  /// the local frame expressed in the reference frame. The columns of the
  /// rotation matrix are the principal axes of the local frame expressed in the
  /// reference frame.
  ///
  /// \param[in] local The frame whose description you want.
  /// \param[in] reference The reference frame you want the frame described in.
  /// \param[out] transform The output transform. Call matrix() to get the
  ///   homogeneous matrix as an Eigen::Matrix4d.
  ///
  /// \returns true if the transform exists, false otherwise.
  ///
  /// \b Example: describe the wrist frame relative to the base frame
  /// \code
  ///   graph.ComputeDescription(transform_graph::LocalFrame("wrist"),
  ///                            transform_graph::RefFrame("base"),
  ///                            &wrist_in_base);
  /// \endcode
  ///
  /// The origin of the wrist in the base frame is:
  /// \code
  ///   wrist_in_base.matrix().topRightCorner(3, 1)
  /// \endcode
  ///
  /// The columns of the rotation matrix are the x, y, and z axes of the wrist
  /// frame, expressed in the frame of the base:
  /// \code
  ///   wrist_in_base.matrix().topLeftCorner(3, 3)
  /// \endcode
  ///
  /// \note
  /// If you are just going to transform one or two points, using
  /// #DescribePosition is more efficient. Use this method to compute the
  /// transform just once if you are going to transform many points.
  bool ComputeDescription(const LocalFrame& local, const RefFrame& reference,
                          Transform* transform) const;

  /// \brief Computes the transform that maps points in the "from" frame to the
  /// "to" frame.
  ///
  /// To do the mapping, multiply the point with this transform. You can think
  /// of this multiplication as applying the same rotation and translation to
  /// get from the "from" frame to the "to" frame.
  ///
  /// \param[in] from The frame you want to map points from.
  /// \param[in] to The frame you want to map points into.
  /// \param[out] transform The output transform. Call matrix() to get the
  ///   homogeneous matrix as an Eigen::Matrix4d.
  ///
  /// \returns true if the transform exists, false otherwise.
  ///
  /// \b Example: get the mapping from left hand to right hand
  /// \code
  ///   graph.Add("left hand", transform_graph::RefFrame("base"), ...);
  ///   graph.Add("right hand", transform_graph::RefFrame("base"), ...);
  ///   graph.ComputeMapping(transform_graph::From("left hand"),
  ///                        transform_graph::To("right hand"), &left_to_right);
  /// \endcode
  ///
  /// Now multiplying left_to_right by the point (0.1, 0, 0) in front of the
  /// left hand results in a point 10 cm in front of the right hand. However,
  /// this point is still expressed in the frame of the left hand.
  ///
  /// \note
  /// If you are just going to map one or two points, using #MapPosition is more
  /// efficient. Use this method to compute the transform just once if you are
  /// going to transform many points.
  bool ComputeMapping(const From& from, const To& to,
                      Transform* transform) const;

  /// \brief Describe a pose in a new frame.
  ///
  /// This is similar to simply adding a new frame to the graph and calling
  /// #ComputeDescription. See #DescribePosition for an example that doesn't
  /// involve orientation.
  ///
  /// \param[in] in The transform (e.g., a Pose) you want to describe in another
  ///   frame.
  /// \param[in] source_frame The name of the frame that the given pose is
  ///   currently expressed in.
  /// \param[in] target_frame The name of the frame that you would like the pose
  ///   to be described in.
  /// \param[out] out The transform that describes the given transform in
  ///   the target reference frame.
  ///
  /// \returns true if the transform could be computed, false otherwise.
  ///
  /// \b Example: get the frame 10 cm in front of the wrist in base coordinates
  /// \code
  ///   Pose p;
  ///   p.position.x = 0.1;
  ///   p.orientation.w = 1;
  ///   transform_graph::Transform pose_in_base
  ///   graph.TransformFrame(p, transform_graph::Source("wrist"),
  ///                        transform_graph::Target("base"), &pose_in_base);
  /// \endcode
  ///
  /// \note
  /// Calling this method computes the entire chain of transformations, so if
  /// you are just going to transform many poses, compute the transform once
  /// using #ComputeDescription.
  bool DescribePose(const Transform& in, const Source& source_frame,
                    const Target& target_frame, Transform* out) const;

  /// \brief Map a pose into a frame.
  ///
  /// Given a pose, this computes what that pose would be in the "to" frame. The
  /// result is expressed, however, in the "from" frame. See #MapPosition for an
  /// example that doesn't involve orientation.
  ///
  /// \param[in] in The pose you want to map to a different frame.
  /// \param[in] frame_in The frame the pose is currently expressed in.
  /// \param[in] frame_out The frame you want to map the pose into.
  /// \param[out] out The output pose, expressed in frame_in.
  ///
  /// \returns true if the transform could be computed, false otherwise.
  ///
  /// \note
  /// Calling this method computes the entire chain of transformations, so if
  /// you are just going to transform many poses, compute the transform once
  /// using #ComputeMapping.
  bool MapPose(const Transform& in, const From& frame_in, const To& frame_out,
               Transform* out) const;

  /// \brief Describes a position in a new frame.
  ///
  /// \param[in] in The position you want to describe in another frame.
  /// \param[in] source_frame The frame the position is currently in.
  /// \param[in] target_frame The frame you want the position to be in.
  /// \param[out] out The given position, expressed in the target frame.
  ///
  /// \returns true if the transform could be computed, false otherwise.
  ///
  /// \b Example: get the position 10 cm in front of the wrist in the base frame
  /// \code
  ///   PointStamped ps;
  ///   ps.header.frame_id = "wrist";
  ///   ps.pose.position.x = 0.1;
  ///   graph.DescribePosition(ps.point, transform_graph::Source("wrist"),
  ///                          transform_graph::Target("base"), &ps_base)
  /// \endcode
  ///
  /// \note
  /// Calling this method computes the entire chain of transformations, so if
  /// you are just going to transform many points, compute the transform once
  /// using #ComputeDescription.
  bool DescribePosition(const Position& in, const Source& source_frame,
                        const Target& target_frame, Position* out) const;

  /// \brief Maps a position into a frame.
  ///
  /// Given a position like (1, 0, 0), this computes what (1, 0, 0) is in the
  /// "to" frame, and expresses the result in the "from" frame.
  /// For example, if left_hand is at (0, 0.3, 0) and right_hand is at
  /// (0, -0.3, 0) and they have the same orientation, then:
  ///
  /// \code
  ///   geometry_msgs::Point pt;
  ///   pt.x = 1; pt.y = 0; pt.z = 0;
  ///   geometry_msgs::Point pt_out;
  ///   MapPosition(pt, transform_graph::From("left_hand"),
  ///               transform_graph::To("right_hand"), &pt_out);
  /// \endcode
  ///
  /// pt_out will be (1, -0.6, 0);
  ///
  /// \param[in] in The position you want to map into another frame.
  /// \param[in] frame_in The frame the position is currently in.
  /// \param[in] frame_out The frame you want to map the position into.
  /// \param[out] out The given position, but now sitting in the "from" frame,
  ///   but numerically expressed in the "from" frame.
  ///
  /// \returns true if the transform could be computed, false otherwise.
  ///
  /// \note
  /// Calling this method computes the entire chain of transformations, so if
  /// you are just going to map many points, compute the transform once
  /// using #ComputeMapping.
  bool MapPosition(const Position& in, const From& frame_in,
                   const To& frame_out, Position* out) const;

 private:
  bool GetTransform(const std::string& source, const std::string& target,
                    Eigen::Affine3d* out) const;

  std::map<std::pair<std::string, std::string>, Transform> transforms_;
  internal::Graph graph_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_GRAPH_H_
