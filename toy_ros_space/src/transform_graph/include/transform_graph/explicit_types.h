// This file defines wrappers around strings that are used to document
// arguments.

#ifndef _TRANSFORM_GRAPH_EXPLICIT_TYPES_H_
#define _TRANSFORM_GRAPH_EXPLICIT_TYPES_H_

#include <string>

namespace transform_graph {
/// Source represents the name of a frame in which some data originates.
class Source {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit Source(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};

/// Target represents the name of a frame we want to express the data in.
class Target {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit Target(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};

/// LocalFrame represents the name of a transform.
class LocalFrame {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit LocalFrame(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};

/// RefFrame represents the name of a transform's reference frame.
class RefFrame {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit RefFrame(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};

/// From represents the name of a frame we want to map from.
class From {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit From(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};

/// To represents the name of a frame we want to map into.
class To {
 public:
  /// Constructor
  ///
  /// \param[in] id The name of the frame.
  explicit To(const std::string& id);

  /// \returns The name of the frame.
  std::string id() const;

 private:
  const std::string& id_;
};
}  // namespace transform_graph

#endif  // _TRANSFORM_GRAPH_EXPLICIT_TYPES_H_
