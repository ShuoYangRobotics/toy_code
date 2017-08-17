# transform_graph

[![Build Status](https://travis-ci.org/jstnhuang/transform_graph.svg?branch=indigo-devel)](https://travis-ci.org/jstnhuang/transform_graph)
[![Coverage Status](https://coveralls.io/repos/github/jstnhuang/transform_graph/badge.svg?branch=indigo-devel)](https://coveralls.io/github/jstnhuang/transform_graph?branch=indigo-devel)

`transform_graph` is a library that computes transformations betweens arbitrary frames in a graph of transformations.
See the generated documentation for details.

Basic example:
```cpp
#include "transform_graph/transform_graph.h"

transform_graph::Graph graph;
graph.Add("wrist", transform_graph::RefFrame("base_link"), wrist_pose_stamped);
graph.Add("kinect", transform_graph::RefFrame("base_link"), kinect_pose_stamped);

// Find out if a point in the frame of the Kinect is near the wrist.
pcl::PointXYZ point_in_kinect = ...;
transform_graph::Position point_in_wrist;
graph.DescribePosition(point_in_kinect, transform_graph::Source("kinect"), transform_graph::Target("wrist"), &point_in_wrist);
if (point_in_wrist.vector().norm() < 0.05) {
  ...
}
```
