#pragma once

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace finenav_2d
{

/// Publishes `base_link -> base_footprint`: the robot's pose projected onto the ground,
/// kept level no matter how the body is tilted.
///
/// The frame name and semantics come from REP-120 (coordinate frames for humanoid
/// robots), which specifies that relative to `odom` the roll and pitch angles are zero
/// and the yaw matches `base_link`.  Note it is *not* one of the four frames REP-105
/// defines -- wheeled stacks borrowed it from REP-120 as a convention.
///
/// Its only consumer today is the rviz Grid display, which needs a mount point that
/// follows the robot while staying level: the worlds spawn the robot far from the
/// origin (map_vocano starts 64 m out and climbs ~19 m), so a grid pinned to `odom` is
/// off screen at startup and metres underfoot after a climb, while a grid on `base_link`
/// would tilt with the body and cut into the terrain.
///
/// Why a node instead of a fixed joint in the xacro: a fixed joint is rigid, so the
/// child would inherit the parent's tilt and the roll/pitch could never be removed.
/// REP-120 reaches the same conclusion for humanoids, though for a different reason
/// (their footprint translation moves as the feet swing; our wheels have no suspension,
/// so the translation is a constant and only the angles need stripping).
class BaseFootprintPublisher
{
public:
  BaseFootprintPublisher(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool enabled_;
  std::string input_odom_topic_;
  std::string odom_frame_;
  /// Parent of the published edge.  Read from `path_base_frame` -- the launch file
  /// already sets that to `base_link` for both LIO modes, so reusing it avoids a second
  /// parameter carrying the same truth.
  std::string base_frame_;
  std::string base_footprint_frame_;
  double ground_offset_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

}  // namespace finenav_2d
