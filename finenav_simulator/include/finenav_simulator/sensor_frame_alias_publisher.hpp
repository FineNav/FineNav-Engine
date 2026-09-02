#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace finenav_2d
{

/// Publishes static identity transforms that alias sensor frames.
///
/// Gazebo scopes sensor frames under the model and link names, so bridged
/// messages carry frame ids like `fine_robot/imu/imu_sensor`, while the ROS
/// ecosystem refers to the plain URDF link names (`imu`, `lidar_frame`).  These
/// aliases make both names resolve to the same frame in the TF tree.  fast_lio
/// relies on the lidar alias in particular: the point-cloud converter relabels
/// its output to `lidar_frame` without transforming the points, which is only
/// correct because this alias says `lidar_frame` *is* the sensor frame.
///
/// Two aliases are configured by four parameters, one (parent, child) pair
/// each; an empty child frame disables that alias.
class SensorFrameAliasPublisher
{
public:
  SensorFrameAliasPublisher(
    rclcpp::Node * node, std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster);

private:
  void PublishAlias(const std::string & parent_frame, const std::string & child_frame);

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::string imu_alias_parent_frame_;
  std::string imu_alias_child_frame_;
  std::string lidar_alias_parent_frame_;
  std::string lidar_alias_child_frame_;
};

}  // namespace finenav_2d
