#include "finenav_simulator/sensor_frame_alias_publisher.hpp"

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace finenav_2d
{

SensorFrameAliasPublisher::SensorFrameAliasPublisher(
  rclcpp::Node * node, std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster)
: node_(node), static_tf_broadcaster_(static_tf_broadcaster)
{
  imu_alias_parent_frame_ = node_->declare_parameter("imu_alias_parent_frame", "imu");
  imu_alias_child_frame_ = node_->declare_parameter("imu_alias_child_frame", "");
  lidar_alias_parent_frame_ = node_->declare_parameter("lidar_alias_parent_frame", "lidar_frame");
  lidar_alias_child_frame_ = node_->declare_parameter("lidar_alias_child_frame", "");

  if (!imu_alias_child_frame_.empty()) {
    PublishAlias(imu_alias_parent_frame_, imu_alias_child_frame_);
  }
  if (!lidar_alias_child_frame_.empty()) {
    PublishAlias(lidar_alias_parent_frame_, lidar_alias_child_frame_);
  }
}

void SensorFrameAliasPublisher::PublishAlias(
  const std::string & parent_frame, const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->get_clock()->now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(transform);
}

}  // namespace finenav_2d
