#include "finenav_simulator/ground_truth_map_localizer.hpp"

#include <string>

#include "finenav_simulator/parameter_helper.hpp"

namespace finenav_2d
{

GroundTruthMapPublisher::GroundTruthMapPublisher(
  rclcpp::Node * node, std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster)
: node_(node),
  static_tf_broadcaster_(static_tf_broadcaster)
{
  enable_identity_map_to_odom_ =
    DeclareOrGetParameter<bool>(node_, "enable_identity_map_to_odom", false);
  map_frame_ = DeclareOrGetParameter<std::string>(node_, "map_frame", "map");
  odom_frame_ = DeclareOrGetParameter<std::string>(node_, "odom_frame", "odom");

  if (enable_identity_map_to_odom_) {
    PublishIdentityMapToOdom();
  }
}

void GroundTruthMapPublisher::PublishIdentityMapToOdom() const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->get_clock()->now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(transform);
}

}  // namespace finenav_2d
