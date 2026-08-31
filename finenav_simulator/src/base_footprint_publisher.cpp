#include "finenav_simulator/base_footprint_publisher.hpp"

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "finenav_simulator/parameter_helper.hpp"

namespace finenav_2d
{

BaseFootprintPublisher::BaseFootprintPublisher(
  rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster)
: node_(node), tf_broadcaster_(tf_broadcaster)
{
  enabled_ = DeclareOrGetParameter<bool>(node_, "enable_base_footprint", true);
  input_odom_topic_ = DeclareOrGetParameter<std::string>(node_, "input_odom_topic", "/odom");
  odom_frame_ = DeclareOrGetParameter<std::string>(node_, "path_odom_frame", "");
  base_frame_ = DeclareOrGetParameter<std::string>(node_, "path_base_frame", "");
  base_footprint_frame_ =
    DeclareOrGetParameter<std::string>(node_, "base_footprint_frame", "base_footprint");
  ground_offset_ = DeclareOrGetParameter<double>(node_, "footprint_ground_offset", 0.0);

  if (!enabled_) {
    return;
  }

  // Same QoS as OdomPathPublisher.  Both hold their own subscription to the one odometry
  // topic; that costs one extra 50 Hz message and keeps each class to a single job.
  odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&BaseFootprintPublisher::OdomCallback, this, std::placeholders::_1));
}

void BaseFootprintPublisher::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const std::string odom_frame = odom_frame_.empty() ? msg->header.frame_id : odom_frame_;
  const std::string base_frame = base_frame_.empty() ? msg->child_frame_id : base_frame_;

  if (base_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Received odometry without a child_frame_id, skipping base_footprint publication.");
    return;
  }

  // The maths below treats the message orientation as `odom -> base_link`.  If the
  // incoming odometry is expressed between some other pair of frames, stripping its
  // roll and pitch does not yield a frame that is level with respect to `odom`, so warn
  // rather than publish a silently wrong transform.
  if (!msg->header.frame_id.empty() && !odom_frame.empty() && msg->header.frame_id != odom_frame) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Odometry frame_id '%s' does not match the configured odom frame '%s'; "
      "base_footprint would not be level with respect to '%s'.",
      msg->header.frame_id.c_str(), odom_frame.c_str(), odom_frame.c_str());
  }
  if (!msg->child_frame_id.empty() && msg->child_frame_id != base_frame) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Odometry child_frame_id '%s' does not match the configured base frame '%s'.",
      msg->child_frame_id.c_str(), base_frame.c_str());
  }

  // Let R be the `odom -> base_link` rotation with roll/pitch/yaw (r, p, y).  We want
  // `odom -> base_footprint` to be Rz(y): same heading, no tilt.  Publishing that as the
  // leaf edge `base_link -> base_footprint` means expressing it in base_link's own axes:
  //
  //     rotation = R^-1 * Rz(y)  ==  Rx(-r) * Ry(-p)
  //
  // i.e. undo the body tilt.  The translation is the constant (0, 0, -offset) in body
  // axes: the wheel contact point is a fixed point of the body, so it needs no rotating.
  tf2::Quaternion q_base(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q_base).getRPY(roll, pitch, yaw);

  tf2::Quaternion q_yaw_only;
  q_yaw_only.setRPY(0.0, 0.0, yaw);
  const tf2::Quaternion q_level = q_base.inverse() * q_yaw_only;

  geometry_msgs::msg::TransformStamped footprint_tf;
  // Reuse the odometry stamp so this edge and `odom -> base_link` share one instant;
  // consumers chaining the two never have to extrapolate.
  footprint_tf.header.stamp = msg->header.stamp;
  footprint_tf.header.frame_id = base_frame;
  footprint_tf.child_frame_id = base_footprint_frame_;
  footprint_tf.transform.translation.x = 0.0;
  footprint_tf.transform.translation.y = 0.0;
  footprint_tf.transform.translation.z = -ground_offset_;
  footprint_tf.transform.rotation.x = q_level.x();
  footprint_tf.transform.rotation.y = q_level.y();
  footprint_tf.transform.rotation.z = q_level.z();
  footprint_tf.transform.rotation.w = q_level.w();
  tf_broadcaster_->sendTransform(footprint_tf);
}

}  // namespace finenav_2d
