#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <ignition/transport/Node.hh>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"

namespace finenav_2d
{

/// Resets the Gazebo simulation to a clean state in four steps:
///   1. Reset simulation time to zero.
///   2. Teleport the robot back to its configured spawn pose.
///   3. Publish zero velocity (stop the robot).
///   4. Pause the simulation.
///
/// ## Triggering a reset
///
/// - Press **R** in the Gazebo window (KeyPublisher → /keyboard/keypress → ROS 2).
/// - Call the ROS 2 service `~/reset_simulation` (std_srvs/srv/Empty) from a
///   script or test.
///
/// ## Why Ignition Transport instead of ROS 2 services
///
/// Gazebo Fortress's built-in WorldControl.reset.all only resets simulation
/// time; it does **not** move the robot. We additionally call `set_pose` and
/// `pause` through Ignition Transport to make the reset truly useful.
///
/// ## Thread safety
///
/// DoReset() makes synchronous, long-running RPC calls (~seconds). To avoid
/// blocking the ROS 2 executor (and therefore ALL other callbacks), the reset
/// sequence runs on a **dedicated background thread**. An atomic flag prevents
/// overlapping resets when both the R key and the service are triggered
/// concurrently.
///
/// @note There is a theoretical race during ROS shutdown: if a reset thread is
///       still running when the SimulatorResetter is destroyed, the detached
///       thread will access freed memory. In practice this requires shutting
///       down the simulator during the brief reset window, which is rare.
class SimulatorResetter
{
public:
  explicit SimulatorResetter(rclcpp::Node * node);
  ~SimulatorResetter();

private:
  /// Execute the four-step reset sequence. Called on a background thread.
  void DoReset();

  /// Subscription callback: listens for keypress events bridged from Gazebo.
  void OnKeypress(const std_msgs::msg::Int32::SharedPtr msg);

  /// Service callback: handles external reset requests.
  void OnResetService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  rclcpp::Node * node_;
  std::string world_name_;
  std::string robot_name_;
  double spawn_x_, spawn_y_, spawn_z_, spawn_yaw_;

  /// Prevents overlapping DoReset() sequences from concurrent triggers.
  std::atomic<bool> resetting_{false};

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keypress_subscription_;

  /// Ignition Transport node — one-shot RPCs to Gazebo's private API.
  std::unique_ptr<ignition::transport::Node> ign_node_;
};

}  // namespace finenav_2d
