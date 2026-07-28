#include "finenav_simulator/simulator_resetter.hpp"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

namespace finenav_2d
{

SimulatorResetter::SimulatorResetter(rclcpp::Node * node)
: node_(node),
  ign_node_(std::make_unique<gz::transport::Node>())
{
  // -- World / robot identity ------------------------------------------------
  // Parameters that identify the Gazebo world and robot for Ignition RPCs.
  world_name_ = node_->declare_parameter("world_name", "default");
  robot_name_ = node_->declare_parameter("robot_name", "fine_robot");

  // -- Spawn pose ------------------------------------------------------------
  // The launch file passes spawn_* as strings (legacy convention), so we
  // declare them as string parameters and convert to double at read time.
  auto get_or_declare_double = [&](const std::string& name, double default_val) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, std::to_string(default_val));
    }
    return std::stod(node_->get_parameter(name).as_string());
  };
  spawn_x_   = get_or_declare_double("spawn_x",   0.0);
  spawn_y_   = get_or_declare_double("spawn_y",   0.0);
  spawn_z_   = get_or_declare_double("spawn_z",   0.0);
  spawn_yaw_ = get_or_declare_double("spawn_yaw", 0.0);

  // -- Topic names -----------------------------------------------------------
  std::string cmd_vel_topic =
    node_->declare_parameter("teleop_output_topic", "/cmd_vel");
  std::string keypress_topic =
    node_->declare_parameter("teleop_input_topic", "/keyboard/keypress");

  // -- Publishers & subscribers ----------------------------------------------
  // We publish a zero Twist during reset to stop the robot.
  cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(10));

  // The reset service is namespaced under the node (e.g.
  // /simulator_helper_node/reset_simulation).
  reset_service_ = node_->create_service<std_srvs::srv::Empty>(
    "~/reset_simulation",
    std::bind(&SimulatorResetter::OnResetService, this,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3));

  // Subscribe to Gazebo keypress events bridged to ROS 2 by ros_gz_bridge.
  keypress_subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
    keypress_topic, rclcpp::QoS(10),
    std::bind(&SimulatorResetter::OnKeypress, this, std::placeholders::_1));

  // -- Wait for Ignition Transport discovery ---------------------------------
  // Ignition Transport nodes need a brief startup period to discover peers
  // (Gazebo server) before they can make RPC calls. 500ms is conservative for
  // a local machine; if resets fail early, this value can be increased.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(node_->get_logger(),
    "Reset ready: press 'R' in Gazebo or 'ros2 service call %s/reset_simulation "
    "std_srvs/srv/Empty' (world='%s', robot='%s', spawn=(%.2f,%.2f,%.2f,%.2f))",
    node_->get_name(), world_name_.c_str(), robot_name_.c_str(),
    spawn_x_, spawn_y_, spawn_z_, spawn_yaw_);
}

SimulatorResetter::~SimulatorResetter() = default;

// ============================================================================
//  DoReset — the core four-step reset sequence
// ============================================================================

void SimulatorResetter::DoReset()
{
  const std::string world_prefix = "/world/" + world_name_;

  // Step 1: reset simulation time to zero.
  // On failure the simulation is in an unknown state — abort immediately.
  {
    gz::msgs::WorldControl req;
    gz::msgs::Boolean rep;
    req.mutable_reset()->set_all(true);

    bool result = false;
    bool called = ign_node_->Request(
      world_prefix + "/control", req, 5000, rep, result);

    if (!called || !result || !rep.data()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Reset FAILED at step 1/4 (time reset): called=%d, result=%d, data=%d",
        called, result, rep.data());
      return;
    }
  }

  // Step 2: teleport the robot back to its spawn pose.
  // If this fails we MUST abort — the robot could be anywhere.
  {
    gz::msgs::Pose req;
    req.set_name(robot_name_);
    req.mutable_position()->set_x(spawn_x_);
    req.mutable_position()->set_y(spawn_y_);
    req.mutable_position()->set_z(spawn_z_);

    // Convert yaw to quaternion: (0, 0, sin(yaw/2), cos(yaw/2)).
    req.mutable_orientation()->set_x(0.0);
    req.mutable_orientation()->set_y(0.0);
    req.mutable_orientation()->set_z(std::sin(spawn_yaw_ / 2.0));
    req.mutable_orientation()->set_w(std::cos(spawn_yaw_ / 2.0));

    gz::msgs::Boolean rep;
    bool result = false;
    bool called = ign_node_->Request(
      world_prefix + "/set_pose", req, 5000, rep, result);

    if (!called || !result || !rep.data()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Reset FAILED at step 2/4 (set_pose): called=%d, result=%d, data=%d",
        called, result, rep.data());
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
      "Reset: robot '%s' teleported to (%.2f, %.2f, %.2f, yaw=%.2f)",
      robot_name_.c_str(), spawn_x_, spawn_y_, spawn_z_, spawn_yaw_);
  }

  // Step 3: stop the robot by publishing zero velocity.
  // This is a fire-and-forget ROS publish — it cannot fail.
  cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});

  // Step 4: pause the simulation so the robot stays at the spawn position.
  {
    gz::msgs::WorldControl req;
    gz::msgs::Boolean rep;
    req.set_pause(true);

    bool result = false;
    bool called = ign_node_->Request(
      world_prefix + "/control", req, 3000, rep, result);

    if (!called || !result || !rep.data()) {
      RCLCPP_ERROR(node_->get_logger(),
        "Reset FAILED at step 4/4 (pause): called=%d, result=%d, data=%d. "
        "Time and pose were reset successfully but simulation is not paused.",
        called, result, rep.data());
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Simulation paused.");
  }
}

// ============================================================================
//  Callbacks
// ============================================================================

void SimulatorResetter::OnKeypress(const std_msgs::msg::Int32::SharedPtr msg)
{
  // R key (ASCII 82) triggers a reset.
  if (msg->data != 'R') {
    return;
  }

  // Prevent overlapping reset sequences (e.g. R key + service arriving
  // concurrently).
  bool expected = false;
  if (!resetting_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(node_->get_logger(),
      "Reset already in progress — ignoring keypress.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "R key pressed — resetting simulation.");

  // Run the reset on a dedicated thread. Ignition Transport RPCs are
  // synchronous and can take seconds; we must not block the ROS executor.
  std::thread([this]() {
    DoReset();
    resetting_ = false;
  }).detach();
}

void SimulatorResetter::OnResetService(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  bool expected = false;
  if (!resetting_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(node_->get_logger(),
      "Reset already in progress — ignoring service call.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Reset service called — resetting simulation.");

  std::thread([this]() {
    DoReset();
    resetting_ = false;
  }).detach();
}

}  // namespace finenav_2d
