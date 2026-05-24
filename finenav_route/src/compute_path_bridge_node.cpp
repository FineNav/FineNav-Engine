// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_path.hpp"
#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "finenav_route/route_path_planner.hpp"

namespace finenav_route
{

namespace
{

constexpr double kDuplicatePoseTolerance = 1e-6;

geometry_msgs::msg::Quaternion orientationAroundZAxis(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double planarDistance(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  return std::hypot(
    a.position.x - b.position.x,
    a.position.y - b.position.y);
}

}  // namespace

class ComputePathBridge : public rclcpp::Node
{
public:
  using ComputePath = finenav_msgs::action::ComputePath;
  using ComputePlan = finenav_msgs::action::ComputePlan;
  using ComputePathGoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

  ComputePathBridge()
  : rclcpp::Node("compute_path_bridge")
  {
    action_name_ = declare_parameter<std::string>("action_name", "compute_path");
    controller_action_name_ =
      declare_parameter<std::string>("controller_action_name", "mppi_layer");
    path_topic_ = declare_parameter<std::string>("path_topic", "line_path");
    nav_state_topic_ = declare_parameter<std::string>("nav_state_topic", "/nav_state");
    server_timeout_s_ = declare_parameter<double>("server_timeout_s", 2.0);
    nav_state_timeout_s_ = declare_parameter<double>("nav_state_timeout_s", 2.0);
    connector_density_ = declare_parameter<double>("connector_density", 0.05);
    use_current_pose_as_start_ = declare_parameter<bool>("use_current_pose_as_start", true);

    if (connector_density_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "connector_density must be positive. Falling back to 0.05 m.");
      connector_density_ = 0.05;
    }

    path_pub_ = create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
    nav_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      nav_state_topic_,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        latest_nav_state_ = *msg;
      });
    controller_client_ =
      rclcpp_action::create_client<ComputePlan>(this, controller_action_name_);

    action_server_ = rclcpp_action::create_server<ComputePath>(
      this,
      action_name_,
      std::bind(&ComputePathBridge::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ComputePathBridge::handleCancel, this, std::placeholders::_1),
      std::bind(&ComputePathBridge::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ComputePath bridge ready: action='%s', path_topic='%s', controller_action='%s', "
      "nav_state_topic='%s'",
      action_name_.c_str(), path_topic_.c_str(), controller_action_name_.c_str(),
      nav_state_topic_.c_str());
  }

private:
  template <typename FutureT>
  bool waitForFuture(FutureT & future, const std::shared_ptr<ComputePathGoalHandle> & goal_handle)
  {
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        return false;
      }
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const ComputePath::Goal> goal)
  {
    if (goal->graph_json.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting ComputePath goal with empty graph_json.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->planner_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting ComputePath goal with empty planner_id.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<ComputePathGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<ComputePathGoalHandle> goal_handle)
  {
    std::thread{std::bind(&ComputePathBridge::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<ComputePathGoalHandle> goal_handle)
  {
    const auto start_time = now();
    const auto goal = goal_handle->get_goal();

    unsigned int route_start_node_id = goal->start_node_id;
    geometry_msgs::msg::Pose current_pose;
    bool prepend_current_pose_connector = false;

    if (use_current_pose_as_start_) {
      if (!waitForCurrentPose(current_pose, goal_handle)) {
        RCLCPP_ERROR(
          get_logger(),
          "No nav_state received within %.2f s. Cannot connect current pose to route.",
          nav_state_timeout_s_);
        goal_handle->abort(makeResult(start_time));
        return;
      }

      try {
        const auto nearest = findNearestNode(goal->graph_json, current_pose);
        route_start_node_id = nearest.id;
        prepend_current_pose_connector = true;
        RCLCPP_INFO(
          get_logger(),
          "Using current pose (%.3f, %.3f) as virtual start. "
          "Nearest route node is %u at %.3f m.",
          current_pose.position.x, current_pose.position.y,
          nearest.id, nearest.distance);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Failed to find nearest route node: %s", ex.what());
        goal_handle->abort(makeResult(start_time));
        return;
      }
    }

    finenav_msgs::msg::Trajectory path;
    try {
      path = route_path_planner_.plan(
        goal->graph_json, route_start_node_id, goal->goal_node_id);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Route path planning failed: %s", ex.what());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    if ((path.valid_fields & finenav_msgs::msg::Trajectory::TRAJ_POSE) == 0 ||
      path.poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "Route path planning produced an empty pose trajectory.");
      goal_handle->abort(makeResult(start_time));
      return;
    }

    if (prepend_current_pose_connector) {
      prependCurrentPoseConnector(current_pose, path);
    }

    path.header.stamp = now();
    if (path.header.frame_id.empty()) {
      path.header.frame_id = "map";
    }

    path_pub_->publish(path);
    RCLCPP_INFO(
      get_logger(),
      "Published route reference trajectory with %zu poses from node %u to node %u.",
      path.poses.size(), route_start_node_id, goal->goal_node_id);

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(server_timeout_s_));
    if (!controller_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(
        get_logger(),
        "Controller action server '%s' is not available.",
        controller_action_name_.c_str());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    ComputePlan::Goal controller_goal;
    controller_goal.goal = path.poses.back();
    controller_goal.planner_id = goal->planner_id;

    auto controller_goal_future = controller_client_->async_send_goal(controller_goal);
    if (!waitForFuture(controller_goal_future, goal_handle)) {
      goal_handle->canceled(makeResult(start_time));
      return;
    }

    const auto controller_goal_handle = controller_goal_future.get();
    if (!controller_goal_handle) {
      RCLCPP_ERROR(get_logger(), "Controller rejected ComputePlan goal.");
      goal_handle->abort(makeResult(start_time));
      return;
    }

    auto result_future = controller_client_->async_get_result(controller_goal_handle);
    if (!waitForFuture(result_future, goal_handle)) {
      controller_client_->async_cancel_goal(controller_goal_handle);
      goal_handle->canceled(makeResult(start_time));
      return;
    }

    const auto controller_result = result_future.get();
    if (controller_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal_handle->succeed(makeResult(start_time));
      return;
    }

    if (controller_result.code == rclcpp_action::ResultCode::CANCELED) {
      goal_handle->canceled(makeResult(start_time));
      return;
    }

    goal_handle->abort(makeResult(start_time));
  }

  ComputePath::Result::SharedPtr makeResult(const rclcpp::Time & start_time) const
  {
    auto result = std::make_shared<ComputePath::Result>();
    result->planning_time = now() - start_time;
    return result;
  }

  bool waitForCurrentPose(
    geometry_msgs::msg::Pose & pose,
    const std::shared_ptr<ComputePathGoalHandle> & goal_handle)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(nav_state_timeout_s_);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (goal_handle->is_canceling()) {
        return false;
      }

      {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        if (latest_nav_state_) {
          pose = latest_nav_state_->pose.pose;
          return true;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return false;
  }

  struct NearestNode
  {
    unsigned int id{0};
    double distance{0.0};
  };

  NearestNode findNearestNode(
    const std::string & graph_json,
    const geometry_msgs::msg::Pose & pose)
  {
    Graph graph;
    GraphToIDMap id_to_graph_map;
    GeoJsonGraphFileLoader loader;
    if (!loader.loadGraphFromJson(graph, id_to_graph_map, graph_json)) {
      throw std::runtime_error("Failed to load route graph json");
    }

    if (graph.empty()) {
      throw std::runtime_error("Route graph contains no nodes");
    }

    NearestNode nearest;
    nearest.distance = std::numeric_limits<double>::max();
    for (const auto & node : graph) {
      const double dx = static_cast<double>(node.coords.x) - pose.position.x;
      const double dy = static_cast<double>(node.coords.y) - pose.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest.distance) {
        nearest.id = node.nodeid;
        nearest.distance = distance;
      }
    }

    return nearest;
  }

  void prependCurrentPoseConnector(
    const geometry_msgs::msg::Pose & current_pose,
    finenav_msgs::msg::Trajectory & path) const
  {
    if (path.poses.empty()) {
      return;
    }

    const auto first_route_pose = path.poses.front();
    const double distance = planarDistance(current_pose, first_route_pose);
    if (distance <= kDuplicatePoseTolerance) {
      assignPlanarOrientations(path.poses);
      return;
    }

    std::vector<geometry_msgs::msg::Pose> connected_poses;
    const auto segments = std::max(
      1u,
      static_cast<unsigned int>(std::ceil(distance / connector_density_)));
    connected_poses.reserve(path.poses.size() + segments);

    for (unsigned int i = 0; i < segments; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(segments);
      geometry_msgs::msg::Pose pose;
      pose.position.x =
        current_pose.position.x +
        (first_route_pose.position.x - current_pose.position.x) * t;
      pose.position.y =
        current_pose.position.y +
        (first_route_pose.position.y - current_pose.position.y) * t;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      connected_poses.push_back(pose);
    }

    connected_poses.insert(connected_poses.end(), path.poses.begin(), path.poses.end());
    assignPlanarOrientations(connected_poses);
    path.poses = std::move(connected_poses);
    path.valid_fields |= finenav_msgs::msg::Trajectory::TRAJ_POSE;
  }

  void assignPlanarOrientations(std::vector<geometry_msgs::msg::Pose> & poses) const
  {
    if (poses.size() < 2) {
      return;
    }

    for (size_t i = 0; i + 1 < poses.size(); ++i) {
      const auto & current = poses[i];
      const auto & next = poses[i + 1];
      const double dx = next.position.x - current.position.x;
      const double dy = next.position.y - current.position.y;
      if (std::hypot(dx, dy) <= kDuplicatePoseTolerance) {
        if (i > 0) {
          poses[i].orientation = poses[i - 1].orientation;
        }
        continue;
      }
      poses[i].orientation = orientationAroundZAxis(std::atan2(dy, dx));
    }

    poses.back().orientation = poses[poses.size() - 2].orientation;
  }

  std::string action_name_;
  std::string controller_action_name_;
  std::string path_topic_;
  std::string nav_state_topic_;
  double server_timeout_s_{2.0};
  double nav_state_timeout_s_{2.0};
  double connector_density_{0.05};
  bool use_current_pose_as_start_{true};

  rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;
  rclcpp_action::Client<ComputePlan>::SharedPtr controller_client_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;
  RoutePathPlanner route_path_planner_;
  std::mutex nav_state_mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_nav_state_;
};

}  // namespace finenav_route

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<finenav_route::ComputePathBridge>());
  rclcpp::shutdown();
  return 0;
}
