// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>

#include "finenav_msgs/action/compute_path.hpp"
#include "finenav_msgs/action/navigate_to_pose.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "finenav_route/route_path_planner.hpp"
#include "bfs_algorithm.hpp"

namespace finenav_route
{

namespace
{

constexpr double kDuplicatePoseTolerance = 1e-6;
constexpr int kRouteCellZ = 0;
constexpr unsigned int kAutoStartNodeId = 0;

struct RouteStartCandidate
{
  unsigned int id{0};
  double distance{0.0};
  std::size_t bfs_depth{0};
};

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

double planarDistance(
  const Node & node,
  const geometry_msgs::msg::Pose & pose)
{
  return std::hypot(
    static_cast<double>(node.coords.x) - pose.position.x,
    static_cast<double>(node.coords.y) - pose.position.y);
}

bfs::Vec3 toRouteCell(const double x, const double y, const double resolution)
{
  return {
    static_cast<int>(std::floor(x / resolution)),
    static_cast<int>(std::floor(y / resolution)),
    kRouteCellZ
  };
}

RouteStartCandidate pickClosestNodeInCell(
  const Graph & graph,
  const std::vector<unsigned int> & indices,
  const geometry_msgs::msg::Pose & pose,
  const std::size_t bfs_depth)
{
  RouteStartCandidate best;
  best.distance = std::numeric_limits<double>::infinity();
  best.bfs_depth = bfs_depth;

  for (const auto index : indices) {
    const auto distance = planarDistance(graph[index], pose);
    if (distance < best.distance ||
      (std::abs(distance - best.distance) <= 1e-9 && graph[index].nodeid < best.id))
    {
      best.id = graph[index].nodeid;
      best.distance = distance;
    }
  }

  return best;
}

RouteStartCandidate findNearestStartNodeWithBfsPlanner(
  const Graph & graph,
  const geometry_msgs::msg::Pose & pose,
  const double resolution)
{
  if (graph.empty()) {
    throw std::runtime_error("Route graph contains no nodes");
  }

  if (resolution <= 0.0 || !std::isfinite(resolution)) {
    throw std::runtime_error("BFS nearest-node search resolution must be positive");
  }

  std::unordered_map<bfs::Vec3, std::vector<unsigned int>, bfs::Vec3Hash> nodes_by_cell;
  const auto start_cell = toRouteCell(pose.position.x, pose.position.y, resolution);
  int max_depth = 0;
  for (unsigned int index = 0; index < graph.size(); ++index) {
    const auto node_cell = toRouteCell(graph[index].coords.x, graph[index].coords.y, resolution);
    nodes_by_cell[node_cell].push_back(index);
    max_depth = std::max(
      max_depth,
      std::max(std::abs(node_cell.x - start_cell.x), std::abs(node_cell.y - start_cell.y)));
  }

  auto is_walkable = [max_depth, start_cell](const bfs::Vec3 & cell) {
    return cell.z == kRouteCellZ &&
      std::abs(cell.x - start_cell.x) <= max_depth &&
      std::abs(cell.y - start_cell.y) <= max_depth;
  };
  auto is_goal = [&nodes_by_cell](const bfs::Vec3 & cell) {
    return nodes_by_cell.contains(cell);
  };

  bfs::BfsAlgorithm bfs_search(is_walkable);
  bfs_search.setConnectivity(bfs::BfsAlgorithm::Connectivity::N8);
  bfs_search.setMaxIterations(-1);
  bfs_search.setMaxPlanningTime(5.0);

  const auto path = bfs_search.findPathToAny(start_cell, is_goal);
  if (path.has_value() && !path->empty()) {
    const auto cell_it = nodes_by_cell.find(path->back());
    if (cell_it != nodes_by_cell.end()) {
      return pickClosestNodeInCell(graph, cell_it->second, pose, path->size() - 1);
    }
  }

  throw std::runtime_error("BFS nearest-node search did not find any route node");
}

}  // namespace

class ComputePathBridge : public rclcpp::Node
{
public:
  using ComputePath = finenav_msgs::action::ComputePath;
  using NavigateToPose = finenav_msgs::action::NavigateToPose;
  using ComputePathGoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

  ComputePathBridge()
  : rclcpp::Node("compute_path_bridge")
  {
    action_name_ = declare_parameter<std::string>("action_name", "compute_path");
    navigator_action_name_ =
      declare_parameter<std::string>("navigator_action_name", "navigate_to_pose");
    navigator_behavior_tree_ =
      declare_parameter<std::string>("navigator_behavior_tree", "RouteNavigateToPose");
    graph_dir_ = declare_parameter<std::string>("graph_dir", "");
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

    try {
      graph_json_by_name_ = loadGraphsFromDirectory(graph_dir_);
      RCLCPP_INFO(
        get_logger(),
        "Loaded %zu route graph(s) from graph_dir='%s'.",
        graph_json_by_name_.size(), graph_dir_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load route graphs: %s", ex.what());
      throw;
    }

    path_pub_ = create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
    topological_path_pub_ = create_publisher<nav_msgs::msg::Path>("/topological_path", 10);
    nav_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      nav_state_topic_,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(nav_state_mutex_);
        latest_nav_state_ = *msg;
      });
    navigator_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, navigator_action_name_);

    action_server_ = rclcpp_action::create_server<ComputePath>(
      this,
      action_name_,
      std::bind(&ComputePathBridge::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ComputePathBridge::handleCancel, this, std::placeholders::_1),
      std::bind(&ComputePathBridge::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ComputePath bridge ready: action='%s', path_topic='%s', navigator_action='%s', "
      "nav_state_topic='%s', graph_dir='%s'",
      action_name_.c_str(), path_topic_.c_str(), navigator_action_name_.c_str(),
      nav_state_topic_.c_str(), graph_dir_.c_str());
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
    if (goal->goal_node_name.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting ComputePath goal with empty goal_node_name.");
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
    const auto & graph_json = getGraphJson();

    unsigned int route_start_node_id = 0;
    unsigned int route_goal_node_id = 0;

    try {
      route_goal_node_id = route_path_planner_.findNodeIdByFrame(graph_json, goal->goal_node_name);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to find goal node by name '%s': %s",
        goal->goal_node_name.c_str(), ex.what());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    geometry_msgs::msg::Pose current_pose;
    bool prepend_current_pose_connector = false;

    if (!waitForCurrentPose(current_pose, goal_handle)) {
      RCLCPP_ERROR(
        get_logger(),
        "No nav_state received within %.2f s. Cannot connect current pose to route.",
        nav_state_timeout_s_);
      goal_handle->abort(makeResult(start_time));
      return;
    }

    try {
      const auto start_candidate =
        findBfsStartNode(graph_json, current_pose);
      route_start_node_id = start_candidate.id;
      prepend_current_pose_connector = true;
      RCLCPP_INFO(
        get_logger(),
        "Using current pose (%.3f, %.3f) as virtual start. "
        "BFS-selected nearest route node is %u at %.3f m, search depth=%zu.",
        current_pose.position.x, current_pose.position.y,
        start_candidate.id, start_candidate.distance, start_candidate.bfs_depth);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to find nearest route start node by BFS: %s", ex.what());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    RoutePathPlanner::PlanResult plan_result;
    try {
      plan_result = route_path_planner_.plan(
        graph_json, route_start_node_id, route_goal_node_id);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Route path planning failed: %s", ex.what());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    auto & path = plan_result.trajectory;

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

    // Publish topological path
    nav_msgs::msg::Path topo_msg;
    topo_msg.header = path.header;
    for (const auto & pose : plan_result.topological_poses) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = topo_msg.header;
      ps.pose = pose;
      topo_msg.poses.push_back(ps);
    }
    topological_path_pub_->publish(topo_msg);

    RCLCPP_INFO(
      get_logger(),
      "Published route reference trajectory with %zu poses and topological path with %zu nodes from node %u to node %u.",
      path.poses.size(), plan_result.topological_poses.size(), route_start_node_id, route_goal_node_id);

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(server_timeout_s_));
    if (!navigator_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(
        get_logger(),
        "Navigator action server '%s' is not available.",
        navigator_action_name_.c_str());
      goal_handle->abort(makeResult(start_time));
      return;
    }

    NavigateToPose::Goal navigator_goal;
    navigator_goal.pose.header = path.header;
    navigator_goal.pose.pose = path.poses.back();
    navigator_goal.behavior_tree = navigator_behavior_tree_;

    auto navigator_goal_future = navigator_client_->async_send_goal(navigator_goal);
    if (!waitForFuture(navigator_goal_future, goal_handle)) {
      goal_handle->canceled(makeResult(start_time));
      return;
    }

    const auto navigator_goal_handle = navigator_goal_future.get();
    if (!navigator_goal_handle) {
      RCLCPP_ERROR(get_logger(), "Navigator rejected NavigateToPose goal.");
      goal_handle->abort(makeResult(start_time));
      return;
    }

    auto result_future = navigator_client_->async_get_result(navigator_goal_handle);
    if (!waitForFuture(result_future, goal_handle)) {
      navigator_client_->async_cancel_goal(navigator_goal_handle);
      goal_handle->canceled(makeResult(start_time));
      return;
    }

    const auto navigator_result = result_future.get();
    if (navigator_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal_handle->succeed(makeResult(start_time));
      return;
    }

    if (navigator_result.code == rclcpp_action::ResultCode::CANCELED) {
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

  static std::unordered_map<std::string, std::string> loadGraphsFromDirectory(
    const std::string & graph_dir)
  {
    if (graph_dir.empty()) {
      throw std::runtime_error("graph_dir parameter is empty");
    }

    const std::filesystem::path graph_dir_path(graph_dir);
    if (!std::filesystem::is_directory(graph_dir_path)) {
      throw std::runtime_error("graph_dir is not a directory: " + graph_dir);
    }

    std::unordered_map<std::string, std::string> graph_json_by_name;
    for (const auto & entry : std::filesystem::directory_iterator(graph_dir_path)) {
      if (!entry.is_regular_file()) {
        continue;
      }

      const auto graph_path = entry.path();
      const auto extension = graph_path.extension().string();
      if (extension != ".json" && extension != ".geojson") {
        continue;
      }

      const auto graph_file = graph_path.string();
      const auto graph_json = loadGraphJsonFromFile(graph_file);
      const auto graph_name = parseGraphName(graph_json, graph_path.stem().string());
      if (graph_name.empty()) {
        throw std::runtime_error("Route graph has empty name: " + graph_file);
      }
      if (graph_json_by_name.contains(graph_name)) {
        throw std::runtime_error("Duplicate route graph name: " + graph_name);
      }
      graph_json_by_name.emplace(graph_name, graph_json);
    }

    if (graph_json_by_name.empty()) {
      throw std::runtime_error("graph_dir contains no .json or .geojson route graphs");
    }
    return graph_json_by_name;
  }

  static std::string parseGraphName(
    const std::string & graph_json,
    const std::string & fallback_name)
  {
    const auto json = nlohmann::json::parse(graph_json);
    const auto name_it = json.find("name");
    if (name_it != json.end() && name_it->is_string()) {
      return name_it->get<std::string>();
    }
    return fallback_name;
  }

  static std::string loadGraphJsonFromFile(const std::string & graph_file)
  {
    if (graph_file.empty()) {
      return {};
    }

    std::ifstream file(graph_file);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open graph_file: " + graph_file);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    const auto graph_json = buffer.str();
    if (graph_json.empty()) {
      throw std::runtime_error("graph_file is empty: " + graph_file);
    }
    return graph_json;
  }

  const std::string & getGraphJson()
  {
    if (graph_json_by_name_.empty()) {
      throw std::runtime_error("No route graphs loaded.");
    }
    auto it = graph_json_by_name_.find("graph");
    if (it != graph_json_by_name_.end()) {
      return it->second;
    }
    return graph_json_by_name_.begin()->second;
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

  RouteStartCandidate findBfsStartNode(
    const std::string & graph_json,
    const geometry_msgs::msg::Pose & pose)
  {
    Graph graph;
    GraphToIDMap id_to_graph_map;
    GeoJsonGraphFileLoader loader;
    if (!loader.loadGraphFromJson(graph, id_to_graph_map, graph_json)) {
      throw std::runtime_error("Failed to load route graph json");
    }

    return findNearestStartNodeWithBfsPlanner(graph, pose, connector_density_);
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
  std::string navigator_action_name_;
  std::string navigator_behavior_tree_;
  std::string graph_dir_;
  std::string path_topic_;
  std::string nav_state_topic_;
  double server_timeout_s_{2.0};
  double nav_state_timeout_s_{2.0};
  double connector_density_{0.05};
  bool use_current_pose_as_start_{true};

  rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr topological_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigator_client_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;
  RoutePathPlanner route_path_planner_;
  std::unordered_map<std::string, std::string> graph_json_by_name_;
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
