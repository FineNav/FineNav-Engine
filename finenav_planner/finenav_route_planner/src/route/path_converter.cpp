// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>

#include "route/path_converter.hpp"
#include "route/route_exceptions.hpp"

namespace finenav_route
{

namespace { constexpr double kDuplicatePoseTolerance = 1e-6; }

static geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(angle / 2.0);
  q.w = std::cos(angle / 2.0);
  return q;
}

void PathConverter::configure(const Config & config) { density_ = config.density; }

PathConverter::Trajectory PathConverter::densify(
  const Route & route, const ReroutingState & rerouting_info) const
{
  Trajectory trajectory;
  if (!route.start_node) return trajectory;

  if (rerouting_info.curr_edge) {
    const Coordinates & start = rerouting_info.closest_pt_on_edge;
    if (rerouting_info.curr_edge->polyline.size() < 2)
      throw NoValidGraph("Route edge does not contain a valid polyline");
    interpolateEdge(start.x, start.y,
      rerouting_info.curr_edge->polyline.front().x,
      rerouting_info.curr_edge->polyline.front().y,
      trajectory.poses);
    appendEdgePolyline(*rerouting_info.curr_edge, trajectory.poses);
  }

  if (!route.edges.empty()) {
    for (unsigned int i = 0; i < route.edges.size() - 1; i++)
      appendEdgePolyline(*route.edges[i], trajectory.poses);
  }

  if (route.edges.empty())
    trajectory.poses.push_back(makePose(route.start_node->coords.x, route.start_node->coords.y));
  else
    appendEdgePolyline(*route.edges.back(), trajectory.poses);

  assignOrientations(trajectory.poses);
  return trajectory;
}

std::vector<geometry_msgs::msg::Pose> PathConverter::extractTopologicalPoses(const Route & route) const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (!route.start_node) return poses;
  poses.push_back(makePose(route.start_node->coords.x, route.start_node->coords.y));
  for (const auto & edge : route.edges)
    poses.push_back(makePose(edge->end->coords.x, edge->end->coords.y));
  assignOrientations(poses);
  return poses;
}

void PathConverter::interpolateEdge(
  float x0, float y0, float x1, float y1,
  std::vector<geometry_msgs::msg::Pose> & poses) const
{
  const float mag = hypotf(x1 - x0, y1 - y0);
  if (mag < 1e-6f) { appendPoseIfNew(makePose(x0, y0), poses); return; }
  const unsigned int num_pts = static_cast<unsigned int>(std::ceil(mag / density_));
  const float interpolated_dist = mag / static_cast<float>(num_pts);
  const float ux = (x1 - x0) / mag, uy = (y1 - y0) / mag;
  float x = x0, y = y0;
  appendPoseIfNew(makePose(x, y), poses);
  for (unsigned int i = 0; i < num_pts - 1; ++i) {
    x += ux * interpolated_dist; y += uy * interpolated_dist;
    appendPoseIfNew(makePose(x, y), poses);
  }
}

void PathConverter::appendEdgePolyline(
  const DirectionalEdge & edge, std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (edge.polyline.size() < 2) throw NoValidGraph("Route edge does not contain a valid polyline");
  for (size_t i = 1; i < edge.polyline.size(); ++i)
    interpolateEdge(edge.polyline[i-1].x, edge.polyline[i-1].y,
                    edge.polyline[i].x,   edge.polyline[i].y, poses);
  appendPoseIfNew(makePose(edge.polyline.back().x, edge.polyline.back().y), poses);
}

geometry_msgs::msg::Pose PathConverter::makePose(float x, float y) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x; pose.position.y = y; pose.orientation.w = 1.0;
  return pose;
}

void PathConverter::appendPoseIfNew(
  const geometry_msgs::msg::Pose & pose, std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (!poses.empty()) {
    const double dx = pose.position.x - poses.back().position.x;
    const double dy = pose.position.y - poses.back().position.y;
    if (std::hypot(dx, dy) <= kDuplicatePoseTolerance) return;
  }
  poses.push_back(pose);
}

void PathConverter::assignOrientations(std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (poses.size() < 2) return;
  for (size_t i = 0; i < poses.size() - 1; ++i) {
    const double dx = poses[i+1].position.x - poses[i].position.x;
    const double dy = poses[i+1].position.y - poses[i].position.y;
    poses[i].orientation = orientationAroundZAxis(std::atan2(dy, dx));
  }
  poses.back().orientation = poses[poses.size() - 2].orientation;
}

}  // namespace finenav_route

