// Copyright (c) 2025 Open Navigation LLC
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include "finenav_route/path_converter.hpp"

namespace finenav_route
{

static geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(angle / 2.0);
  q.w = std::cos(angle / 2.0);
  return q;
}

void PathConverter::configure(const Config & config)
{
  density_ = config.density;
  smoothing_radius_ = config.smoothing_radius;
  smooth_corners_ = config.smooth_corners;
}

PathConverter::Trajectory PathConverter::densify(
  const Route & route,
  const ReroutingState & rerouting_info) const
{
  Trajectory trajectory;

  if (!route.start_node) {
    return trajectory;
  }

  if (rerouting_info.curr_edge) {
    const Coordinates & start = rerouting_info.closest_pt_on_edge;
    if (!rerouting_info.curr_edge->polyline.empty()) {
      interpolateEdge(
        start.x, start.y,
        rerouting_info.curr_edge->polyline.front().x,
        rerouting_info.curr_edge->polyline.front().y,
        trajectory.poses);
      appendEdgePolyline(*rerouting_info.curr_edge, trajectory.poses);
    } else {
      const Coordinates & end = rerouting_info.curr_edge->end->coords;
      interpolateEdge(start.x, start.y, end.x, end.y, trajectory.poses);
    }
  }

  Coordinates start;
  Coordinates end;

  if (!route.edges.empty()) {
    start = route.edges[0]->start->coords;

    for (unsigned int i = 0; i < route.edges.size() - 1; i++) {
      const EdgePtr edge = route.edges[i];
      const EdgePtr & next_edge = route.edges[i + 1];
      end = edge->end->coords;

      CornerArc corner_arc(start, end, next_edge->end->coords, smoothing_radius_);
      if (corner_arc.isCornerValid() && smooth_corners_) {
        end = corner_arc.getCornerStart();
        interpolateEdge(start.x, start.y, end.x, end.y, trajectory.poses);
        corner_arc.interpolateArc(density_ / smoothing_radius_, trajectory.poses);
        start = corner_arc.getCornerEnd();
      } else {
        appendEdgePolyline(*edge, trajectory.poses);
        start = end;
      }
    }
  }

  if (route.edges.empty()) {
    trajectory.poses.push_back(makePose(route.start_node->coords.x, route.start_node->coords.y));
  } else {
    appendEdgePolyline(*route.edges.back(), trajectory.poses);
  }

  assignOrientations(trajectory.poses);
  return trajectory;
}

void PathConverter::interpolateEdge(
  float x0, float y0, float x1, float y1,
  std::vector<geometry_msgs::msg::Pose> & poses) const
{
  const float mag = hypotf(x1 - x0, y1 - y0);
  if (mag < 1e-6f) {
    poses.push_back(makePose(x0, y0));
    return;
  }

  const unsigned int num_pts = static_cast<unsigned int>(std::ceil(mag / density_));
  const float interpolated_dist = mag / static_cast<float>(num_pts);

  const float ux = (x1 - x0) / mag;
  const float uy = (y1 - y0) / mag;

  float x = x0;
  float y = y0;
  poses.push_back(makePose(x, y));

  unsigned int pt_ctr = 0;
  while (pt_ctr < num_pts - 1) {
    x += ux * interpolated_dist;
    y += uy * interpolated_dist;
    pt_ctr++;
    poses.push_back(makePose(x, y));
  }
}

void PathConverter::appendEdgePolyline(
  const DirectionalEdge & edge,
  std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (edge.polyline.size() < 2) {
    interpolateEdge(
      edge.start->coords.x, edge.start->coords.y,
      edge.end->coords.x, edge.end->coords.y, poses);
    poses.push_back(makePose(edge.end->coords.x, edge.end->coords.y));
    return;
  }

  for (size_t i = 1; i < edge.polyline.size(); ++i) {
    const auto & start = edge.polyline[i - 1];
    const auto & end = edge.polyline[i];
    interpolateEdge(start.x, start.y, end.x, end.y, poses);
  }

  const auto & end = edge.polyline.back();
  poses.push_back(makePose(end.x, end.y));
}

geometry_msgs::msg::Pose PathConverter::makePose(float x, float y) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

void PathConverter::assignOrientations(std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (poses.size() < 2) {
    return;
  }

  for (size_t i = 0; i < poses.size() - 1; ++i) {
    const auto & pose = poses[i];
    const auto & next_pose = poses[i + 1];
    const double dx = next_pose.position.x - pose.position.x;
    const double dy = next_pose.position.y - pose.position.y;
    poses[i].orientation = orientationAroundZAxis(std::atan2(dy, dx));
  }

  poses.back().orientation = poses[poses.size() - 2].orientation;
}

}  // namespace finenav_route
