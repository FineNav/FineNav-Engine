// Copyright (c) 2025 Open Navigation LLC
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef FINENAV_ROUTE__UTILS_HPP_
#define FINENAV_ROUTE__UTILS_HPP_

#include <cmath>

#include "finenav_route/types.hpp"

namespace finenav_route
{
namespace utils
{

inline geometry_msgs::msg::PoseStamped toMsg(const float x, const float y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

inline float normalizedDot(
  const float v1x, const float v1y,
  const float v2x, const float v2y)
{
  const float mag1 = std::hypotf(v1x, v1y);
  const float mag2 = std::hypotf(v2x, v2y);
  if (mag1 < 1e-6f || mag2 < 1e-6f) {
    return 0.0f;
  }
  return (v1x / mag1) * (v2x / mag2) + (v1y / mag1) * (v2y / mag2);
}

inline Coordinates findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  const Coordinates & start, const Coordinates & end)
{
  Coordinates pt;
  const float vx = end.x - start.x;
  const float vy = end.y - start.y;
  const float ux = start.x - pose.pose.position.x;
  const float uy = start.y - pose.pose.position.y;
  const float uv = vx * ux + vy * uy;
  const float vv = vx * vx + vy * vy;

  if (vv < 1e-6f) {
    return start;
  }

  const float t = -uv / vv;
  if (t > 0.0f && t < 1.0f) {
    pt.x = (1.0f - t) * start.x + t * end.x;
    pt.y = (1.0f - t) * start.y + t * end.y;
  } else if (t <= 0.0f) {
    pt = start;
  } else {
    pt = end;
  }

  return pt;
}

inline float distance(const Coordinates & coords, const geometry_msgs::msg::PoseStamped & pose)
{
  return hypotf(coords.x - pose.pose.position.x, coords.y - pose.pose.position.y);
}

}  // namespace utils
}  // namespace finenav_route

#endif  // FINENAV_ROUTE__UTILS_HPP_
