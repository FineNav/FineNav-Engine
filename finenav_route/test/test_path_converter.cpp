// Copyright (c) 2025 Open Navigation LLC
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "finenav_route/path_converter.hpp"

using namespace finenav_route;  // NOLINT

TEST(PathConverterTest, DensifiesRouteToPoseTrajectory)
{
  PathConverter converter;
  converter.configure(PathConverter::Config{});

  Route route;
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 10;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 11;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 10.0;
  test_node3.nodeid = 12;
  test_node3.coords.x = 20.0;
  test_node3.coords.y = 20.0;

  DirectionalEdge test_edge1, test_edge2;
  test_edge1.edgeid = 13;
  test_edge1.start = &test_node1;
  test_edge1.end = &test_node2;
  test_edge2.edgeid = 14;
  test_edge2.start = &test_node2;
  test_edge2.end = &test_node3;

  route.start_node = &test_node1;
  route.route_cost = 50.0;
  route.edges.push_back(&test_edge1);
  route.edges.push_back(&test_edge2);

  auto traj = converter.densify(route);
  EXPECT_EQ(traj.poses.size(), 567u);
  EXPECT_NEAR(traj.poses[0].position.x, 0.0, 0.01);
  EXPECT_NEAR(traj.poses[0].position.y, 0.0, 0.01);
  EXPECT_NEAR(traj.poses.back().position.x, 20.0, 0.01);
  EXPECT_NEAR(traj.poses.back().position.y, 20.0, 0.01);
}

TEST(PathConverterTest, HandlesSinglePointRoute)
{
  PathConverter converter;
  converter.configure(PathConverter::Config{});

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 10.0;
  test_node.coords.y = 40.0;

  Route route;
  route.start_node = &test_node;

  auto traj = converter.densify(route);
  EXPECT_EQ(traj.poses.size(), 1u);
  EXPECT_NEAR(traj.poses[0].position.x, 10.0, 0.01);
  EXPECT_NEAR(traj.poses[0].position.y, 40.0, 0.01);
}

TEST(PathConverterTest, HandlesReroutingPrefix)
{
  PathConverter converter;
  converter.configure(PathConverter::Config{});

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 1.0;
  test_node.coords.y = 0.0;

  Route route;
  route.start_node = &test_node;

  DirectionalEdge edge;
  edge.end = &test_node;

  ReroutingState info;
  info.closest_pt_on_edge.x = 0.0;
  info.closest_pt_on_edge.y = 0.0;
  info.curr_edge = &edge;

  auto traj = converter.densify(route, info);
  EXPECT_EQ(traj.poses.size(), 21u);
}

TEST(PathConverterTest, InterpolatesEdgesByDensity)
{
  PathConverter converter;
  converter.configure(PathConverter::Config{});

  std::vector<geometry_msgs::msg::Pose> poses;
  converter.interpolateEdge(10.0, 10.0, 20.0, 20.0, poses);

  EXPECT_EQ(poses.size(), 283u);
  for (unsigned int i = 0; i != poses.size() - 1; i++) {
    EXPECT_LT(
      hypotf(
        poses[i].position.x - poses[i + 1].position.x,
        poses[i].position.y - poses[i + 1].position.y), 0.05);
  }
}
