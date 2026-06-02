// Copyright (c) 2025 Open Navigation LLC
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "finenav_route/route_exceptions.hpp"
#include "finenav_route/route_planner.hpp"

using namespace finenav_route;  // NOLINT

inline Graph create4x4Graph()
{
  EdgeCost default_cost;
  Graph graph;
  graph.resize(16);
  unsigned int idx = 1;
  for (unsigned int j = 0; j != 4; j++) {
    for (unsigned int i = 0; i != 4; i++) {
      Node & node = graph[idx - 1];
      node.coords.x = static_cast<float>(i);
      node.coords.y = static_cast<float>(j);
      node.nodeid = idx;
      idx++;
    }
  }

  graph[0].addEdge(default_cost, &graph[1], idx++);
  graph[1].addEdge(default_cost, &graph[0], idx++);
  graph[0].addEdge(default_cost, &graph[4], idx++);
  graph[4].addEdge(default_cost, &graph[0], idx++);

  graph[1].addEdge(default_cost, &graph[2], idx++);
  graph[2].addEdge(default_cost, &graph[1], idx++);
  graph[1].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[1], idx++);

  graph[2].addEdge(default_cost, &graph[3], idx++);
  graph[3].addEdge(default_cost, &graph[2], idx++);
  graph[2].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[2], idx++);

  graph[7].addEdge(default_cost, &graph[3], idx++);
  graph[3].addEdge(default_cost, &graph[7], idx++);

  graph[4].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[4], idx++);
  graph[4].addEdge(default_cost, &graph[8], idx++);
  graph[8].addEdge(default_cost, &graph[4], idx++);

  graph[5].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[5], idx++);
  graph[5].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[5], idx++);

  graph[6].addEdge(default_cost, &graph[7], idx++);
  graph[7].addEdge(default_cost, &graph[6], idx++);
  graph[6].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[6], idx++);

  graph[7].addEdge(default_cost, &graph[11], idx++);
  graph[11].addEdge(default_cost, &graph[7], idx++);

  graph[8].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[8], idx++);
  graph[8].addEdge(default_cost, &graph[12], idx++);
  graph[12].addEdge(default_cost, &graph[8], idx++);

  graph[9].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[9], idx++);
  graph[9].addEdge(default_cost, &graph[13], idx++);
  graph[13].addEdge(default_cost, &graph[9], idx++);

  graph[10].addEdge(default_cost, &graph[11], idx++);
  graph[11].addEdge(default_cost, &graph[10], idx++);
  graph[10].addEdge(default_cost, &graph[14], idx++);
  graph[14].addEdge(default_cost, &graph[10], idx++);

  graph[11].addEdge(default_cost, &graph[15], idx++);

  graph[12].addEdge(default_cost, &graph[13], idx++);
  graph[13].addEdge(default_cost, &graph[12], idx++);

  graph[13].addEdge(default_cost, &graph[14], idx++);
  graph[14].addEdge(default_cost, &graph[13], idx++);

  graph[14].addEdge(default_cost, &graph[15], idx++);
  return graph;
}

TEST(RoutePlannerTest, FindsDistanceCostRoutes)
{
  RoutePlanner planner;
  planner.configure(RoutePlanner::Config{});
  std::vector<unsigned int> blocked_ids;
  RouteRequest route_request;

  Graph graph = create4x4Graph();

  Route route = planner.findRoute(graph, 0u, 15u, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 6.0, 0.001);
  EXPECT_EQ(route.edges.size(), 6u);

  EXPECT_THROW(
    planner.findRoute(graph, 15u, 0u, blocked_ids, route_request),
    NoValidRouteCouldBeFound);

  route = planner.findRoute(graph, 0u, 12u, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 3.0, 0.001);
  EXPECT_EQ(route.edges.size(), 3u);

  unsigned int key_edgeid = 19u;
  blocked_ids.push_back(key_edgeid);
  route = planner.findRoute(graph, 0u, 12u, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 5.0, 0.001);
  EXPECT_EQ(route.edges.size(), 5u);
  for (auto & edge : route.edges) {
    EXPECT_NE(edge->edgeid, key_edgeid);
  }

  blocked_ids.clear();
  graph[0].neighbors[1].edge_cost.cost = 1e6;
  graph[0].neighbors[1].edge_cost.overridable = false;
  route = planner.findRoute(graph, 0u, 12u, blocked_ids, route_request);
  EXPECT_NEAR(route.route_cost, 5.0, 0.001);
  EXPECT_EQ(route.edges.size(), 5u);
}

TEST(RoutePlannerTest, ReportsInvalidInputs)
{
  RoutePlanner planner;
  planner.configure(RoutePlanner::Config{.max_iterations = 5});
  RouteRequest route_request;
  std::vector<unsigned int> blocked_ids;
  Graph graph;

  EXPECT_THROW(
    planner.findRoute(graph, 0u, 15u, blocked_ids, route_request),
    NoValidGraph);

  graph = create4x4Graph();
  EXPECT_THROW(
    planner.findRoute(graph, 0u, 15u, blocked_ids, route_request),
    TimedOut);

  planner.configure(RoutePlanner::Config{});
  graph[0].neighbors[1].edge_cost.overridable = false;
  graph[0].neighbors[1].edge_cost.cost = 0.0;
  EXPECT_THROW(
    planner.findRoute(graph, 0u, 15u, blocked_ids, route_request),
    NoValidGraph);
}
