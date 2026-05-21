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

#ifndef FINENAV_ROUTE__ROUTE_PLANNER_HPP_
#define FINENAV_ROUTE__ROUTE_PLANNER_HPP_

#include <limits>
#include <vector>

#include "finenav_route/route_exceptions.hpp"
#include "finenav_route/types.hpp"

namespace finenav_route
{

class RoutePlanner
{
public:
  struct Config
  {
    int max_iterations{0};
    float distance_weight{1.0f};
  };

  RoutePlanner() = default;
  virtual ~RoutePlanner() = default;

  void configure(const Config & config);

  virtual Route findRoute(
    Graph & graph, unsigned int start_index, unsigned int goal_index,
    const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request = {});

protected:
  inline void resetSearchStates(Graph & graph);

  void findShortestGraphTraversal(
    Graph & graph, const NodePtr start_node, const NodePtr goal_node,
    const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request);

  inline bool getTraversalCost(
    const EdgePtr edge, float & score, const std::vector<unsigned int> & blocked_ids,
    const RouteRequest & route_request);

  inline NodeElement getNextNode();
  inline void addNode(const float cost, const NodePtr node);
  inline EdgeVector & getEdges(const NodePtr node);
  inline void clearQueue();
  inline bool isGoal(const NodePtr node);
  inline bool isStart(const NodePtr node);
  EdgeType classifyEdge(const EdgePtr edge);

  int max_iterations_{std::numeric_limits<int>::max()};
  float distance_weight_{1.0f};
  unsigned int start_id_{0};
  unsigned int goal_id_{0};
  NodeQueue queue_;
};

}  // namespace finenav_route

#endif  // FINENAV_ROUTE__ROUTE_PLANNER_HPP_
