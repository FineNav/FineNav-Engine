// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace finenav_route
{

struct Metadata
{
  template<typename T>
  T getValue(const std::string & key, T & default_val) const
  {
    auto it = data.find(key);
    if (it == data.end()) { return default_val; }
    return std::any_cast<T>(it->second);
  }

  template<typename T>
  void setValue(const std::string & key, T & value) { data[key] = value; }

  std::unordered_map<std::string, std::any> data;
};

enum class OperationTrigger { NODE = 0, ON_ENTER = 1, ON_EXIT = 2 };

struct Operation { std::string type; OperationTrigger trigger; Metadata metadata; };
using Operations = std::vector<Operation>;

struct EdgeCost { float cost{0.0f}; bool overridable{true}; };
struct Coordinates { std::string frame_id{"map"}; float x{0.0f}; float y{0.0f}; };

struct Node;
using NodePtr       = Node *;
using Graph         = std::vector<Node>;
using GraphToIDMap  = std::unordered_map<unsigned int, unsigned int>;
using NodeExtents   = std::pair<unsigned int, unsigned int>;

struct DirectionalEdge {
  unsigned int edgeid{0};
  NodePtr start{nullptr};
  NodePtr end{nullptr};
  std::vector<Coordinates> polyline;
  EdgeCost edge_cost;
  Metadata metadata;
  Operations operations;
  float getEdgeLength() const;
};

using EdgePtr       = DirectionalEdge *;
using EdgeVector    = std::vector<DirectionalEdge>;
using EdgePtrVector = std::vector<EdgePtr>;

struct SearchState {
  EdgePtr parent_edge{nullptr};
  float integrated_cost{std::numeric_limits<float>::max()};
  float traversal_cost{std::numeric_limits<float>::max()};
  void reset() {
    integrated_cost = std::numeric_limits<float>::max();
    traversal_cost  = std::numeric_limits<float>::max();
    parent_edge     = nullptr;
  }
};

struct Node {
  unsigned int nodeid{0};
  Coordinates coords;
  EdgeVector neighbors;
  Metadata metadata;
  Operations operations;
  SearchState search_state;

  void addEdge(EdgeCost & cost, NodePtr node, unsigned int edgeid,
               Metadata meta_data = {}, Operations operations_data = {}) {
    neighbors.push_back({edgeid, this, node, {}, cost, meta_data, operations_data});
  }
  void addEdge(EdgeCost & cost, NodePtr node, unsigned int edgeid,
               std::vector<Coordinates> polyline_data,
               Metadata meta_data = {}, Operations operations_data = {}) {
    neighbors.push_back({edgeid, this, node, std::move(polyline_data),
                         cost, meta_data, operations_data});
  }
};

inline float DirectionalEdge::getEdgeLength() const {
  if (polyline.size() >= 2) {
    float length = 0.0f;
    for (size_t i = 1; i < polyline.size(); ++i) {
      length += hypotf(polyline[i].x - polyline[i-1].x, polyline[i].y - polyline[i-1].y);
    }
    return length;
  }
  return hypotf(end->coords.x - start->coords.x, end->coords.y - start->coords.y);
}

struct Route { NodePtr start_node{nullptr}; EdgePtrVector edges; float route_cost{0.0f}; };

struct RouteRequest {
  unsigned int start_nodeid{0};
  unsigned int goal_nodeid{0};
  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
  bool use_poses{true};
};

struct ReroutingState { EdgePtr curr_edge{nullptr}; Coordinates closest_pt_on_edge; };

enum class EdgeType { NONE = 0, START = 1, END = 2 };

using NodeElement = std::pair<float, NodePtr>;

struct NodeComparator {
  bool operator()(const NodeElement & a, const NodeElement & b) const { return a.first > b.first; }
};

using NodeQueue = std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator>;

}  // namespace finenav_route

