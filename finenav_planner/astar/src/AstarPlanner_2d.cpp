// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 2026/5/20.
//
#include "AstarPlanner_2d.hpp"

#include <limits>
#include <cmath>
#include <string>
#include <type_traits>
#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// void AStarPlanner::configure(const ConfigType& params) {
//     // params.heuristic may be a string (generated param) or an enum
//     if constexpr (std::is_same_v<decltype(params.heuristic), std::string>) {
//         const auto &h = params.heuristic;
//         if (h == "Euclidean") heuristic_ = astar::Heuristic::Euclidean;
//         else if (h == "Manhattan") heuristic_ = astar::Heuristic::Manhattan;
//         else if (h == "Chebyshev") heuristic_ = astar::Heuristic::Chebyshev;
//         else heuristic_ = astar::Heuristic::Euclidean;
//     } else {
//         heuristic_ = params.heuristic; // Configure heuristic type
//     }
//     allow_diagonal_ = params.allow_diagonal;
//     diagonal_weight_ = params.diagonal_weight;
// }

void AStarPlanner::configure(const ConfigType& params)
{
    const auto &h = params.heuristic;

    if (h == "Euclidean")
    {
        heuristic_ = astar::Heuristic::Euclidean;
    }
    else if (h == "Manhattan")
    {
        heuristic_ = astar::Heuristic::Manhattan;
    }
    else if (h == "Chebyshev")
    {
        heuristic_ = astar::Heuristic::Chebyshev;
    }
    else
    {
        heuristic_ = astar::Heuristic::Euclidean;
    }

    allow_diagonal_ = params.allow_diagonal;
    diagonal_weight_ = params.diagonal_weight;
    goal_tolerance_ = params.goal_tolerance;
}

AStarPlanner::Result AStarPlanner::plan(const Context& ctx, const IMapView& map_view) {
    astar::Vec2 start(static_cast<int>(std::lround(ctx.start_pose.position.x)),
                      static_cast<int>(std::lround(ctx.start_pose.position.y)));
    astar::Vec2 goal(static_cast<int>(std::lround(ctx.goal_pose.position.x)),
                     static_cast<int>(std::lround(ctx.goal_pose.position.y)));

    auto map_adapter = [&map_view](const astar::Vec2& pos) {
        return map_view.isWalkable(pos) ? map_view.getCost(pos) : std::numeric_limits<double>::infinity();
    };

    astar::AStar<decltype(map_adapter)> planner(map_adapter, heuristic_);
    planner.setAllowDiagonal(allow_diagonal_);
    planner.setDiagonalWeight(diagonal_weight_);

    auto path = planner.findPath(start, goal);
    if (!path) {
        return std::nullopt; // No path found
    }

    OutputDataT<AStarPlanner> output;
    for (const auto& point : *path) {
        geometry_msgs::msg::Pose p;
        p.position.x = static_cast<double>(point.x);
        p.position.y = static_cast<double>(point.y);
        p.position.z = 0.0;
        // default orientation (0,0,0,1)
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }

    return output;
}

} // namespace finenav
