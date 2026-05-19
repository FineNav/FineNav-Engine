#pragma once

#include <optional>
#include <limits>
#include <cmath>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"

// Include generated params header for this planner (same style as SimpleTestPlanner)
#include "finenav_planner_astar/astar_params.hpp"

#include "astar_algorithm.hpp"

namespace finenav {

class AStarPlanner {
public:
    AStarPlanner() = default;

    using ConfigType = astar_planner::Params; // Define configuration parameters
    static constexpr IOProfile InputProfile = {
        .pose = false,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };

    class IMapView {
    public:
        virtual ~IMapView() = default;
        virtual bool isWalkable(const astar::Vec2& pos) const = 0;
        virtual double getCost(const astar::Vec2& pos) const = 0;
    };

    using Context = PlanningContext<AStarPlanner>;
    using Result  = std::optional<OutputDataT<AStarPlanner>>;

    void configure(const ConfigType& params) {
        heuristic_ = params.heuristic; // Configure heuristic type
        allow_diagonal_ = params.allow_diagonal;
        diagonal_weight_ = params.diagonal_weight;
    }

    Result plan(const Context& ctx, const IMapView& map_view) {
        astar::Vec2 start(ctx.start_pose.x, ctx.start_pose.y);
        astar::Vec2 goal(ctx.goal_pose.x, ctx.goal_pose.y);

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
            output.trajectory.push_back({point.x, point.y});
        }

        return output;
    }

private:
    astar::Heuristic heuristic_ = astar::Heuristic::Euclidean;
    bool allow_diagonal_ = true;
    double diagonal_weight_ = std::sqrt(2);
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::AStarPlanner, astar_planner)