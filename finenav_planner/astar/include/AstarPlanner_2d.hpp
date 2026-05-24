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

    void configure(const ConfigType& params);

    Result plan(const Context& ctx, const IMapView& map_view);

private:
    astar::Heuristic heuristic_ = astar::Heuristic::Euclidean;
    bool allow_diagonal_ = true;
    double diagonal_weight_ = std::sqrt(2);
    double goal_tolerance_ = 0.25;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::AStarPlanner, astar_planner)