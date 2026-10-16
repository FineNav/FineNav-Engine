// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_example/dummy_follow_planner_params.hpp"

#include "dummy_world_view_interface.hpp"

namespace finenav::example {

class DummyFollowPlanner {
  public:
    using ConfigType = dummy_follow_planner::Params;
    using IWorldView = DummyWorldView;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};

    using Context = finenav::core::PlanningContext<DummyFollowPlanner>;
    using Result = std::optional<finenav::core::OutputTrajectoryT<DummyFollowPlanner>>;

    void configure(const ConfigType& config);

    void reset() {}

    Result plan(const Context& context, const IWorldView& world_view);

  private:
    double linear_speed_{0.5};
    double angular_speed_{1.0};
    double lookahead_distance_{1.0};
};

}  // namespace finenav::example
