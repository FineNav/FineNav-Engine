// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_example/dummy_line_planner_params.hpp"

#include "dummy_world_view_interface.hpp"

namespace finenav::example {

class DummyLinePlanner {
  public:
    using ConfigType = dummy_line_planner::Params;
    using IWorldView = DummyWorldView;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    using Context = finenav::core::PlanningContext<DummyLinePlanner>;
    using Result = std::optional<finenav::core::OutputTrajectoryT<DummyLinePlanner>>;

    void configure(const ConfigType& config) { path_resolution_ = config.path_resolution; }

    void reset() {}

    Result plan(const Context& context, const IWorldView& world_view);

  private:
    double path_resolution_{0.2};
};

}  // namespace finenav::example
