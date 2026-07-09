// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

#include <Eigen/Dense>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/concepts.hpp"
#include "finenav_core/plan/data_packet.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_planner_neupan/neupan_params.hpp"

namespace py = pybind11;

namespace finenav {

class neupan_plan {
public:
    neupan_plan() = default;

    using ConfigType = neupan_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose  = false,
        .vel   = true,
        .accel = false,
        .time  = false
    };

    class IMapView {
    public:
        virtual ~IMapView() = default;
        virtual std::vector<Eigen::Vector3d> getPointCloud() const = 0;
    };

    using Context = PlanningContext<neupan_plan>;
    using Result  = std::optional<OutputDataT<neupan_plan>>;

    void configure(const ConfigType& params);

    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

private:
    double getYawFromPose(const geometry_msgs::msg::Pose& pose) const;

    ConfigType config_{};
    bool configured_{false};
    std::unique_ptr<py::scoped_interpreter> py_guard_;
    py::object planner_;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::neupan_plan, neupan_planner)
