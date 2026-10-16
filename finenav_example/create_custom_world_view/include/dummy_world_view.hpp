// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plugin_world_view.hpp"

#include "dummy_map.hpp"
#include "dummy_planner.hpp"

#include "finenav_example/dummy_world_view_params.hpp"

namespace finenav::example {

/**
 * @brief SimpleWorldView — the bridge between DummyMap and DummyPlanner::IWorldView.
 *
 * This is the ONE artifact that knows both concrete types.  It is written once
 * by the navigation-logic author and registered via FINENAV_REGISTER_WORLDVIEW;
 * the end user then assembles the ControlLayer by NAME without ever seeing
 * either DummyMap or DummyPlanner.
 *
 * It is also the reference shape of the finenav::core::WorldView concept:
 *
 *   MANDATORY                        OPTIONAL (declare to receive)
 *   -----------------------------    -----------------------------------
 *   using ConfigType = <gpl Params>  setLogger(rclcpp::Logger)
 *   configure(const ConfigType&)     setVisualizer(viz::FineNavVisualizer&)
 *   prepare(const PlanningContext&)
 *
 * The constructor convention
 * (ViewT(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...)) is what the
 * framework uses to auto-build the view from the maps it resolved at assembly
 * time — the author only declares the map types in the registration macro.
 */
class SimpleWorldView : public DummyPlanner::IWorldView {
public:
    /// Parameter channel — the YAML namespace is the registration name.
    using ConfigType = dummy_world_view::Params;

    explicit SimpleWorldView(rclcpp::Node::SharedPtr node, const DummyMap& map)
        : map_(map), logger_(node->get_logger().get_child("dummy_world_view")) {}

    // ── MANDATORY: parameter channel ──────────────────────────────────────
    /// Called once at startup and again on every `ros2 param set`.
    /// Params carries an rclcpp::Time stamp, so we cache the field we use
    /// rather than keeping a whole Params copy (same as DummyMap / DummyPlanner).
    void configure(const ConfigType& cfg) { prepare_log_period_ = cfg.prepare_log_period; }

    // ── MANDATORY: per-plan derivation hook ───────────────────────────────
    /**
     * @brief Called ONCE PER plan(), inside the map read lock, before plan().
     *
     * This is the ONLY place heavy derivation belongs — inflation, terrain
     * analysis, cost-field building, anything that reads map_ or needs the
     * robot pose / goal.  The query methods below (isWalkable) must stay cheap
     * and read only what prepare() left behind.
     *
     * The heavy product here is a walkability lattice sampled from map_.  It is
     * gated on ctx.maps_changed: the framework owns the backing map's revision
     * counter and sets that flag whenever a map changed since the previous
     * prepare() (and on the first prepare).  When it is false the lattice is
     * reused as-is, so prepare() is just the cheap robot-side bookkeeping —
     * exactly the shape terrain/inflation/cost-field caches should follow.
     */
    void prepare(const finenav::core::PlanningContext<DummyPlanner>& ctx);

    /// How many times the walkability lattice has been rebuilt — exposed so a
    /// caller can confirm the maps_changed gate is skipping recomputation.
    int64_t deriveRebuilds() const { return derive_count_; }

    // ── OPTIONAL injections ───────────────────────────────────────────────
    // Declare these to receive the framework logger / visualizer automatically.
    // The framework detects them with if constexpr — omit them and nothing breaks.
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    // ── Query path: must be cheap ─────────────────────────────────────────
    bool isWalkable(const finenav::common::Position3D& p) const override {
        return !map_.isOccupied(p);
    }

private:
    /// Sample map_ on a fixed lattice over its window, proving the pattern a
    /// real view uses for its inflation / cost-field cache.
    void rebuildWalkableCache();

    static constexpr int kSampleGrid = 16;

    const DummyMap&     map_;
    std::vector<char>   walkable_cache_;  ///< kSampleGrid * kSampleGrid cells
    bool                cache_valid_{false};
    int64_t             derive_count_{0};
    int64_t             prepare_log_period_{100};
    rclcpp::Logger      logger_;
    int64_t             prepare_count_{0};
};

}  // namespace finenav::example
