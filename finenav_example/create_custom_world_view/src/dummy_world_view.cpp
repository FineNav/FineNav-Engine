// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_world_view.hpp"

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {

void SimpleWorldView::prepare(const finenav::core::PlanningContext<DummyPlanner>& ctx) {
    // Heavy map-derived derivation belongs here, under the map read lock, once
    // per plan().  It is gated on ctx.maps_changed: the framework compares the
    // backing maps' revision counters and sets the flag only when one actually
    // changed (or on the first prepare), so a plan over an unchanged map reuses
    // the cache and pays just one integer compare.  A real view replaces the
    // lattice below with its inflation / terrain / cost-field product.
    if (ctx.maps_changed || !cache_valid_) {
        rebuildWalkableCache();
        cache_valid_ = true;
        ++derive_count_;
    }

    ++prepare_count_;
    if (prepare_log_period_ <= 0 || prepare_count_ % prepare_log_period_ != 0) {
        return;
    }
    const auto& p = ctx.robot_state.pose.position;
    RCLCPP_INFO(logger_, "SimpleWorldView::prepare() #%ld — robot at (%.2f, %.2f), cache %s "
        "(rebuild #%ld).",
        static_cast<long>(prepare_count_), p.x, p.y,
        ctx.maps_changed ? "rebuilt" : "reused", static_cast<long>(derive_count_));
}

void SimpleWorldView::rebuildWalkableCache() {
    const auto bounds = map_.getWindowBounds();
    const auto extent = sizes(bounds);  // ADL: finenav::common::sizes
    walkable_cache_.assign(static_cast<size_t>(kSampleGrid) * kSampleGrid, 0);
    for (int iy = 0; iy < kSampleGrid; ++iy) {
        for (int ix = 0; ix < kSampleGrid; ++ix) {
            const finenav::common::Position3D p{
                bounds.min.x + extent.x * (ix + 0.5) / kSampleGrid,
                bounds.min.y + extent.y * (iy + 0.5) / kSampleGrid,
                bounds.min.z};
            walkable_cache_[static_cast<size_t>(iy) * kSampleGrid + ix] =
                (!map_.isInside(p) || !map_.isOccupied(p)) ? 1 : 0;
        }
    }
}

}  // namespace finenav::example

// One-line registration.  The macro records only TYPES (planner + maps); the
// framework resolves those types back to their registered algorithm names
// ("dummy_planner" / "dummy_map") at assembly time — so the WorldView author
// never writes a planner/map name by hand, and plugin load order is irrelevant.
//
// Arguments: (view type, param namespace, planner type, maps...).  As with
// FINENAV_REGISTER_PLANNER / _MAP, that one namespace token is also the
// registration name — so it must match the top-level key of
// dummy_world_view.yaml.
FINENAV_REGISTER_WORLDVIEW(
    finenav::example::SimpleWorldView,
    dummy_world_view,
    finenav::example::DummyPlanner,
    finenav::example::DummyMap)
