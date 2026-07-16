// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "imap_view.hpp"        // astar::IMapView
#include "terrain_analysis.hpp"
#include "grid_map.hpp"

/// Thin adapter: wraps TerrainAnalyzer + GridMap<Voxel> behind astar::IMapView.
/// Normalises raw TerrainAnalyzer costs (0–254) to [0.0, 1.0].
class DetourMapView : public astar::IMapView {
public:
    DetourMapView(const finenav::GridMap<Voxel>& map,
                  const TerrainAnalyzer& terrain_analyzer)
        : map_(map), terrain_analyzer_(&terrain_analyzer) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        return terrain_analyzer_->getCost(p, map_) < 254;
    }

    double getCost(const finenav::Position3D& p) const override {
        return static_cast<double>(terrain_analyzer_->getCost(p, map_)) / 254.0;
    }

    double getStepSize() const override {
        return map_.getResolution();
    }

private:
    const finenav::GridMap<Voxel>& map_;
    const TerrainAnalyzer* terrain_analyzer_;
};
