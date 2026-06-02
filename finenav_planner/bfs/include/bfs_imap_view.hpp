// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/concepts.hpp"

namespace bfs {

// =============================================================
// IMapView
//
// User-facing world-coordinate map interface for BfsPathSearch.
// BFS treats all walkable cells as equal cost, so only
// isWalkable() and getStepSize() are used.
// =============================================================

class IMapView {
public:
    virtual ~IMapView() = default;

    virtual bool isWalkable(const finenav::Position3D& p) const = 0;
    virtual double getStepSize() const = 0;
};

} // namespace bfs
