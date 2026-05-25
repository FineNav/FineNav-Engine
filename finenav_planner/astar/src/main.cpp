// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// =============================================================
// Standalone demo: AstarPathSearch end-to-end
//
// This file shows how a user connects their map data to the
// planner by implementing AstarPathSearch::IMapView.
//
// The user only needs to implement three world-coordinate
// queries — they never interact with Vec3, grid cells, or the
// A* algorithm internals.
// =============================================================

#include <iostream>
#include <cmath>

#include "AstarPlanner_2d.hpp"   // AstarPathSearch + IMapView
#include "grid_map.hpp"          // simple grid data container (test utility)

// ── Concrete IMapView ─────────────────────────────────────────────────────
//
// Wraps an astar::GridMap (cell-indexed) and converts world-coordinate
// queries to cell indices using the same resolution/origin that the
// planner uses internally.
//
// NOTE: the user does NOT hardcode the cell indices here.  They just
//       answer "is this world point walkable?" using whatever logic
//       suits their map representation.

class GridMapView : public finenav::AstarPathSearch::IMapView {
public:
    GridMapView(const astar::GridMap& grid, double resolution,
                double origin_x = 0.0, double origin_y = 0.0)
        : grid_(grid),
          res_(resolution),
          ox_(origin_x), oy_(origin_y)
    {}

    bool isWalkable(const finenav::Position3D& p) const override
    {
        int cx = static_cast<int>(std::floor((p.x() - ox_) / res_));
        int cy = static_cast<int>(std::floor((p.y() - oy_) / res_));
        return grid_.isWalkableCell(cx, cy);
    }

    double getCost(const finenav::Position3D& p) const override
    {
        int cx = static_cast<int>(std::floor((p.x() - ox_) / res_));
        int cy = static_cast<int>(std::floor((p.y() - oy_) / res_));
        return grid_.getCostCell(cx, cy);
    }

private:
    const astar::GridMap& grid_;
    double res_, ox_, oy_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main()
{
    constexpr double RESOLUTION = 1.0;   // 1 m per cell (easy integer coords)
    constexpr int    W = 10, H = 10;

    // 1. Build the map data (cell-indexed)
    astar::GridMap grid(W, H);

    // L-shaped obstacle: horizontal wall row 3 cols 2-7
    for (int x = 2; x <= 7; ++x)
        grid.setObstacle(x, 3, true);
    // vertical wall col 5 rows 3-7
    for (int y = 3; y <= 7; ++y)
        grid.setObstacle(5, y, true);

    // 2. Wrap the grid in a concrete IMapView
    //    (The user never touches Vec3 or grid cell indices here.)
    GridMapView map_view(grid, RESOLUTION);

    // 3. Configure the planner
    finenav::AstarPathSearch planner;

    astar_planner::Params params;    // generated from astar.yaml
    params.resolution     = RESOLUTION;
    params.origin_x       = 0.0;
    params.origin_y       = 0.0;
    params.origin_z       = 0.0;
    params.dimension      = "2D";
    params.heuristic      = "Euclidean";
    params.allow_diagonal = false;
    params.diagonal_weight= 1.414;
    params.goal_tolerance = 0.5;
    params.allow_unknown  = true;
    params.max_iterations = 1000000;
    params.max_planning_time = 5.0;

    planner.configure(params);

    // 4. Build a planning context (world-coordinate start / goal)
    finenav::PlanningContext<finenav::AstarPathSearch> ctx;
    ctx.start_pose.position.x = 1.5;   // centre of cell (1,1)
    ctx.start_pose.position.y = 1.5;
    ctx.goal_pose.position.x  = 8.5;   // centre of cell (8,8)
    ctx.goal_pose.position.y  = 8.5;

    // 5. Plan
    auto result = planner.plan(ctx, map_view);

    if (!result.has_value()) {
        std::cout << "No path found.\n";
        return 1;
    }

    std::cout << "Path found!  Waypoints: " << result->poses.size() << "\n\n";

    // 6. Render the grid with the path
    const auto& poses = result->poses;
    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            // Check if any waypoint falls in this cell
            bool on_path = false;
            for (const auto& pose : poses)
            {
                int px = static_cast<int>(std::floor(pose.position.x / RESOLUTION));
                int py = static_cast<int>(std::floor(pose.position.y / RESOLUTION));
                if (px == x && py == y) { on_path = true; break; }
            }

            if      (x == 1 && y == 1) std::cout << "S ";
            else if (x == 8 && y == 8) std::cout << "G ";
            else if (!grid.isWalkableCell(x, y)) std::cout << "# ";
            else if (on_path)           std::cout << "* ";
            else                        std::cout << ". ";
        }
        std::cout << "\n";
    }

    std::cout << "\nWorld waypoints:\n";
    for (const auto& pose : poses)
        std::cout << "  (" << pose.position.x
                  << ", " << pose.position.y << ")\n";

    return 0;
}

