#pragma once
#include <cmath>
#include "vec2.hpp"

namespace astar {

    enum class Heuristic {
        Euclidean,
        Manhattan,
        Chebyshev
    };

    inline double calculateHeuristic(
        Heuristic type,
        const Vec2& from,
        const Vec2& to) {

        double dx = std::abs(from.x - to.x);
        double dy = std::abs(from.y - to.y);

        switch (type) {
        case Heuristic::Euclidean:
            return std::sqrt(dx*dx + dy*dy);
        case Heuristic::Manhattan:
            return dx + dy;
        case Heuristic::Chebyshev:
            return std::max(dx, dy);
        default:
            return dx + dy;
        }
    }

} // namespace astar