// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <limits>

namespace astar {

// =============================================================
// Key  —  two-component priority key for the D* Lite open list
//
// Key comparison is lexicographic on (k1, k2).
// Smaller key  =>  higher priority.
//
// Formula (Koenig & Likhachev, AAAI 2002):
//   k1 = min(g(s), rhs(s)) + h(start, s) + km
//   k2 = min(g(s), rhs(s))
// =============================================================

inline constexpr double kInf = std::numeric_limits<double>::infinity();

// ── epsilon for floating-point key comparisons ─────────────────
inline constexpr double kKeyEpsilon = 1e-9;

struct Key {
    double k1 = kInf;
    double k2 = kInf;

    constexpr Key() = default;
    constexpr Key(double a, double b) noexcept : k1(a), k2(b) {}

    /// Lexicographic comparison with epsilon tolerance.
    /// Two keys are equal when |k1_a - k1_b| < epsilon AND
    /// |k2_a - k2_b| < epsilon.
    bool operator==(const Key& o) const noexcept {
        return std::abs(k1 - o.k1) < kKeyEpsilon &&
               std::abs(k2 - o.k2) < kKeyEpsilon;
    }

    bool operator<(const Key& o) const noexcept {
        if (std::abs(k1 - o.k1) >= kKeyEpsilon)
            return k1 < o.k1;
        if (std::abs(k2 - o.k2) >= kKeyEpsilon)
            return k2 < o.k2;
        return false;  // effectively equal
    }

    bool operator>(const Key& o)  const noexcept { return o < *this; }
    bool operator<=(const Key& o) const noexcept { return !(*this > o); }
    bool operator>=(const Key& o) const noexcept { return !(*this < o); }
    bool operator!=(const Key& o) const noexcept { return !(*this == o); }
};

}  // namespace astar
