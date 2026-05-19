// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <limits>
#include "vec2.hpp"

namespace astar {

    // 2D node type used by A* (templated cost type)
    template<typename T = double>
    struct Node2D {
        Vec2 pos;
        T g = std::numeric_limits<T>::max();
        T h = 0;
        T f = std::numeric_limits<T>::max();
        Vec2 parent{-1, -1};

        Node2D() = default;
        explicit Node2D(const Vec2& p) : pos(p) {}

        void updateF() {
            f = g + h;
        }
    };

    // 优先队列比较器
    template<typename T>
    struct Node2DCompare {
        bool operator()(const Node2D<T>& a, const Node2D<T>& b) const {
            return a.f > b.f;
        }
    };

} // namespace astar

