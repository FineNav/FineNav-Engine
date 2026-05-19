#pragma once
#include <cstddef>
#include <cmath>

namespace astar {

    struct Vec2 {
        int x, y;

        Vec2(int x = 0, int y = 0) : x(x), y(y) {}

        bool operator==(const Vec2& other) const {
            return x == other.x && y == other.y;
        }

        Vec2 operator+(const Vec2& other) const {
            return Vec2{x + other.x, y + other.y};
        }

        double distance(const Vec2& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            return std::sqrt(dx*dx + dy*dy);
        }

        int manhattan(const Vec2& other) const {
            return std::abs(x - other.x) + std::abs(y - other.y);
        }
    };

    struct Vec2Hash {
        std::size_t operator()(const Vec2& v) const noexcept {
            return (static_cast<size_t>(v.x) << 32) ^ static_cast<size_t>(v.y);
        }
    };

} // namespace astar
