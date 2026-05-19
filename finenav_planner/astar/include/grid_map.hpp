#pragma once
#include <vector>
#include <limits>
#include "vec2.hpp"

namespace astar {

    class GridMap {
    public:
        GridMap(int width, int height, double default_cost = 1.0)
            : width_(width), height_(height), costs_(width * height, default_cost) {}

        void setObstacle(const Vec2& pos, bool is_obstacle) {
            if (isValid(pos)) {
                costs_[pos.y * width_ + pos.x] = is_obstacle ? INF_COST : 1.0;
            }
        }

        void setCost(const Vec2& pos, double cost) {
            if (isValid(pos)) {
                costs_[pos.y * width_ + pos.x] = cost;
            }
        }

        bool isWalkable(const Vec2& pos) const {
            return isValid(pos) && costs_[pos.y * width_ + pos.x] < INF_COST;
        }

        double getCost(const Vec2& pos) const {
            return isValid(pos) ? costs_[pos.y * width_ + pos.x] : INF_COST;
        }

        int getWidth() const  { return width_; }
        int getHeight() const { return height_; }

    private:
        bool isValid(const Vec2& pos) const {
            return pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_;
        }

        int width_, height_;
        std::vector<double> costs_;
        static constexpr double INF_COST = 1e9;
    };

} // namespace astar