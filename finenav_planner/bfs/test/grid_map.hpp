// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>

namespace bfs {

class GridMap {
public:
    GridMap(int width, int height)
        : width_(width), height_(height),
          occupied_(width * height, false)
    {}

    void setObstacle(int x, int y, bool obstacle) {
        if (isValid(x, y)) {
            occupied_[idx(x, y)] = obstacle;
        }
    }

    bool isWalkableCell(int x, int y) const {
        return isValid(x, y) && !occupied_[idx(x, y)];
    }

private:
    bool isValid(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    int idx(int x, int y) const { return y * width_ + x; }

    int width_;
    int height_;
    std::vector<bool> occupied_;
};

} // namespace bfs
