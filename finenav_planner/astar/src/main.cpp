#include <iostream>
#include "astar_algorithm.hpp"
//#include "AstarPlanner_2d.hpp"

int main() {
    // 创建10x10的小地图
    astar::GridMap map(10, 10);

    // 添加一个简单的L形障碍物
    // 水平墙 (第3行，第2-7列)
    for (int x = 2; x <= 7; x++) {
        map.setObstacle(astar::Vec2{x, 3}, true);
    }

    // 垂直墙 (第5列，第3-7行)
    for (int y = 3; y <= 7; y++) {
        map.setObstacle(astar::Vec2{5, y}, true);
    }

    // 起点和终点
    astar::Vec2 start{1, 1};
    astar::Vec2 goal{8, 8};

    // A*求解器
    astar::AStar<decltype(map)> planner(map, astar::Heuristic::Manhattan);
    planner.setAllowDiagonal(false);  // 只用上下左右

    // 找路径
    auto path = planner.findPath(start, goal);

    if (path.has_value()) {
        std::cout << "找到路径! 节点数: " << path->size() << "\n\n";

        // 打印地图
        for (int y = 0; y < map.getHeight(); y++) {
            for (int x = 0; x < map.getWidth(); x++) {
                astar::Vec2 pos{x, y};

                // 检查是否在路径上
                bool on_path = false;
                for (const auto& p : *path) {
                    if (p == pos) {
                        on_path = true;
                        break;
                    }
                }

                if (pos == start) {
                    std::cout << "S ";
                } else if (pos == goal) {
                    std::cout << "G ";
                } else if (!map.isWalkable(pos)) {
                    std::cout << "# ";
                } else if (on_path) {
                    std::cout << "* ";
                } else {
                    std::cout << ". ";
                }
            }
            std::cout << "\n";
        }

        std::cout << "\n路径坐标: ";
        for (const auto& p : *path) {
            std::cout << "(" << p.x << "," << p.y << ") ";
        }
        std::cout << "\n";

    } else {
        std::cout << "没找到路径!\n";
    }

    return 0;
}