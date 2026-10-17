# finenav_planners

规划组件集中维护在 [FineNav/finenav_planner](https://github.com/FineNav/finenav_planner)。
网站栏目采用复数 `finenav_planners`，实际仓库名为单数 `finenav_planner`。

## 组件目录

以下目录已对照仓库 `main` 分支的 `bab23e5` 版本核对：

| ROS 包 | Planner 注册名 | 用途 |
| --- | --- | --- |
| `finenav_route_planner` | `astar_planner` | 栅格 A* 路径搜索。 |
| `finenav_route_planner` | `bfs_planner` | 使用零启发函数的栅格搜索。 |
| `finenav_route_planner` | `route_planner` | 基于拓扑路网的路径规划。 |
| `finenav_mppi_SE2` | `finenav_mppi_SE2` | 面向地面机器人的 MPPI 局部规划。 |
| `finenav_rotate_heading_aligner` | `finenav_rotate_shim` | 航向对齐组件。 |

具体接口和参数见对应目录的 README：
[路径规划](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_route_planner)、
[MPPI](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_mppi_SE2)、
[航向对齐](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_rotate_heading_aligner)。

## Engine 接入状态

以下组件需完成接入后组装为导航应用。
核对版本尚未为新的包名发现 API 声明 `finenav_register_algo_plugin()` 资源。
使用前需要通过 `finenav_cmake` 发布共享库；读取地图的算法还需配套 WorldView，
并检查上下游的输入、输出是否匹配。

迁移航向对齐组件时，还要核对完成语义：当前 Engine 将 Planner 返回的 `std::nullopt`
视为失败，不表示该阶段成功完成。不能直接把旧 TrackingPolicy 的切换流程套进新 pipeline。

## 示例 Planner

Getting Started 使用的是 `finenav_example` 自带的两个示例：

| Planner | 输入 → 输出 | 主要参数 |
| --- | --- | --- |
| `dummy_line_planner` | 机器人位姿 + 目标 → 直线位姿路径 | `path_resolution`：采样间距，默认 0.2 m。 |
| `dummy_follow_planner` | 机器人状态 + 路径 → 速度指令 | `linear_speed` 默认 0.5 m/s；`angular_speed` 增益默认 1.0；`lookahead_distance` 默认 1.0 m。 |

Dummy 跟踪器用于演示跟踪接入，没有终点朝向对齐或到点减速。
对应教程：[选择 Map 和 Planner](../choose-algorithms.md)、[组装控制 Pipeline](../cpp-application.md)。

需要扩展算法时，继续阅读[自定义 Planner](../custom-planner.md)。
