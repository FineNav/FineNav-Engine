# 2. 选择 Map 和 Planner

根据应用的输入、输出选择组件。本例将目标点转换为位姿路径，再生成底盘速度指令。

## 示例组件

| 职责 | 实现 / 注册名 | 用途 |
| --- | --- | --- |
| Map | `DummyMap` / `dummy_map` | 提供最小占据查询，所有位置均被视为没有障碍物。 |
| 路径规划 | `DummyLinePlanner` / `dummy_line_planner` | 机器人当前位姿 + 目标 → 直线位姿路径。 |
| 路径跟踪 | `DummyFollowPlanner` / `dummy_follow_planner` | 机器人状态 + 位姿路径 → 速度指令。 |
| 第一层适配 | `DummyLineWorldView` / `dummy_line_world_view` | 对接 Map 与直线规划器。 |
| 第二层适配 | `DummyFollowWorldView` / `dummy_follow_world_view` | 对接同一 Map 与路径跟踪器。 |

以上组件全部由 ROS 包 `finenav_example` 提供，不依赖独立地图、规划仓库中的算法。

## 组件配置

```cpp
engine->loadAlgoPlugins({"finenav_example"});
engine->createControlLayer(
    "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
engine->createControlLayer(
    "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
```

`loadAlgoPlugins()` 接收 **ROS 包名**，无需加 `/lib`；插件包需要通过
`finenav_register_algo_plugin()` 声明可发现的共享库。
本例 Planner 需要读取地图，因此 `createControlLayer()` 选择的是 **WorldView 注册名**。
该注册已经绑定 Map 和 Planner 类型，无需在应用中再分别传入两者的名称。
Engine 自动创建所需的 `dummy_map` 服务，并让两层复用它。

## 接入检查

| 检查事项 | 检查内容 |
| --- | --- |
| 环境表示 | Map 表示适合当前环境，并支持传感器数据接入。 |
| Map 到 Planner | WorldView 实现 Planner 的 `IWorldView` 所需的全部查询。 |
| Planner 到下一层 | 输出包含下一层需要的位姿、速度等字段。 |
| 最终输出 | 末层输出与底盘接口兼容；本例使用 `/cmd_vel`。 |
| 插件加载 | 插件已编译安装，Engine 可按 ROS 包名发现并加载。 |

插件包需在 CMake 中调用 `finenav_register_algo_plugin(<库目标名>)`，
供 `loadAlgoPlugins()` 按包名查找插件库。`finenav_example` 已包含该配置；
自定义插件需补充此声明，详见 [WorldView 教程](worldview.md)的“类型绑定与插件注册”。

其他组件见 [finenav_planners](plugins/planners.md) 和 [finenav_maps](plugins/maps.md)。
只克隆仓库，不会自动生成选定 Map 与 Planner 之间的 WorldView。

源码：[直线 Planner](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_planner/include/dummy_line_planner.hpp)、
[跟踪 Planner](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_planner/include/dummy_follow_planner.hpp)、
[Map](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_map/include/dummy_map.hpp)。

下一步：[3. 编写 WorldView](worldview.md)。
