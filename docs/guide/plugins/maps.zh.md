# finenav_maps

地图组件维护在 [FineNav/finenav_map](https://github.com/FineNav/finenav_map)。

## 组件目录

`stvl_map` 是使用 `sensor_msgs/msg/PointCloud2` 点云更新的滚动时空体素地图。
Map 注册名和参数命名空间均为 `stvl_map`。

支持接收地图坐标系或配置的输入坐标系（默认 `base_link`）内的点云，
并根据衰减配置清除过期占据体素。详见
[README 与参数定义](https://github.com/FineNav/finenav_map/tree/main/stvl_map)。

## Engine 接入状态

`stvl_map` 的 CMake 尚未发布 `loadAlgoPlugins({"package_name"})`
所需的 ament 资源。使用前需要补充插件发现声明，并为所选 Planner 实现 WorldView。
其 README 中仍有旧版 Engine 组装调用；应用 API 请以当前的
[WorldView](../worldview.md) 和 [Pipeline](../cpp-application.md) 教程为准。

## 示例 Map

Getting Started 使用 ROS 包 `finenav_example` 中的 `dummy_map`。
它没有观测数据源，`isOccupied()` 始终返回 false。
示例中的两个 WorldView 共享同一个 Map 服务，适合练习组件接入，不提供障碍物感知导航。

参数定义中有只读的 `window_size`，但当前 Dummy 实现的查询边界仍为固定值，
不宜用它演示地图窗口尺寸调整。

需要实现自己的环境表示和数据接入时，继续阅读[自定义 Map](../custom-map.md)。
