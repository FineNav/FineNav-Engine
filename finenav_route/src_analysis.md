# finenav_route src 文件保留说明

`finenav_route` 不是直接搬运 Nav2 的 `route_server` 节点，而是把 Nav2 原生拓扑导航拆成 FineNav 可加载的 planner：输入来自 `ControlLayer` 的 `PlanningContext`，输出为 `finenav::DataPacket` 轨迹。这样可以继续使用 FineNav 的 `ComputePlan` action、BT、Localizer 和后续 MPPI 跟踪层。

## 当前保留的 src

| 文件 | 作用 |
| --- | --- |
| `src/graph_route_planner.cpp` | FineNav 适配入口。负责读取 `finenav_route` 参数、懒加载 GeoJSON 图、用机器人位姿和目标位姿匹配最近图节点、调用 `RoutePlanner` 搜索、再调用 `PathConverter` 生成 FineNav 轨迹。 |
| `src/route_planner.cpp` | 拓扑图搜索核心。保留 Nav2 Route 的 Dijkstra 思路，按边代价搜索 `Route`；当前代价使用固定 `cost` 或折线几何长度乘 `distance_weight`。 |
| `src/node_spatial_tree.cpp` | 节点空间索引。基于 `nanoflann` 构建 2D KD-tree，将任意机器人/目标位姿映射到最近的拓扑节点。 |
| `src/path_converter.cpp` | 路径转换器。把稀疏 `Route` 转成 FineNav 轨迹点；如果前端 GeoJSON 的边提供 `LineString` 折线，则沿折线输出，不再把边简单拉直。 |
| `src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp` | GeoJSON 读取器。使用 `nlohmann::json` 解析前端提供的 `FeatureCollection`，读取 `Point` 节点和 `LineString` / `MultiLineString` 边，构建内部 `Graph`。 |

## 当前没有保留的 Nav2 src

| Nav2 文件类别 | 不保留原因 |
| --- | --- |
| `route_server.cpp` / `main.cpp` | FineNav 已有 `ControlLayer` action server 和 BT 调度，不需要再启动 Nav2 生命周期节点。 |
| `graph_loader.cpp` | 原文件依赖 pluginlib、TF 和 Nav2 生命周期参数；当前只保留内置 GeoJSON loader，直接从 `graph_filepath` 读取。 |
| `edge_scorer.cpp` 与 edge scorer 插件 | 为降低接入复杂度，先使用固定边代价和距离代价。动态封路、语义代价、costmap 代价可后续以 FineNav 风格重新加入。 |
| `goal_intent_extractor.cpp` | FineNav 的 `PlanningContext` 已提供当前机器人位姿和目标位姿，节点匹配与起终点裁剪已收敛到 `GraphRoutePlanner`。 |
| `route_tracker.cpp` / `operations_manager.cpp` / route operation 插件 | FineNav 当前由后续控制层和 BT 判断跟踪、到达、卡死等状态；Nav2 Route 的执行期事件系统暂不移植。 |
| `graph_saver.cpp` 与 saver 插件 | 前端是图数据来源，当前只需要读取。运行时编辑后保存图可以后续单独做。 |

## 前端 GeoJSON 数据约定

前端可以传标准 GeoJSON `FeatureCollection`，解析使用 `nlohmann::json`。当前必需字段如下：

| 元素 | 必需字段 |
| --- | --- |
| 节点 `Point` | `properties.id`，`geometry.coordinates` 至少包含 `[x, y]`。`properties.frame` 可选。 |
| 边 `LineString` | `properties.id`、`properties.startid`、`properties.endid`、`geometry.coordinates`。`cost` 和 `overridable` 可选。 |

边的 `geometry.coordinates` 会被完整保存为 `DirectionalEdge::polyline`。因此你给的多点折线会原样参与输出轨迹；搜索代价也会按折线总长度计算，而不是只按起点到终点直线距离。

## FineNav 接入方式

`GraphRoutePlanner` 已注册为 `planner_id = "finenav_route"`。可以通过 `FineNavEngine::createRouteLayer()` 创建独立拓扑路由层：

```cpp
auto route_layer = finenav_engine->createRouteLayer("route_layer");

finenav_engine->registerPipeline(
  FineNavEngine::PipelineIO{
    .input_topic = "",
    .output_topic = "route_path",
  },
  route_layer);
```

参数示例：

```yaml
/hello_finenav/route_layer:
  ros__parameters:
    finenav_route:
      graph_filepath: "/absolute/path/to/graph.geojson"
      route_frame: "map"
      path_density: 0.05
      smooth_corners: false
      distance_weight: 1.0
```

前端 GeoJSON
  ↓
保存为 graph.geojson，路径写入 finenav_route.graph_filepath
  ↓
FineNavEngine::createRouteLayer("route_layer")
  ↓
ControlLayer<PlannerSet<GraphRoutePlanner>>
  ↓
BT / ComputePlan action 请求 route_layer
  ↓
ControlLayer 从 Localizer 注入 robot_state
  ↓
GraphRoutePlanner::plan(ctx, RouteGraphView)
  ↓
GeoJsonGraphFileLoader 用 nlohmann::json 读取 GeoJSON
  ↓
Point 生成 Node，LineString 生成 DirectionalEdge + polyline
  ↓
NodeSpatialTree 找 robot_pose 和 goal_pose 最近拓扑节点
  ↓
RoutePlanner 做 Dijkstra 拓扑搜索
  ↓
PathConverter 把 Route 转成 FineNav Trajectory
  ↓
ControlLayer 发布 finenav_msgs::msg::Trajectory
  ↓
后级 MPPI / 控制层把它当参考轨迹跟踪

ControlLayer ::planOnce()
planner.plan(ctx, map_view)