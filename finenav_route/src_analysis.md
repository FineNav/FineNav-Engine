# finenav_route src 文件保留说明

当前 `finenav_route` 按前端拓扑图输入工作：前端传入 `example.json` 风格 GeoJSON、起点节点 id 和终点节点 id；route 层解析图、按拓扑关系搜索、拼接边的 `geometry.coordinates`，生成 `finenav_msgs::msg::Trajectory` 给 MPPI 跟踪。

## 当前保留的 src

| 文件 | 作用 |
| --- | --- |
| `src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp` | 使用 `nlohmann::json` 解析 GeoJSON。`Point` 生成节点，`LineString` / `MultiLineString` 生成有向边，并保存边上的 polyline。 |
| `src/route_planner.cpp` | 拓扑图搜索核心。根据 `start_node_id` / `goal_node_id` 在 `GraphToIDMap` 中查索引，然后按边代价搜索 `Route`。 |
| `src/path_converter.cpp` | 将搜索得到的 `Route` 转成轨迹。严格沿每条边保存的 polyline 输出；缺失 polyline 会作为非法图处理。 |
| `src/route_path_planner.cpp` | 当前 FineNav 接入入口。接收 `graph_json + start_node_id + goal_node_id`，串起 GeoJSON 解析、拓扑搜索和轨迹转换，输出 `finenav_msgs::msg::Trajectory`。 |

## 当前不再保留的旧链路

| 文件/能力 | 删除原因 |
| --- | --- |
| `graph_route_planner.cpp` / `graph_route_planner.hpp` | 它走 FineNav `ControlLayer::plan(ctx, map_view)`，只能从 `ctx.robot_state.pose` / `ctx.goal_pose` 反推节点；这不符合当前“前端直接给节点 id 和拓扑图”的链路。 |
| `node_spatial_tree.cpp` / `node_spatial_tree.hpp` | KD 树只用于 `pose -> 最近拓扑节点`。当前输入已经是 `start_node_id / goal_node_id`，不需要空间最近邻。 |
| `corner_smoothing.hpp` / `test_corner_smoothing.cpp` | 圆弧拐角平滑会改写前端边的真实 polyline。当前以前端 `geometry.coordinates` 为准，route 层不再做额外平滑。 |
| `finenav_route.yaml` | 旧 `GraphRoutePlanner` 的参数文件。当前 graph 由 `ComputePath.action` 请求直接携带，不再通过参数文件加载。 |
| `test_spatial_tree.cpp` / `performance_benchmarking.cpp` | 只服务于旧 KD 树链路，已从测试构建中移除。 |

## 当前数据流

```text
前端 ComputePath.action
  graph_json: example.json 风格 GeoJSON
  start_node_id
  goal_node_id
  planner_id: MPPI 控制层 planner id
  ↓
compute_path_bridge_node
  ↓
RoutePathPlanner::plan(graph_json, start_node_id, goal_node_id)
  ↓
GeoJsonGraphFileLoader
  Point -> Node
  LineString / MultiLineString -> DirectionalEdge + polyline
  ↓
RoutePlanner
  GraphToIDMap: node id -> graph index
  Dijkstra 搜索 Route
  ↓
PathConverter
  拼接每条 edge.polyline
  输出 finenav_msgs::msg::Trajectory
  ↓
发布 line_path
  ↓
调用原 ComputePlan.action
  ↓
MPPI ControlLayer 从 line_path 读取 ref_traj，
并继续使用自己的真实局部 MapView 做控制
```

## GeoJSON 约定

节点：

```json
{
  "type": "Feature",
  "geometry": {
    "type": "Point",
    "coordinates": [0.1168, 0.9273, 0]
  },
  "properties": {
    "id": 1,
    "frame": "WP_a6b2"
  }
}
```

边：

```json
{
  "type": "Feature",
  "geometry": {
    "type": "LineString",
    "coordinates": [[0.1168, 0.9273, 0], [0.0805, 4.3659, 0]]
  },
  "properties": {
    "id": 8,
    "startid": 1,
    "endid": 2,
    "cost": 0,
    "overridable": true
  }
}
```

其中 `startid` / `endid` 决定拓扑连接关系，`geometry.coordinates` 决定输出给 MPPI 的实际参考轨迹形状。
