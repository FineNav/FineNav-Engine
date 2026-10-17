# 4. 组装控制 Pipeline

选定 Map、Planner 并完成 WorldView 后，应用代码只需加载插件包、创建控制层，再声明连接顺序。

## 示例应用

以下代码来自 `finenav_example/getting_started/hello_finenav.cpp`：

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine_cpp/finenav_engine.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    engine->loadAlgoPlugins({"finenav_example"});
    engine->createControlLayer(
        "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
    engine->createControlLayer(
        "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
    engine->registerControlBranch("navigate", {"line_layer", "follow_layer"});

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

## 数据流

```mermaid
flowchart LR
    G[导航目标] --> L["line_layer · 事件驱动"]
    L -->|位姿路径| F["follow_layer · 10 Hz"]
    F --> C[/cmd_vel/]
```

| 调用 | 作用 |
| --- | --- |
| `setDefaultBehaviorTree("GettingStartedNavigate")` | 没有显式指定树 ID 时，导航使用的默认任务树。 |
| `loadAlgoPlugins({"finenav_example"})` | 发现包内已安装的 Map、Planner、WorldView 插件库。 |
| `createControlLayer(..., "line_layer", EventDrivenPolicy{})` | 有新上游参考输入时生成路径。 |
| `createControlLayer(..., "follow_layer", PeriodicPolicy{10.0})` | 按目标 10 Hz，根据最新路径和机器人状态输出跟踪指令。 |
| `registerControlBranch("navigate", {"line_layer", "follow_layer"})` | 按顺序连接两层，注册名为 `navigate` 的分支。 |

第一层消费导航目标，第二层消费第一层生成的位姿路径。层名列表中不需要加入 `_root`。
注册分支只定义连接关系；由行为树中的 `ExecuteControlBranch` 激活执行。

这些组装接口返回 `bool`。最小示例省略了错误处理；编写自己的应用时，应检查返回值，失败时停止初始化。

## 参数配置

`hello_finenav.launch.py` 加载以下运行配置：

```yaml
/hello_finenav:
  ros__parameters:
    use_sim_time: true

/hello_finenav/finenav_navigator:
  ros__parameters:
    behavior_trees:
      - "finenav_example/getting_started/behavior_trees"

/hello_finenav/finenav_localizer:
  ros__parameters:
    pose_topic: ""
    pose_smoothing_time: 0.0
    initial_pose_topic: "/initialpose"
    twist_topic: ""

    odom_topic: "/odom"
    odom_use_twist: true

    state_history_duration: 5.0

    force_2d_mode: true
    publish_tf: false
    publish_odom: true

/hello_finenav/dummy_map:
  ros__parameters:
    dummy_map:
      window_size: 50.0

/hello_finenav/line_layer:
  ros__parameters:
    dummy_line_planner:
      path_resolution: 0.2

/hello_finenav/follow_layer:
  ros__parameters:
    dummy_follow_planner:
      linear_speed: 0.5
      angular_speed: 1.0
      lookahead_distance: 1.0
```

`/hello_finenav/line_layer` 和 `/hello_finenav/follow_layer` 来自 C++ 中的层名；
`dummy_line_planner`、`dummy_follow_planner` 是各算法的参数命名空间。
修改层名后，需要同步修改 YAML 中对应的节点路径。
定位器读取 `/odom`，默认底盘接口向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。

运行时调整跟踪速度：

```bash
ros2 param set /hello_finenav/follow_layer dummy_follow_planner.linear_speed 0.3
```

Planner 速度参数支持动态配置。C++ 组装关系变化后需要重新编译并重启。
YAML 中的行为树目录指向安装后的资源，树 ID 必须与 C++ 设置的默认 ID 一致。

源码：[应用入口](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp)、
[运行配置](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml)、
[公开 API](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_engine_cpp/include/finenav_engine_cpp/finenav_engine.hpp)。

下一步：[5. 按需求配置行为树](behavior-tree.md)。
