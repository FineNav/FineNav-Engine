# 3. 编写 WorldView

WorldView 把选定 Map 的查询适配成 Planner 需要的接口。
完整示例位于 `finenav_example/create_custom_world_view/`。
在示例包中完成适配后，可迁移至自定义插件包。

## Planner 接口

两个示例 Planner 分别定义自己的 `IWorldView`。例如，
`DummyLinePlanner` 在类内声明接口：

```cpp
class DummyLinePlanner {
  public:
    class IWorldView {
      public:
        virtual ~IWorldView() = default;
        virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
    };
    // ...
};
```

`DummyFollowPlanner` 也独立声明 `IWorldView`，目前包含相同的查询。
两个接口是独立类型，适配类分别实现对应 Planner 的接口。
Planner 只需要判断某个位置是否可通行，不需要依赖具体地图的数据存储方式。

## 实现适配类

在 `finenav::example` 命名空间内，直线 Planner 使用以下实现：

```cpp
class DummyLineWorldView : public DummyLinePlanner::IWorldView {
  public:
    using ConfigType = dummy_line_world_view::Params;

    DummyLineWorldView(rclcpp::Node::SharedPtr, const DummyMap& map) : map_(map) {}

    void configure(const ConfigType&) {}

    void prepare(const finenav::core::PlanningContext<DummyLinePlanner>&) {}

    bool isWalkable(const finenav::common::Position3D& position) const override { return !map_.isOccupied(position); }

  private:
    const DummyMap& map_;
};
```

构造函数接收 Map，`isWalkable()` 将查询转换为 `!map_.isOccupied(position)`。
`configure()` 用于应用适配参数，`prepare()` 可以准备本轮规划所需数据；本例中两者均为空实现。
`DummyFollowWorldView` 采用相同写法，继承 `DummyFollowPlanner::IWorldView`。

## 类型绑定与插件注册

注册文件将每个适配类与一个 Planner、所选 Map 绑定：

```cpp
FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyLineWorldView, dummy_line_world_view,
                           finenav::example::DummyLinePlanner, finenav::example::DummyMap)

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyFollowWorldView, dummy_follow_world_view,
                           finenav::example::DummyFollowPlanner, finenav::example::DummyMap)
```

这里的注册名就是应用调用 `createControlLayer()` 时传入的名称。
参数定义为 CMake 文件旁边的 `dummy_line_world_view.yaml` 和 `dummy_follow_world_view.yaml`。
该子目录的完整构建配置为：

```cmake
generate_parameter_library(dummy_line_world_view_params dummy_line_world_view.yaml)
generate_parameter_library(dummy_follow_world_view_params dummy_follow_world_view.yaml)

add_library(dummy_world_view SHARED src/dummy_world_view.cpp)
target_include_directories(dummy_world_view PUBLIC include)
target_link_libraries(dummy_world_view
    dummy_line_world_view_params
    dummy_follow_world_view_params
    dummy_map
    dummy_map_params
    dummy_line_planner
    dummy_line_planner_params
    dummy_follow_planner
    dummy_follow_planner_params)
ament_target_dependencies(dummy_world_view finenav_core)

install(TARGETS dummy_world_view ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

finenav_register_algo_plugin(dummy_world_view)
```

父级软件包负责查找 `finenav_cmake` 并加入这个子目录。
迁移到自己的包时，还需在 `package.xml` 声明依赖、安装共享库，
并通过 `finenav_register_algo_plugin()` 发布插件发现信息。

## 重新编译

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example
source install/setup.bash
```

重启应用以加载新插件。换 Map 时，调整 Map 类型和查询转换；换 Planner 时，
按照它实际声明的 `IWorldView` 实现接口，并更新注册绑定。

源码：[适配类](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/include/dummy_world_view.hpp)、
[类型注册](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/src/dummy_world_view.cpp)、
[CMake 配置](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/CMakeLists.txt)。

下一步：[4. 组装控制 Pipeline](cpp-application.md)。
