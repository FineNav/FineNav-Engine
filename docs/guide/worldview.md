# 3. Write a WorldView

A WorldView adapts the chosen Map to the interface required by a Planner.
The working example is in `finenav_example/create_custom_world_view/`.
You can read or modify it in place before creating a separate plugin package.

## Read the Planner's interface

Both example Planners use `DummyWorldView` as their `IWorldView`:

```cpp
class DummyWorldView {
  public:
    virtual ~DummyWorldView() = default;
    virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
};
```

The Planner asks whether a position is walkable; it does not need to know how
the underlying Map stores its data.

## Implement the adapter

Inside namespace `finenav::example`, the line Planner's adapter is:

```cpp
class DummyLineWorldView final : public DummyWorldView {
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

The constructor receives the Map. `isWalkable()` translates the query to
`!map_.isOccupied(position)`. `configure()` applies adapter parameters and
`prepare()` can prepare per-plan data; both are empty in this example.
`DummyFollowWorldView` follows the same pattern with `DummyFollowPlanner`.

## Bind the types and publish the library

The registration file binds each adapter to one Planner and the selected Map:

```cpp
FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyLineWorldView, dummy_line_world_view,
                           finenav::example::DummyLinePlanner, finenav::example::DummyMap)

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyFollowWorldView, dummy_follow_world_view,
                           finenav::example::DummyFollowPlanner, finenav::example::DummyMap)
```

The registration name is what the application passes to `createControlLayer()`.
The parameter schema files are `dummy_line_world_view.yaml` and
`dummy_follow_world_view.yaml` beside the CMake file. The example's complete
subdirectory build configuration is:

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

The parent package finds `finenav_cmake` and includes this subdirectory.
When moving to your own package, also declare the dependencies in `package.xml`,
install the shared library, and publish it with `finenav_register_algo_plugin()`.

## Rebuild after a change

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example
source install/setup.bash
```

Restart the application to load the rebuilt library. For a different Map,
replace the Map type and query translation; for a different Planner,
implement its actual `IWorldView` contract and update the registration.

Source: [adapter classes](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_world_view/include/dummy_world_view.hpp),
[registrations](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_world_view/src/dummy_world_view.cpp),
[CMake](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_world_view/CMakeLists.txt).

Continue with [4. Assemble a control pipeline](cpp-application.md).
