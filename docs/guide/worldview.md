# Write a WorldView

A WorldView adapts a concrete Map to the query interface declared by a Planner.
The layered example uses separate WorldView registrations for path generation
and path following while both read the same `DummyMap`.

## Connect map queries

Both example Planners expect `isWalkable(position)`. `DummyMap` provides
`isOccupied(position)`, so the WorldView returns the inverse:

```cpp
#include <rclcpp/rclcpp.hpp>

#include "dummy_line_planner.hpp"
#include "dummy_map.hpp"
#include "finenav_example/dummy_line_world_view_params.hpp"

namespace finenav::example {

class DummyLineWorldView final : public DummyWorldView {
public:
    using ConfigType = dummy_line_world_view::Params;

    DummyLineWorldView(rclcpp::Node::SharedPtr, const DummyMap& map)
        : map_(map) {}

    void configure(const ConfigType&) {}
    void prepare(const core::PlanningContext<DummyLinePlanner>&) {}

    bool isWalkable(const common::Position3D& position) const override {
        return !map_.isOccupied(position);
    }

private:
    const DummyMap& map_;
};

}  // namespace finenav::example
```

Use `configure()` for WorldView parameters. `prepare()` runs once before each
`plan()` call while the backing maps are read-locked. Direct queries can leave
it empty; cached inflation, terrain, or cost-field work belongs there and can
use `context.maps_changed` to skip unchanged maps.

## Register and publish the WorldView

```cpp
#include "dummy_world_view.hpp"
#include "finenav_core/algo_register_macros.hpp"

FINENAV_REGISTER_WORLDVIEW(
    finenav::example::DummyLineWorldView,
    dummy_line_world_view,
    finenav::example::DummyLinePlanner,
    finenav::example::DummyMap)
```

The registration binds a WorldView name to Planner and Map types. FineNavEngine
resolves those types to registered plugins, creates or reuses the required map
servers, and builds the layer. Publish the containing shared library once:

```cmake
finenav_register_algo_plugin(dummy_world_view)
```

The complete
[WorldView examples](https://github.com/FineNav/FineNav-Engine/tree/dev/finenav_example/create_custom_world_view)
register `dummy_line_world_view` and `dummy_follow_world_view` in one library.
Next, [configure the C++ application](cpp-application.md) to load the package.
