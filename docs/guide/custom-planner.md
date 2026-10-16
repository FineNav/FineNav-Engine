# Custom Planner

Write a custom Planner when the available algorithms do not fit your robot or
task. Start from the
[Planner examples](https://github.com/FineNav/FineNav-Engine/tree/dev/finenav_example/create_custom_planner).

## Define the contract

Declare an `IWorldView` interface containing the environment queries the
algorithm uses. Keep those queries independent of a concrete Map type so a
WorldView can adapt the selected Map later.

Choose input and output profiles for each pipeline stage. The layered example
uses pose input and pose output for path generation:

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};
```

The follower consumes that pose path and produces velocity commands:

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};
```

## Implement the algorithm

`DummyLinePlanner` demonstrates the complete Planner contract:

```cpp
using ConfigType = dummy_line_planner::Params;
using IWorldView = DummyWorldView;
using Context = finenav::core::PlanningContext<DummyLinePlanner>;
using Result = std::optional<finenav::core::OutputTrajectoryT<DummyLinePlanner>>;

void configure(const ConfigType& config);
void reset();
Result plan(const Context& context, const IWorldView& world_view);
```

Apply parameters in `configure()`, clear episode state in `reset()`, and compute
the output from `robot_state`, `ref_traj`, and the WorldView in `plan()`. Return
`std::nullopt` when valid input cannot produce a result.

## Register and publish the plugin

Define the parameter YAML, generate its parameter library, and build the Planner
as a shared library. The YAML namespace, generated parameter namespace, and
registration name must match:

```cpp
FINENAV_REGISTER_PLANNER(
    finenav::example::DummyLinePlanner,
    dummy_line_planner)
```

Publish the library through the package's ament resource:

```cmake
finenav_register_algo_plugin(dummy_line_planner)
```

Then [write a WorldView](worldview.md) for the Planner and selected Map, and
assemble the registered components in a [C++ application](cpp-application.md).
