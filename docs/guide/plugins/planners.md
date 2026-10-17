# finenav_planners

The planner collection is maintained in
[FineNav/finenav_planner](https://github.com/FineNav/finenav_planner).
The site section uses the plural name `finenav_planners`; the repository name is singular.

## Component catalogue

The following entries were checked against repository `main`, revision `bab23e5`:

| ROS package | Registered Planner | Purpose |
| --- | --- | --- |
| `finenav_route_planner` | `astar_planner` | Grid A* route search. |
| `finenav_route_planner` | `bfs_planner` | Grid search with a zero heuristic. |
| `finenav_route_planner` | `route_planner` | Topological route planning using a road graph. |
| `finenav_mppi_SE2` | `finenav_mppi_SE2` | MPPI local planning for ground robots. |
| `finenav_rotate_heading_aligner` | `finenav_rotate_shim` | Heading alignment component. |

Package READMEs describe their interfaces and parameters:
[route planners](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_route_planner),
[MPPI](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_mppi_SE2),
[heading alignment](https://github.com/FineNav/finenav_planner/tree/bab23e5/finenav_rotate_heading_aligner).

## Engine integration status

These packages are a component catalogue, not a preassembled navigation application.
The inspected revision does not yet declare `finenav_register_algo_plugin()`
resources for the new package-name discovery API. Before using a component,
publish its shared library through `finenav_cmake`, implement the required
WorldView if it reads a Map, and verify pipeline input/output compatibility.

In particular, check the heading aligner's completion semantics when porting it:
current Engine treats `std::nullopt` from a Planner as a failure, not successful
stage completion. Do not copy old TrackingPolicy-based instructions into the new pipeline.

## Example Planners

Getting Started uses two separate examples bundled in `finenav_example`:

| Planner | Input → output | Main parameters |
| --- | --- | --- |
| `dummy_line_planner` | Robot pose + goal → straight pose path | `path_resolution` (m), default 0.2. |
| `dummy_follow_planner` | Robot state + path → velocity command | `linear_speed` (m/s) 0.5; `angular_speed` gain 1.0; `lookahead_distance` (m) 1.0. |

The dummy follower illustrates tracking; it has no final-heading alignment or
terminal braking. See [Choose Map and Planner](../choose-algorithms.md) and
[Assemble a control pipeline](../cpp-application.md).

Need another algorithm? Continue with [Custom Planner](../custom-planner.md).
