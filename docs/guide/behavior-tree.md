# 5. Configure a Behavior Tree

The pipeline defines how to compute commands. The Behavior Tree defines when
to execute it, what counts as arrival, and how to respond to a failure.
Start with `finenav_example/getting_started/behavior_trees/navigate_to_pose.xml`.

## Read the supplied tree

```xml
<root BTCPP_format="4">

    <BehaviorTree ID="GettingStartedNavigate">
        <Sequence>
            <PublishNavigationGoal/>

            <RecoveryNode number_of_retries="3">
                <SubTree ID="DriveToGoal"/>

                <!-- Control branch FAULTED: recovery -->
                <Sequence name="Recover">
                    <GetFaultedControlLayer control_layer="{faulted_layer}"/>
                    <StopChassisMotion/>
                    <RestartFromControlLayer control_layer="{faulted_layer}"/>
                </Sequence>
            </RecoveryNode>
        </Sequence>
    </BehaviorTree>


    <BehaviorTree ID="DriveToGoal">
        <Sequence>
            <ReactiveFallback name="NavigateOrSucceed">
                <IsGoalReached  xy_tolerance="0.25"
                                yaw_tolerance="0.1"
                                speed_tolerance="0.1"/>
                <ExecuteControlBranch branch="navigate"/>
            </ReactiveFallback>
            <!-- Goal reached: publish stop chassis. -->
            <StopChassisMotion/>
        </Sequence>
    </BehaviorTree>

</root>
```

The C++ application chooses `GettingStartedNavigate` as its default tree.
The navigator loads the XML from the directory configured in YAML:

```yaml
/hello_finenav/finenav_navigator:
  ros__parameters:
    behavior_trees:
      - "finenav_example/getting_started/behavior_trees"
```

| Node | Meaning in this task |
| --- | --- |
| `PublishNavigationGoal` | Supplies the requested goal to the first layer. |
| `ReactiveFallback` | Rechecks arrival while the branch is running. |
| `IsGoalReached` | Checks the configured position, orientation, and speed tolerances. |
| `ExecuteControlBranch branch="navigate"` | Activates the C++-registered branch; stays RUNNING until halted or a layer faults. |
| `StopChassisMotion` | Sends a stop command when reached, or before recovery. |
| `RecoveryNode` | Retries the navigation child after the recovery child succeeds, up to the configured retry count. |
| `GetFaultedControlLayer` + `RestartFromControlLayer` | Selects the faulted layer and requests restarting that layer and its downstream layers. |

`ExecuteControlBranch` does not decide that a goal is reached. Keep a completion
condition such as `IsGoalReached` so the task can finish. The recovery sequence
is the supplied example policy; validate actual failure/restart cases with your
selected algorithms before relying on it.

## Turn requirements into configuration

| Requirement | Change |
| --- | --- |
| Accept a larger or smaller arrival region | Adjust `xy_tolerance` in metres. |
| Require a particular terminal heading | Use a positive `yaw_tolerance` in radians and a controller that can align at the goal. |
| Require a stopped robot | Use positive `speed_tolerance` in m/s and a controller that brakes before success. |
| Retry a different number of times | Change `number_of_retries`. |
| Execute another pipeline | Register it in C++, then use its exact name in `branch`. |

The supplied dummy follower has no final-yaw alignment or distance-based terminal
braking. Its default tree requests both (`yaw_tolerance="0.1"` and
`speed_tolerance="0.1"`), so reaching the vicinity alone may not complete the task.

For a **position-only simulation exercise**, replace the existing `IsGoalReached`
element inside `DriveToGoal` with:

```xml
<IsGoalReached xy_tolerance="0.25"
               yaw_tolerance="0.0"
               speed_tolerance="0.0"/>
```

For this node, zero disables the corresponding check. Once position is reached,
the tree halts the branch and calls `StopChassisMotion`. This changes the
exercise's acceptance criteria; it does not add braking or heading control.

## Apply and check the change

Edit the XML in the source package. Keep its directory in the runtime YAML.
If you rename the main tree ID, update `setDefaultBehaviorTree()` as well.
Rebuild to update installed resources and restart:

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

Send a new goal and check that the chosen condition ends the task and `/cmd_vel`
receives a stop command. For recovery changes, also exercise an algorithm failure
and retry exhaustion; normal-path execution does not validate recovery.

Source: [example tree](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/getting_started/behavior_trees/navigate_to_pose.xml),
[completion checks](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_navigator/src/plugins/condition/is_goal_reached.cpp).

Next, explore [finenav_planners](plugins/planners.md) and [finenav_maps](plugins/maps.md).
