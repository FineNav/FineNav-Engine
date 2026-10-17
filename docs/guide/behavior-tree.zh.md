# 5. 按需求配置行为树

控制 pipeline 计算运动指令，Behavior Tree（行为树）管理任务执行、完成条件和故障恢复。
从 `finenav_example/getting_started/behavior_trees/navigate_to_pose.xml` 开始修改。

## 示例行为树

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

C++ 应用将 `GettingStartedNavigate` 设为默认树。导航器根据 YAML 中的目录加载 XML：

```yaml
/hello_finenav/finenav_navigator:
  ros__parameters:
    behavior_trees:
      - "finenav_example/getting_started/behavior_trees"
```

| 节点 | 作用 |
| --- | --- |
| `PublishNavigationGoal` | 将请求的导航目标提供给第一层。 |
| `ReactiveFallback` | 在分支运行期间持续重新检查到达条件。 |
| `IsGoalReached` | 检查配置的位置、朝向与速度容差。 |
| `ExecuteControlBranch branch="navigate"` | 激活 C++ 注册的分支，保持 RUNNING，直到被中止或某一层故障。 |
| `StopChassisMotion` | 到达目标或恢复前发送停车指令。 |
| `RecoveryNode` | 恢复子节点成功后重试导航子节点，次数由配置限制。 |
| `GetFaultedControlLayer` + `RestartFromControlLayer` | 定位故障层，请求从该层开始重启下游流程。 |

`ExecuteControlBranch` 不负责判断到达，必须配合 `IsGoalReached` 等完成条件，任务才能正常结束。
恢复序列是示例提供的策略；换用实际算法后，需要验证真实故障与重启场景，再依赖它进行恢复。

## 任务配置

| 需求 | 修改位置 |
| --- | --- |
| 调整到点范围 | 修改 `xy_tolerance`，单位为米。 |
| 到点后必须朝向指定方向 | 设置正数 `yaw_tolerance`（弧度），并选用具备终点朝向控制能力的算法。 |
| 必须停稳后才算成功 | 设置正数 `speed_tolerance`（m/s），并让控制器在完成前减速。 |
| 调整失败重试次数 | 修改 `number_of_retries`。 |
| 执行另一条 pipeline | 先在 C++ 注册分支，再将准确名称填入 `branch`。 |

Dummy 跟踪器没有终点朝向对齐和基于剩余距离的到点减速；
示例原始行为树却同时要求 `yaw_tolerance="0.1"` 与 `speed_tolerance="0.1"`，
所以仅移动到目标附近，不一定能结束任务。

如果只是进行**到达位置即结束的仿真练习**，将 `DriveToGoal` 中原有的 `IsGoalReached` 元素替换为：

```xml
<IsGoalReached xy_tolerance="0.25"
               yaw_tolerance="0.0"
               speed_tolerance="0.0"/>
```

这个节点中，0 表示关闭对应检查。满足位置条件后，行为树中止分支并调用 `StopChassisMotion`。
这只是调整练习的验收条件，不会为跟踪器增加减速或朝向控制能力。

## 配置验证

修改源包中的 XML，保持运行 YAML 中的目录正确。
如果更改主树 ID，还需要同步修改 `setDefaultBehaviorTree()`。
重新编译以更新安装资源，再重启应用：

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

发送新目标，确认所选条件能够结束任务，且 `/cmd_vel` 收到停车指令。
修改恢复策略时，还要主动覆盖算法失败、重试次数耗尽等场景；只跑通正常流程不能证明恢复有效。

源码：[示例行为树](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/behavior_trees/navigate_to_pose.xml)、
[完成条件实现](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_navigator/src/plugins/condition/is_goal_reached.cpp)。

组件目录：[finenav_planners](plugins/planners.md) 和 [finenav_maps](plugins/maps.md)。
