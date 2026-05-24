#!/usr/bin/env python3

import pathlib

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from finenav_msgs.action import ComputePath


class ComputePathGoalSender(Node):
    def __init__(self):
        super().__init__("send_compute_path_goal")
        self.declare_parameter("action_name", "/compute_path")
        self.declare_parameter("graph_file", "")
        self.declare_parameter("start_node_id", 1)
        self.declare_parameter("goal_node_id", 4)
        self.declare_parameter("planner_id", "finenav_mppi_controller")

        action_name = self.get_parameter("action_name").value
        self._client = ActionClient(self, ComputePath, action_name)

    def send_goal(self):
        graph_file = pathlib.Path(self.get_parameter("graph_file").value)
        if not graph_file.is_file():
            raise FileNotFoundError(f"Route graph file does not exist: {graph_file}")

        goal = ComputePath.Goal()
        goal.planner_id = self.get_parameter("planner_id").value
        goal.graph_json = graph_file.read_text(encoding="utf-8")
        goal.start_node_id = int(self.get_parameter("start_node_id").value)
        goal.goal_node_id = int(self.get_parameter("goal_node_id").value)

        self.get_logger().info(
            f"Sending ComputePath goal: graph='{graph_file}', "
            f"start_node_id={goal.start_node_id}, goal_node_id={goal.goal_node_id}, "
            f"planner_id='{goal.planner_id}'"
        )

        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("ComputePath action server is not available")

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ComputePath goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"ComputePath finished with status={result.status}")


def main():
    rclpy.init()
    node = ComputePathGoalSender()
    try:
        node.send_goal()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
