#!/usr/bin/env python3
"""
MPPI Critics Visualizer
=======================
Subscribe to /mppi_critic_stats/critics_stats and plot critic performance
in real-time using matplotlib.

Usage:
    ros2 run finenav_mppi_controller critics_visualizer.py
    # or with custom topic:
    ros2 run finenav_mppi_controller critics_visualizer.py --ros-args -p topic:=/custom/topic
"""

import sys
from collections import OrderedDict

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from finenav_msgs.msg import CriticsStats


class CriticsVisualizer(Node):
    """ROS 2 node that subscribes to CriticsStats and renders live plots."""

    def __init__(self):
        super().__init__('critics_visualizer')

        topic = self.declare_parameter('topic', '/mppi_critic_stats/critics_stats').value
        self.history_len = self.declare_parameter('history_length', 100).value

        self.sub = self.create_subscription(CriticsStats, topic, self.stats_callback, 10)

        # Rolling history: critic_name → deque of (cost_sum, changed)
        self.history = OrderedDict()
        self.latest = None
        self.frame_count = 0

        self.get_logger().info(f'Listening on {topic} — waiting for CriticsStats...')

        # ── Setup matplotlib ──────────────────────────────────────────────
        self.fig, (self.ax_bar, self.ax_history) = plt.subplots(
            2, 1, figsize=(14, 9),
            gridspec_kw={'height_ratios': [1, 1.2]})
        self.fig.canvas.manager.set_window_title('MPPI Critics Performance')
        self.fig.tight_layout(pad=3.5)

        self.ani = FuncAnimation(self.fig, self._update_plot, interval=200,
                                  cache_frame_data=False, blit=False)
        plt.show(block=False)

    # ── ROS callback ──────────────────────────────────────────────────────
    def stats_callback(self, msg: CriticsStats):
        self.frame_count += 1
        self.latest = msg

        for name, changed, cost in zip(msg.critics, msg.changed, msg.costs_sum):
            if name not in self.history:
                self.history[name] = []
            self.history[name].append((cost, changed))
            # Trim history
            if len(self.history[name]) > self.history_len:
                self.history[name].pop(0)

    # ── Plot update ───────────────────────────────────────────────────────
    def _update_plot(self, _frame):
        # No data yet → show waiting message
        if self.latest is None:
            self.ax_bar.clear()
            self.ax_bar.text(0.5, 0.5, 'Waiting for CriticsStats…\n'
                              'Is visualize: true set in mppi_params.yaml?',
                              transform=self.ax_bar.transAxes,
                              ha='center', va='center', fontsize=14, color='gray')
            self.ax_bar.set_xticks([])
            self.ax_bar.set_yticks([])
            self.ax_history.clear()
            self.ax_history.set_xticks([])
            self.ax_history.set_yticks([])
            return

        # ── Top: bar chart of current costs ───────────────────────────────
        self.ax_bar.clear()
        names = list(self.latest.critics)
        costs = list(self.latest.costs_sum)
        changed = list(self.latest.changed)

        y_pos = np.arange(len(names))
        colors = ['#2ecc71' if c else '#bdc3c7' for c in changed]
        bars = self.ax_bar.barh(y_pos, costs, color=colors, edgecolor='#2c3e50', linewidth=0.5)

        # Annotate each bar with its value
        for i, (bar, cost) in enumerate(zip(bars, costs)):
            x_pos = bar.get_width()
            label = f' {cost:.1f}' if abs(cost) < 10000 else f' {cost:.2e}'
            self.ax_bar.text(x_pos, bar.get_y() + bar.get_height() / 2,
                              label, va='center', ha='left', fontsize=9)

        self.ax_bar.set_yticks(y_pos)
        self.ax_bar.set_yticklabels(names, fontsize=9)
        self.ax_bar.invert_yaxis()
        self.ax_bar.set_xlabel('Cost Contribution', fontsize=10)
        self.ax_bar.set_title(f'Critic Cost Contribution  (frame #{self.frame_count})\n'
                               'green = changed (non-zero)   gray = unchanged (zero)',
                               fontsize=11, fontweight='bold')
        self.ax_bar.grid(axis='x', alpha=0.3)

        # ── Bottom: cost history over time ────────────────────────────────
        self.ax_history.clear()

        if self.history:
            max_cost = 0
            for name, entries in self.history.items():
                if not entries:
                    continue
                ys = np.array([e[0] for e in entries])
                xs = np.arange(len(ys))
                max_cost = max(max_cost, np.max(np.abs(ys)))
                self.ax_history.plot(xs, ys, linewidth=1.2, alpha=0.85, label=name)

            self.ax_history.axhline(y=0, color='black', linewidth=0.5, linestyle='--')
            self.ax_history.set_xlabel('Frame', fontsize=10)
            self.ax_history.set_ylabel('Cost Contribution', fontsize=10)
            self.ax_history.set_title(f'Cost History  (rolling {self.history_len} frames)',
                                       fontsize=11, fontweight='bold')
            self.ax_history.legend(loc='upper left', fontsize=8, ncol=2,
                                    framealpha=0.7)
            self.ax_history.grid(alpha=0.3)

            # Auto-scale x-axis to show full history
            total = max(len(v) for v in self.history.values()) if self.history else 10
            self.ax_history.set_xlim(0, max(total, 10))
            if max_cost > 0:
                self.ax_history.set_ylim(-max_cost * 1.15, max_cost * 1.15)

        self.fig.canvas.draw_idle()


def main(args=None):
    rclpy.init(args=args)
    node = CriticsVisualizer()

    # Spin ROS in a background thread so matplotlib can run on the main thread
    from rclpy.executors import SingleThreadedExecutor
    import threading
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
