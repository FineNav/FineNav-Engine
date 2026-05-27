#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import os
import signal

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')
        
        self.declare_parameter('launch_file', '')
        self.launch_file = self.get_parameter('launch_file').value
        
        self.process = None
        
        self.start_srv = self.create_service(Trigger, '/start_navigation', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_navigation', self.stop_callback)
        
        self.get_logger().info("Navigation Manager started.")
        
        # Start navigation by default
        self.start_navigation()

    def start_navigation(self):
        if self.process is not None:
            self.get_logger().warn("Navigation is already running.")
            return False, "Navigation is already running."
        
        if not self.launch_file:
            return False, "No launch file specified."

        try:
            # We use a new process group to make sure we can kill all child processes
            self.process = subprocess.Popen(
                ['ros2', 'launch', self.launch_file],
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Started navigation from {self.launch_file}")
            return True, "Navigation started."
        except Exception as e:
            self.get_logger().error(f"Failed to start navigation: {e}")
            return False, str(e)

    def stop_navigation(self):
        if self.process is None:
            self.get_logger().warn("Navigation is not running.")
            return False, "Navigation is not running."
        
        try:
            # Kill the whole process group
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=5)
            self.process = None
            self.get_logger().info("Navigation stopped.")
            return True, "Navigation stopped."
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Navigation process did not stop gracefully, killing it.")
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process = None
            return True, "Navigation force stopped."
        except Exception as e:
            self.get_logger().error(f"Failed to stop navigation: {e}")
            return False, str(e)

    def start_callback(self, request, response):
        success, message = self.start_navigation()
        response.success = success
        response.message = message
        return response

    def stop_callback(self, request, response):
        success, message = self.stop_navigation()
        response.success = success
        response.message = message
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.process:
            node.stop_navigation()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
