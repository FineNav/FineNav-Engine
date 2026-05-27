#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
import os
import datetime
import cv2

# Try to import custom service, fallback to a dummy if not available
try:
    from finenav_msgs.srv import SetMap
except ImportError:
    # If not compiled, we can't easily use it, but we'll define the class structure
    # for the sake of the script's logic.
    class SetMap:
        class Request:
            def __init__(self):
                self.map = OccupancyGrid()
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""

class CloudToMapAdapter(Node):
    def __init__(self):
        super().__init__('cloud_to_map_adapter')

        # Parameters
        self.declare_parameter('min_z', 2.0)
        self.declare_parameter('max_z', 5.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 1000)  # 50m
        self.declare_parameter('height', 1000) # 50m
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Grid Map initialization (0: unknown, 100: occupied, 0: free)
        # We'll use -1 for unknown, 100 for occupied, 0 for free
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        self.is_mapping = False
        self.has_disk_map = False
        
        # Initial transform from odom to the start of mapping (initial frame)
        self.initial_pose_in_odom = None # Will store a Transform object

        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_for_lio',
            self.cloud_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load existing map from disk if it exists
        self.load_map()

        # Services
        self.start_srv = self.create_service(Trigger, '/start_mapping', self.start_mapping_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_mapping', self.stop_mapping_callback)
        
        # Check if SetMap is available
        try:
            from finenav_msgs.srv import SetMap as ActualSetMap
            self.set_map_srv = self.create_service(ActualSetMap, '/set_map', self.set_map_callback)
        except ImportError:
            self.get_logger().error("finenav_msgs.srv.SetMap not found. /set_map service not initialized.")

        # Timer for TF to Pose and Map publishing
        self.timer = self.create_timer(0.05, self.publish_pose)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("Cloud to Map Adapter initialized.")

    def cloud_callback(self, msg):
        if not self.is_mapping:
            return

        try:
            # Get transform from cloud frame to odom frame
            T_odom_cloud = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # If we have an initial pose, we want to work in the 'initial' frame
            # P_initial = (T_odom_initial)^-1 * T_odom_cloud * P_cloud
            
            import math
            
            # Extract T_odom_cloud
            tx_oc = T_odom_cloud.transform.translation.x
            ty_oc = T_odom_cloud.transform.translation.y
            q_oc = T_odom_cloud.transform.rotation
            yaw_oc = math.atan2(2 * (q_oc.w * q_oc.z + q_oc.x * q_oc.y), 1 - 2 * (q_oc.y * q_oc.y + q_oc.z * q_oc.z))

            if self.initial_pose_in_odom:
                # Extract T_odom_initial
                tx_oi = self.initial_pose_in_odom.transform.translation.x
                ty_oi = self.initial_pose_in_odom.transform.translation.y
                q_oi = self.initial_pose_in_odom.transform.rotation
                yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
                
                # Relative transform: Initial -> Cloud
                # dx, dy in odom
                dx = tx_oc - tx_oi
                dy = ty_oc - ty_oi
                
                # Rotate dx, dy into initial frame
                tx = dx * math.cos(yaw_oi) + dy * math.sin(yaw_oi)
                ty = -dx * math.sin(yaw_oi) + dy * math.cos(yaw_oi)
                tz = T_odom_cloud.transform.translation.z - self.initial_pose_in_odom.transform.translation.z
                
                # Relative yaw
                yaw = yaw_oc - yaw_oi
            else:
                tx, ty, tz = tx_oc, ty_oc, T_odom_cloud.transform.translation.z
                yaw = yaw_oc

        except Exception as e:
            self.get_logger().warn(f"Could not transform cloud: {e}")
            return

        # Read points
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points)
        except Exception as e:
            self.get_logger().error(f"Error reading points: {e}")
            return

        if not points_list:
            return
        
        # Sensor position in grid coordinates (relative to initial frame)
        sx = int((tx - self.origin_x) / self.resolution)
        sy = int((ty - self.origin_y) / self.resolution)

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # To speed up, we can process points in batches or use a subset
        # We use a mask to track areas to be cleared to avoid clearing existing obstacles
        free_mask = np.zeros_like(self.grid, dtype=np.uint8)
        
        count = 0
        for p in points_list:
            lx, ly, lz = p
            
            # Simple 3D transform
            gx = lx * cos_y - ly * sin_y + tx
            gy = lx * sin_y + ly * cos_y + ty
            gz = lz + tz

            # Target position in grid coordinates
            ix = int((gx - self.origin_x) / self.resolution)
            iy = int((gy - self.origin_y) / self.resolution)

            if 0 <= ix < self.width and 0 <= iy < self.height:
                if self.min_z <= gz <= self.max_z:
                    # Obstacle detected: Mark the path as potentially free
                    if 0 <= sx < self.width and 0 <= sy < self.height:
                        cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)
                    
                    # Mark the hit point as occupied (this happens last)
                    self.grid[iy, ix] = 100
                    count += 1
                elif gz < self.min_z:
                    # Point is below min_z (likely floor): Mark the path as potentially free
                    if 0 <= sx < self.width and 0 <= sy < self.height:
                        cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)

        # Apply free_mask: only set to 0 (free) if it was -1 (unknown)
        # This ensures we don't clear existing 100 (occupied)
        self.grid[(free_mask == 1) & (self.grid == -1)] = 0
        
        if count > 0:
            self.get_logger().info(f"Projected {count} new points to map. Total occupied: {np.sum(self.grid == 100)}")

    def publish_map(self):
        if not self.is_mapping and not self.has_disk_map:
            # self.get_logger().debug("Not publishing map: not mapping and no disk map.")
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Calculate origin in odom frame
        # If we are in initial frame mapping, origin_x/y is relative to initial frame
        # If we have an initial pose, transform the origin (origin_x, origin_y) in initial frame to odom
        if self.is_mapping and self.initial_pose_in_odom:
            import math
            tx_oi = self.initial_pose_in_odom.transform.translation.x
            ty_oi = self.initial_pose_in_odom.transform.translation.y
            q_oi = self.initial_pose_in_odom.transform.rotation
            yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
            
            # Map origin in odom = T_odom_initial * [origin_x, origin_y, 0]
            ox_odom = tx_oi + self.origin_x * math.cos(yaw_oi) - self.origin_y * math.sin(yaw_oi)
            oy_odom = ty_oi + self.origin_x * math.sin(yaw_oi) + self.origin_y * math.cos(yaw_oi)
            msg.info.origin.position.x = ox_odom
            msg.info.origin.position.y = oy_odom
            msg.info.origin.orientation = q_oi # The map's orientation follows the initial pose
        else:
            # For disk map or no initial pose, origin_x/y are already in odom
            msg.info.origin.position.x = self.origin_x
            msg.info.origin.position.y = self.origin_y
            msg.info.origin.orientation.w = 1.0
        
        msg.info.origin.position.z = 0.0
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)
        
        map_type = "ACTIVE_MAPPING" if self.is_mapping else "DISK_MAP"
        occupied_count = np.sum(self.grid == 100)
        free_count = np.sum(self.grid == 0)
        unknown_count = np.sum(self.grid == -1)
        
        self.get_logger().info(
            f"Published map [{map_type}]: size={self.width}x{self.height}, "
            f"origin=({self.origin_x:.2f}, {self.origin_y:.2f}), "
            f"stats: occupied={occupied_count}, free={free_count}, unknown={unknown_count}"
        )

    def publish_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.odom_frame
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = 0.0  # Force Z to 0
            pose_msg.pose.orientation = trans.transform.rotation

            self.pose_pub.publish(pose_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # self.get_logger().warn(f"TF lookup failed: {e}")
            pass

    def start_mapping_callback(self, request, response):
        self.is_mapping = True
        self.has_disk_map = False # Stop publishing the old disk map
        
        # Reset to default parameters
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Capture current robot pose as the 'initial' frame origin in odom
        try:
            self.initial_pose_in_odom = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            self.get_logger().info(f"Captured initial pose in odom: "
                                   f"x={self.initial_pose_in_odom.transform.translation.x:.2f}, "
                                   f"y={self.initial_pose_in_odom.transform.translation.y:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to capture initial pose: {e}. Mapping will be relative to odom origin.")
            self.initial_pose_in_odom = None

        # In the 'initial' frame, the robot is at (0,0). 
        # We center the map on the robot, so the origin in 'initial' frame is:
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0
        
        # Fresh grid
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        
        # Immediate update
        self.publish_map()
        
        response.success = True
        response.message = "Mapping started, relative to the initial pose (initial frame)."
        self.get_logger().info(response.message)
        return response

    def stop_mapping_callback(self, request, response):
        self.is_mapping = False
        success = self.save_map()
        response.success = success
        if success:
            response.message = "Mapping stopped and map saved."
        else:
            response.message = "Mapping stopped but failed to save map."
        self.get_logger().info(response.message)
        return response

    def set_map_callback(self, request, response):
        # Update internal map from request
        new_map = request.map
        self.get_logger().info("Received new map via /set_map. Overwriting current map.")
        
        # Update parameters
        self.resolution = new_map.info.resolution
        self.width = new_map.info.width
        self.height = new_map.info.height
        self.origin_x = new_map.info.origin.position.x
        self.origin_y = new_map.info.origin.position.y
        
        # Update grid
        self.grid = np.array(new_map.data, dtype=np.int8).reshape((self.height, self.width))
        
        # Save to disk immediately
        self.save_map()
        
        response.success = True
        response.message = "Map updated and saved successfully."
        
        # Publish the updated map for visualization
        self.publish_map()
        
        return response

    def load_map(self):
        try:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            pgm_path = os.path.join(dir_path, "map.pgm")
            yaml_path = os.path.join(dir_path, "map.yaml")

            if not os.path.exists(pgm_path) or not os.path.exists(yaml_path):
                self.get_logger().info("No existing map found on disk.")
                return

            # Load YAML for metadata
            import yaml
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.resolution = data['resolution']
                self.origin_x = data['origin'][0]
                self.origin_y = data['origin'][1]

            # Load PGM using OpenCV
            # PGM: 0 black (occupied), 255 white (free), 205 gray (unknown)
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().error(f"Failed to load image from {pgm_path}")
                return

            self.height, self.width = img.shape
            # ROS Grid is bottom-up, PGM is top-down
            img = np.flipud(img)

            # Convert back to ROS grid values
            self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
            self.grid[img < 10] = 100    # Black -> Occupied
            self.grid[img > 245] = 0     # White -> Free
            self.grid[(img >= 10) & (img <= 245)] = -1 # Gray -> Unknown

            self.has_disk_map = True
            self.get_logger().info(f"Successfully loaded map from {pgm_path} ({self.width}x{self.height})")
        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")

    def save_map(self):
        try:
            # Current directory of the script
            dir_path = os.path.dirname(os.path.realpath(__file__))
            pgm_filename = "map.pgm"
            yaml_filename = "map.yaml"
            pgm_path = os.path.join(dir_path, pgm_filename)
            yaml_path = os.path.join(dir_path, yaml_filename)

            # Save PGM
            pgm_data = np.zeros_like(self.grid, dtype=np.uint8)
            pgm_data[self.grid == 100] = 0
            pgm_data[self.grid == 0] = 255
            pgm_data[self.grid == -1] = 205
            
            # Flip vertically for PGM format
            pgm_data = np.flipud(pgm_data)

            with open(pgm_path, 'wb') as f:
                f.write(f"P5\n{self.width} {self.height}\n255\n".encode())
                f.write(pgm_data.tobytes())

            # Save YAML
            with open(yaml_path, 'w') as f:
                f.write(f"image: {pgm_filename}\n")
                f.write(f"resolution: {self.resolution}\n")
                f.write(f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n")
                f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

            self.get_logger().info(f"Saved map to {pgm_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CloudToMapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
