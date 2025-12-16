# Perception Subagent

This subagent handles perception tasks for Physical AI systems, integrating multiple sensors to create a coherent understanding of the environment.

## Purpose
- Process and integrate data from multiple sensors (LiDAR, cameras, IMU, etc.)
- Create environmental models and detect objects
- Provide perception outputs for higher-level planning and control
- Handle sensor calibration and validation

## Inputs
- Sensor data streams (LiDAR, camera images, IMU, etc.)
- Calibration parameters
- Environmental context information

## Outputs
- Detected objects with poses
- Environmental map
- Sensor quality metrics
- Perception confidence scores

## Architecture

The Perception Subagent follows a modular architecture:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Sensor        │    │  Preprocessing   │    │  Processing     │
│   Interface     │───▶│  & Calibration   │───▶│  Pipeline       │
│                 │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │                        │
                              ▼                        ▼
                    ┌──────────────────┐    ┌─────────────────┐
                    │  Data Fusion     │    │  Object & Map   │
                    │  & Integration   │───▶│  Generation     │
                    └──────────────────┘    └─────────────────┘
                              │                        │
                              ▼                        ▼
                    ┌─────────────────────────────────────────┐
                    │           Output Generation           │
                    │  (Detected Objects, Maps, Metrics)    │
                    └─────────────────────────────────────────┘
```

## Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2
from cv_bridge import CvBridge
from dataclasses import dataclass
from typing import List, Dict, Any
import threading
import queue

@dataclass
class PerceptionObject:
    """Data structure for detected objects"""
    id: str
    type: str
    position: np.ndarray  # 3D position [x, y, z]
    orientation: np.ndarray  # quaternion [x, y, z, w]
    size: np.ndarray  # 3D size [width, height, depth]
    confidence: float
    timestamp: float

@dataclass
class EnvironmentalMap:
    """Data structure for environmental representation"""
    occupancy_grid: np.ndarray  # 2D occupancy grid
    point_cloud: np.ndarray  # 3D point cloud
    semantic_labels: np.ndarray  # Semantic segmentation
    timestamp: float

class PerceptionSubagent(Node):
    def __init__(self, node_name='perception_subagent'):
        super().__init__(node_name)
        
        # Initialize components
        self.cv_bridge = CvBridge()
        self.object_queue = queue.Queue(maxsize=10)
        self.map_queue = queue.Queue(maxsize=2)
        
        # Configuration
        self.lidar_config = {
            'min_range': 0.1,
            'max_range': 25.0,
            'cluster_distance': 0.5,
            'ground_removal_height': 0.1
        }
        
        self.camera_config = {
            'detection_model': 'yolo',  # or 'ssd', 'rcnn'
            'confidence_threshold': 0.5,
            'class_names': ['person', 'obstacle', 'object']
        }
        
        # Publishers
        self.object_pub = self.create_publisher(
            MarkerArray, 
            '/perception/detected_objects', 
            10
        )
        self.map_pub = self.create_publisher(
            PointCloud2, 
            '/perception/environment_map', 
            10
        )
        self.quality_pub = self.create_publisher(
            Float32, 
            '/perception/quality_score', 
            10
        )
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/sensors/lidar_scan',
            self.lidar_callback,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/rgb_image',
            self.camera_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu',
            self.imu_callback,
            10
        )
        
        # Processing timers
        self.perception_timer = self.create_timer(
            0.1,  # 10 Hz processing
            self.perception_processing_callback
        )
        
        self.get_logger().info("Perception Subagent initialized")

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        try:
            # Convert ranges to points
            ranges = np.array(msg.ranges)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
            
            # Filter valid ranges
            valid_mask = (ranges >= self.lidar_config['min_range']) & \
                        (ranges <= self.lidar_config['max_range']) & \
                        (np.isfinite(ranges))
            
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]
            
            # Convert to Cartesian coordinates
            x = valid_ranges * np.cos(valid_angles)
            y = valid_ranges * np.sin(valid_angles)
            points = np.column_stack((x, y, np.zeros_like(x)))  # 2D scan in 3D space
            
            # Remove ground points
            ground_mask = points[:, 2] > self.lidar_config['ground_removal_height']
            filtered_points = points[ground_mask]
            
            # Cluster points to find objects
            objects = self.cluster_lidar_points(filtered_points)
            
            # Store for processing
            self.lidar_data = {
                'points': filtered_points,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'objects': objects
            }
            
        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {e}")

    def camera_callback(self, msg):
        """Process incoming camera data"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Object detection (simplified - would use actual detection model)
            objects = self.detect_objects_in_image(cv_image)
            
            # Store for processing
            self.camera_data = {
                'image': cv_image,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'objects': objects
            }
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera data: {e}")

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        try:
            # Extract IMU information
            self.imu_data = {
                'orientation': np.array([
                    msg.orientation.x, msg.orientation.y, 
                    msg.orientation.z, msg.orientation.w
                ]),
                'angular_velocity': np.array([
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                ]),
                'linear_acceleration': np.array([
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                ]),
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")

    def cluster_lidar_points(self, points, max_distance=0.5):
        """Cluster LiDAR points to identify objects"""
        if len(points) == 0:
            return []
        
        # Simple clustering algorithm (DBSCAN-like)
        labels = np.full(len(points), -1, dtype=int)
        cluster_id = 0
        
        for i, point in enumerate(points):
            if labels[i] != -1:
                continue  # Already assigned
                
            # Find neighbors within max_distance
            distances = np.sqrt(np.sum((points - point)**2, axis=1))
            neighbors = np.where(distances < max_distance)[0]
            
            if len(neighbors) > 5:  # Minimum points for object
                labels[neighbors] = cluster_id
                cluster_id += 1
        
        # Create objects from clusters
        objects = []
        for cluster_id in range(np.max(labels) + 1):
            if cluster_id == -1:
                continue
                
            cluster_points = points[labels == cluster_id]
            if len(cluster_points) > 0:
                centroid = np.mean(cluster_points, axis=0)
                size = np.ptp(cluster_points, axis=0)  # Peak-to-peak (max - min)
                
                obj = PerceptionObject(
                    id=f"obj_{cluster_id}",
                    type="obstacle",  # Would be classified in real implementation
                    position=centroid,
                    orientation=np.array([0.0, 0.0, 0.0, 1.0]),  # Identity quaternion
                    size=size,
                    confidence=0.8,  # Would be computed from cluster properties
                    timestamp=self.get_clock().now().nanoseconds * 1e-9
                )
                objects.append(obj)
        
        return objects

    def detect_objects_in_image(self, image):
        """Detect objects in camera image (simplified implementation)"""
        # This is a simplified implementation
        # Real implementation would use a trained object detection model
        
        # For demonstration, we'll create some mock detections
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect edges as a very basic form of object detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for i, contour in enumerate(contours[:5]):  # Limit to first 5 contours
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                centroid = np.array([x + w/2, y + h/2, 0.0])  # Add depth placeholder
                
                obj = PerceptionObject(
                    id=f"cam_obj_{i}",
                    type="object",  # Would be classified in real implementation
                    position=centroid,
                    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                    size=np.array([w, h, 0.1]),  # Add depth placeholder
                    confidence=0.7,
                    timestamp=self.get_clock().now().nanoseconds * 1e-9
                )
                objects.append(obj)
        
        return objects

    def data_fusion(self):
        """Fuse sensor data from different modalities"""
        if not all(hasattr(self, attr) for attr in ['lidar_data', 'camera_data', 'imu_data']):
            return None
            
        # Simple fusion: combine LiDAR objects with camera objects
        # In reality, this would use more sophisticated techniques like Kalman filtering
        # or deep learning-based fusion
        
        fused_objects = []
        
        # Add LiDAR objects
        for obj in self.lidar_data['objects']:
            fused_objects.append(obj)
        
        # Add camera objects (would need to project to 3D space in real implementation)
        for obj in self.camera_data['objects']:
            # For now, we'll just add with some confidence adjustment
            obj.confidence *= 0.9  # Camera objects might be less reliable for distance
            fused_objects.append(obj)
        
        return fused_objects

    def create_environmental_map(self, objects, points):
        """Create environmental representation from sensor data"""
        # Create a simple occupancy grid
        grid_size = 20  # 20x20 meters
        resolution = 0.1  # 10cm resolution
        grid_width = int(grid_size / resolution)
        
        occupancy_grid = np.zeros((grid_width, grid_width), dtype=np.uint8)
        
        # Mark objects in grid
        for obj in objects:
            grid_x = int((obj.position[0] + grid_size/2) / resolution)
            grid_y = int((obj.position[1] + grid_size/2) / resolution)
            
            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_width:
                occupancy_grid[grid_x, grid_y] = 100  # Mark as occupied
        
        env_map = EnvironmentalMap(
            occupancy_grid=occupancy_grid,
            point_cloud=points,
            semantic_labels=np.zeros(len(points), dtype=np.uint8),
            timestamp=self.get_clock().now().nanoseconds * 1e-9
        )
        
        return env_map

    def perception_processing_callback(self):
        """Main perception processing loop"""
        try:
            # Check if we have all necessary data
            if not all(hasattr(self, attr) for attr in ['lidar_data', 'camera_data', 'imu_data']):
                self.get_logger().debug("Waiting for all sensor data...")
                return
            
            # Fuse sensor data
            fused_objects = self.data_fusion()
            if not fused_objects:
                self.get_logger().debug("No fused objects detected")
                return
            
            # Create environmental map
            all_points = self.lidar_data['points'] if hasattr(self, 'lidar_data') else np.array([])
            env_map = self.create_environmental_map(fused_objects, all_points)
            
            # Calculate quality score based on sensor data availability and object detection
            quality_score = min(1.0, len(fused_objects) * 0.1 + 0.5)
            
            # Publish results
            self.publish_objects(fused_objects)
            self.publish_environmental_map(env_map)
            self.publish_quality_score(quality_score)
            
            self.get_logger().info(f"Perception update: {len(fused_objects)} objects detected")
            
        except Exception as e:
            self.get_logger().error(f"Error in perception processing: {e}")

    def publish_objects(self, objects):
        """Publish detected objects as visualization markers"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "perception"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = obj.position[0]
            marker.pose.position.y = obj.position[1]
            marker.pose.position.z = obj.position[2]
            
            # Orientation
            marker.pose.orientation.x = obj.orientation[0]
            marker.pose.orientation.y = obj.orientation[1]
            marker.pose.orientation.z = obj.orientation[2]
            marker.pose.orientation.w = obj.orientation[3]
            
            # Scale (size)
            marker.scale.x = obj.size[0] if obj.size[0] > 0.1 else 0.5
            marker.scale.y = obj.size[1] if obj.size[1] > 0.1 else 0.5
            marker.scale.z = obj.size[2] if obj.size[2] > 0.1 else 0.5
            
            # Color based on confidence
            marker.color.r = 1.0 - obj.confidence
            marker.color.g = obj.confidence
            marker.color.b = 0.0
            marker.color.a = 0.8  # Alpha
            
            marker_array.markers.append(marker)
        
        self.object_pub.publish(marker_array)

    def publish_environmental_map(self, env_map):
        """Publish environmental map"""
        # For now, just log the map update
        self.get_logger().debug(f"Environmental map updated with {len(env_map.point_cloud)} points")

    def publish_quality_score(self, quality_score):
        """Publish perception quality score"""
        score_msg = Float32()
        score_msg.data = quality_score
        self.quality_pub.publish(score_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_agent = PerceptionSubagent()
    
    try:
        rclpy.spin(perception_agent)
    except KeyboardInterrupt:
        pass
    finally:
        perception_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration and Deployment

### Launch file example
```
# perception_subagent.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='physical_ai_perception',
            executable='perception_subagent',
            name='perception_subagent',
            parameters=[
                {'lidar_config.min_range': 0.1},
                {'lidar_config.max_range': 25.0},
                {'camera_config.confidence_threshold': 0.5},
            ],
            remappings=[
                ('/sensors/lidar_scan', '/lidar/scan'),
                ('/sensors/rgb_image', '/camera/image_raw'),
            ]
        )
    ])
```

## Integration Notes
- The subagent should be integrated with the main robot control system
- Sensor topics should be remapped to match actual robot configuration
- Performance monitoring should track processing latency and resource usage
- The subagent should handle sensor failures gracefully with fallback modes