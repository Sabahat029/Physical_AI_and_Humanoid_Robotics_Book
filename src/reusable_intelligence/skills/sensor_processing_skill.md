# Sensor Processing Skill

This skill provides standardized methods for processing various sensor data types commonly used in Physical AI and humanoid robotics systems.

## Purpose
- Standardize sensor data processing pipelines
- Provide consistent interfaces for different sensor types
- Enable rapid prototyping of sensor fusion applications
- Handle common sensor preprocessing tasks (filtering, calibration, etc.)

## Inputs
- Sensor data (various types: LaserScan, Image, Imu, PointCloud2, etc.)
- Configuration parameters (filter settings, calibration parameters)
- Processing pipeline specification

## Outputs
- Processed sensor data
- Quality metrics and confidence measures
- Error flags for sensor issues

## Implementation

```python
import numpy as np
from sensor_msgs.msg import LaserScan, Image, PointCloud2, Imu
from cv_bridge import CvBridge
import open3d as o3d

class SensorProcessingSkill:
    def __init__(self):
        self.cv_bridge = CvBridge()
        
    def process_lidar_scan(self, scan_msg, config=None):
        """
        Process LiDAR scan data with optional filtering
        """
        if config is None:
            config = {
                'min_range': 0.1,
                'max_range': 10.0,
                'remove_ground': True,
                'cluster_distance': 0.3
            }
        
        # Filter ranges based on min/max
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Remove invalid ranges
        valid_mask = (ranges >= config['min_range']) & (ranges <= config['max_range'])
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        points = np.column_stack((x, y))
        
        # Optional ground plane removal (simplified)
        if config.get('remove_ground', False):
            # Basic ground removal - filter points with low z values
            points = points[np.abs(points[:, 1]) > 0.1]  # Remove points near ground level
        
        return {
            'points': points,
            'ranges': valid_ranges,
            'angles': valid_angles,
            'quality_score': len(points) / len(ranges) if len(ranges) > 0 else 0
        }
    
    def process_camera_image(self, image_msg, config=None):
        """
        Process camera image with optional preprocessing
        """
        if config is None:
            config = {
                'convert_to_cv': True,
                'normalize': True,
                'apply_filters': False
            }
        
        # Convert ROS image to OpenCV
        if config['convert_to_cv']:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        
        # Normalize if needed
        if config['normalize']:
            if cv_image.dtype == np.uint8:
                normalized_img = cv_image.astype(np.float32) / 255.0
            else:
                normalized_img = cv_image
        
        # Optional filtering
        if config.get('apply_filters', False):
            # Apply Gaussian blur as example
            import cv2
            processed_img = cv2.GaussianBlur(normalized_img, (5, 5), 0)
        else:
            processed_img = normalized_img
        
        return {
            'image': processed_img,
            'original_encoding': image_msg.encoding,
            'width': image_msg.width,
            'height': image_msg.height,
            'quality_score': 1.0  # Assuming good image quality
        }
    
    def process_imu_data(self, imu_msg, config=None):
        """
        Process IMU data with optional filtering
        """
        if config is None:
            config = {
                'filter_orientation': True,
                'filter_angular_velocity': True,
                'filter_linear_acceleration': True
            }
        
        # Extract IMU components
        orientation = np.array([
            imu_msg.orientation.x,
            imu_msg.orientation.y, 
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ])
        
        angular_velocity = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        
        linear_acceleration = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        
        processed_data = {
            'orientation': orientation,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }
        
        # Apply optional filtering
        if config.get('filter_orientation', False):
            # Implement orientation filtering if needed
            pass
            
        if config.get('filter_angular_velocity', False):
            # Implement angular velocity filtering if needed
            pass
            
        if config.get('filter_linear_acceleration', False):
            # Implement linear acceleration filtering if needed  
            pass
        
        return {
            'data': processed_data,
            'timestamp': imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9,
            'quality_score': 1.0
        }
    
    def sensor_fusion(self, sensor_data_list, fusion_method='simple_average'):
        """
        Fuse data from multiple sensors
        """
        if fusion_method == 'simple_average':
            # Simple averaging fusion (example for similar data types)
            if len(sensor_data_list) > 0 and all('points' in data for data in sensor_data_list):
                # Example fusion for point cloud data
                all_points = []
                for data in sensor_data_list:
                    all_points.extend(data['points'])
                
                return np.mean(all_points, axis=0) if len(all_points) > 0 else np.array([])
        
        elif fusion_method == 'kalman_filter':
            # Placeholder for Kalman filter implementation
            pass
            
        elif fusion_method == 'particle_filter':
            # Placeholder for particle filter implementation
            pass
        
        return None

# Example usage
if __name__ == '__main__':
    skill = SensorProcessingSkill()
    
    # Example processing would require actual ROS messages
    # This is a demonstration of the interface
    pass
```

## Usage Examples

### Processing LiDAR data
```
skill = SensorProcessingSkill()
lidar_config = {
    'min_range': 0.3,
    'max_range': 20.0,
    'remove_ground': True,
    'cluster_distance': 0.5
}
processed_lidar = skill.process_lidar_scan(ros_lidar_msg, lidar_config)
```

### Processing camera data
```
camera_config = {
    'convert_to_cv': True,
    'normalize': True,
    'apply_filters': True
}
processed_image = skill.process_camera_image(ros_image_msg, camera_config)
```

### Combining multiple sensors
```
sensor_list = [lidar_data, camera_data, imu_data]
fused_data = skill.sensor_fusion(sensor_list, fusion_method='simple_average')
```

## Integration Notes
- This skill should be integrated with ROS 2 topic subscribers
- Consider integration with message filters for time synchronization
- Performance optimization may be needed for real-time applications
- Quality scoring helps with sensor validation and error handling