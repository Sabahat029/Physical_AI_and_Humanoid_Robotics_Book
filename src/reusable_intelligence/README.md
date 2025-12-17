# Reusable Intelligence Components for Physical AI & Humanoid Robotics

This directory contains reusable components (Skills, Subagents, and Tools) for Physical AI and Humanoid Robotics systems, designed to accelerate development and ensure consistency across projects.

## Directory Structure

```
reusable_intelligence/
├── skills/           # Reusable skill modules
├── subagents/        # Autonomous subagent implementations  
├── tools/            # Utility and development tools
└── README.md         # This file
```

## Skills

Skills represent focused functional capabilities that can be composed to create complex behaviors.

### Available Skills

1. **ROS 2 Node Creation Skill** (`skills/ros2_node_creator_skill.md`)
   - Provides templates and utilities for creating ROS 2 nodes
   - Supports publisher, subscriber, service, and action node types
   - Includes best practices for structure, error handling, and logging

2. **Sensor Processing Skill** (`skills/sensor_processing_skill.md`)
   - Standardized methods for processing various sensor data types
   - Support for LiDAR, camera, IMU, and other common sensor types
   - Sensor fusion capabilities and quality scoring

## Subagents

Subagents are autonomous modules that handle specific domains of robot functionality with their own decision-making capabilities.

### Available Subagents

1. **Perception Subagent** (`subagents/perception_subagent.md`)
   - Integrates multiple sensors for environmental understanding
   - Detects objects and creates environmental maps
   - Provides perception outputs for planning and control

2. **Control Subagent** (`subagents/control_subagent.md`)
   - Handles low-level robot control and trajectory execution
   - Manages motor commands and safety systems
   - Ensures smooth and safe motion execution

## Tools

Tools are utilities that support development, debugging, and maintenance of Physical AI systems.

### Available Tools

1. **URDF Validation Tool** (`tools/urdf_validation_tool.md`)
   - Validates URDF files for syntax and completeness
   - Checks robot definitions for physical plausibility
   - Provides quality scores and improvement recommendations

2. **Robot System Monitor Tool** (`tools/system_monitor_tool.md`)
   - Monitors system health and performance metrics
   - Tracks resource usage and operational status
   - Generates reports and alerts for system issues

## Usage Guidelines

### For Skills
- Import and instantiate skills as needed in your application code
- Configure parameters based on your specific robot and task requirements  
- Use skills as building blocks to compose complex behaviors
- Extend skills with additional functionality as needed

### For Subagents
- Each subagent runs as an independent ROS 2 node
- Configure subagents through parameters and topic remappings
- Connect subagents using ROS 2 messaging patterns
- Monitor subagent status through diagnostic topics

### For Tools
- Command-line tools can be run independently for analysis
- API-based tools can be integrated into development workflows
- Tools support multiple output formats for integration with other systems
- Use tools as part of CI/CD pipelines for quality assurance

## Integration with Physical AI Book

These components complement the concepts and implementations described in the Physical AI and Humanoid Robotics book by providing:

- **Practical Implementations**: Concrete code examples for the theoretical concepts
- **Best Practices**: Standardized approaches tested in real applications
- **Modular Architecture**: Components that can be combined and reused
- **Development Acceleration**: Reduce development time for common robot functions

## Example Integration

```python
# Example: Using the Sensor Processing Skill
from skills.sensor_processing_skill import SensorProcessingSkill

skill = SensorProcessingSkill()

# Process LiDAR data
lidar_config = {
    'min_range': 0.3,
    'max_range': 20.0,
    'remove_ground': True
}
processed_lidar = skill.process_lidar_scan(ros_lidar_msg, lidar_config)

# Process camera data
camera_config = {
    'convert_to_cv': True,
    'normalize': True
}
processed_image = skill.process_camera_image(ros_image_msg, camera_config)

# Combine sensor data
fused_data = skill.sensor_fusion([processed_lidar, processed_image])
```

## Development and Extension

To extend these components:

1. Follow the same documentation and code structure patterns
2. Ensure proper testing and validation of new components
3. Update this README when adding new components
4. Maintain backward compatibility where possible
5. Document configuration parameters and usage patterns

## License

These reusable intelligence components are provided as part of the Physical AI and Humanoid Robotics educational materials. See the main project license for usage terms.