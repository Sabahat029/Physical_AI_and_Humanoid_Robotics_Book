# ROS 2 Node Creation Skill

This skill provides the capability to quickly create ROS 2 nodes with standard patterns and best practices.

## Purpose
- Standardize ROS 2 node creation across the Physical AI system
- Provide templates for common node types (publisher, subscriber, service, action)
- Include boilerplate code for logging, parameters, and lifecycle management

## Inputs
- Node name (string)
- Node type (publisher, subscriber, service_server, service_client, action_server, action_client)
- Message types (for publishers/subscribers)
- Service/action types (for services/actions)

## Outputs
- Complete ROS 2 node code in Python
- Standardized structure following ROS 2 best practices
- Proper error handling and logging

## Implementation

```python
import rclpy
from rclpy.node import Node

class NodeCreationSkill:
    def create_basic_node(self, node_name, node_content=""):
        """
        Creates a basic ROS 2 node structure
        """
        node_code = f'''import rclpy
from rclpy.node import Node

class {node_name.title().replace(" ", "")}Node(Node):
    def __init__(self):
        super().__init__('{node_name.lower().replace(" ", "_")}')
        self.get_logger().info("Node initialized")
        
{node_content}

def main(args=None):
    rclpy.init(args=args)
    node = {node_name.title().replace(" ", "")}Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        return node_code

    def create_publisher_node(self, node_name, topic_name, msg_type, msg_content=""):
        """
        Creates a ROS 2 publisher node
        """
        content = f'''        self.publisher_ = self.create_publisher({msg_type}, '{topic_name}', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
{msg_content}
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1'''
        return self.create_basic_node(node_name, content)

    def create_subscriber_node(self, node_name, topic_name, msg_type, callback_content=""):
        """
        Creates a ROS 2 subscriber node
        """
        content = f'''        self.subscription = self.create_subscription(
            {msg_type},
            '{topic_name}',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
{callback_content}
        self.get_logger().info('I heard: "%s"' % msg.data)'''
        return self.create_basic_node(node_name, content)

# Example usage
if __name__ == '__main__':
    skill = NodeCreationSkill()
    
    # Create a publisher node
    pub_node_code = skill.create_publisher_node(
        "publisher_node", 
        "topic", 
        "String",
        "        msg = String()\n        msg.data = 'Hello World: %d' % self.i"
    )
    
    print(pub_node_code)
```

## Usage Examples

### Creating a sensor data publisher
```
skill = NodeCreationSkill()
sensor_pub = skill.create_publisher_node(
    "lidar_publisher",
    "/sensors/lidar_scan",
    "LaserScan",
    "        msg = LaserScan()\n        # populate lidar data\n        msg.ranges = [1.0, 2.0, 3.0]"
)
```

### Creating a command subscriber
```
command_sub = skill.create_subscriber_node(
    "motor_command_subscriber",
    "/motor/commands",
    "JointState",
    "        # Process motor commands\n        self.execute_command(msg)"
)
```

## Integration Notes
- This skill should be integrated with code generation tools
- Output should be validated for ROS 2 standards compliance
- Generated code should include proper error handling and documentation
- Consider integration with launch file generation for complete node deployment