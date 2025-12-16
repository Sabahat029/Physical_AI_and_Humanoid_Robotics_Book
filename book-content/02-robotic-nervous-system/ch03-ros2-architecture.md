# Chapter 3: ROS 2 Architecture - Nodes, Topics, Services, Actions

**Date**: December 15, 2025  
**Module**: Robotic Nervous System (ROS 2)  
**Chapter**: 3 of 12  
**Estimated Reading Time**: 150 minutes  
**Prerequisites**: Basic Python knowledge, Ubuntu/Linux familiarity, understanding of distributed systems concepts

## Learning Objectives

By the end of this chapter, you will be able to:

1. Master ROS 2 computational graph concepts and architecture principles
2. Design and implement nodes, publishers, subscribers for robotic systems
3. Create services and actions for goal-oriented robot communication
4. Understand and configure Quality of Service (QoS) settings for different use cases
5. Configure launch files and parameter management systems
6. Implement robust communication patterns for real-time robotic applications

---

## 3.1 Introduction to ROS 2 Architecture and Computational Graph

### The ROS 2 Computational Graph

ROS 2 (Robot Operating System 2) represents a fundamental architecture for developing and deploying robotic systems. Unlike traditional monolithic software approaches, ROS 2 employs a distributed computational graph architecture where different software components (nodes) communicate through a message-passing system. This architecture enables the development of complex robotic systems by allowing different functionality to be encapsulated in separate, communicating nodes.

The computational graph in ROS 2 consists of nodes that perform computation and communicate with each other through messages. These nodes can run on the same machine or distributed across multiple machines, allowing for flexible system design. The communication between nodes is managed by the ROS 2 middleware, which handles message routing, discovery, and delivery.

This distributed architecture is particularly well-suited to Physical AI systems, where different types of information processing (perception, planning, control, etc.) can be encapsulated in separate nodes that communicate efficiently. The modular design allows for system components to be developed, tested, and maintained independently, while the communication system ensures that information flows appropriately between components.

### Key Architectural Concepts

The ROS 2 architectural model is built around several key concepts that enable its distributed nature. **Nodes** are the fundamental computational units that perform specific functions and communicate with other nodes. **Messages** are the data structures that carry information between nodes. **Topics** are named buses over which messages are published and subscribed. **Services** provide synchronous request-response communication, while **Actions** provide asynchronous goal-oriented communication.

The ROS 2 architecture also includes **Parameters** for configuration, **Actions** for long-running tasks with feedback, and **Launch files** for starting groups of nodes with specific configurations. These architectural elements work together to provide a flexible and powerful framework for developing robotic systems.

The design of ROS 2 addresses several limitations of the original ROS system, particularly around real-time performance, security, and deployment in production environments. The DDS (Data Distribution Service) middleware provides the foundation for reliable communication, while the client library implementations (rclcpp for C++ and rclpy for Python) provide the interface between application code and the middleware.

### Advantages for Physical AI Systems

ROS 2's architectural design provides several advantages for Physical AI systems. The distributed nature aligns well with the modular sensorimotor processing required by embodied systems. Different sensors can be encapsulated in separate nodes, with their data processed by specialized algorithms that then publish their results to other nodes for integration and decision making.

The real-time performance characteristics of ROS 2 are important for Physical AI systems that require responsive interaction with the environment. The quality of service settings allow for different communication patterns depending on the criticality of the information being communicated. Time-critical control information can be prioritized over less urgent status information.

The modularity of ROS 2 also supports the development of reusable components that can be shared across different Physical AI applications. A perception node developed for one robot can potentially be used with minor modifications on another robot, accelerating development and allowing researchers to build on each other's work.

### Middleware and Client Library Architecture

ROS 2's architecture is built on top of DDS middleware, which provides the underlying communication infrastructure. DDS (Data Distribution Service) is an industry-standard middleware specification that provides publish-subscribe communication patterns with rich quality of service capabilities. This middleware choice provides ROS 2 with enterprise-level performance and reliability.

The ROS 2 architecture includes multiple client libraries that provide language-specific interfaces to the underlying middleware. The most commonly used client libraries are rclcpp for C++ and rclpy for Python, but other languages are supported as well. These client libraries provide the interface between application code and the middleware, handling the complexities of message serialization, network communication, and service discovery.

The layered architecture of ROS 2 separates the application interface (client libraries) from the middleware implementation, allowing for flexibility in middleware selection. This design also allows for the middleware to be updated or replaced while maintaining application compatibility, which is important for long-term system maintenance.

---

## 3.2 Nodes: The Fundamental Computational Units

### Understanding ROS 2 Nodes

A node in ROS 2 is an executable that uses ROS 2 client libraries to communicate with other nodes. Nodes are the fundamental computational units that perform specific functions within a robotic system. Each node typically encapsulates a single functionality, such as sensor data processing, control algorithm implementation, or user interface management.

Nodes in ROS 2 are designed to be lightweight and focused on a single responsibility, following the Unix philosophy of doing one thing well. This design allows for complex systems to be built by connecting multiple simple nodes, each performing its specific function. The communication between nodes is handled by the ROS 2 middleware, allowing nodes to be developed and tested independently.

The lifecycle of a ROS 2 node is managed by the client library, which handles node initialization, parameter management, and cleanup. Nodes can be created with specific names, namespaces, and parameters, allowing for flexible system configuration. The node interface provides methods for creating publishers, subscribers, services, and other communication endpoints.

### Node Implementation Patterns

Implementing ROS 2 nodes requires understanding several design patterns that enable effective communication and coordination. The **publisher-subscriber pattern** is the most common, where nodes publish data to topics and other nodes subscribe to receive this data. This asynchronous communication pattern is well-suited to real-time sensorimotor processing where data is continuously generated and processed.

The **client-server pattern** is implemented through ROS 2 services, where one node provides a service and other nodes request specific operations from that service. This synchronous communication pattern is useful for operations that require a specific response, such as requesting specific robot configurations or triggering specific actions.

The **action pattern** provides asynchronous goal-oriented communication with feedback and status updates. This pattern is useful for long-running operations where the requester needs to know the progress and final result of the operation, such as navigation goals or manipulation tasks.

### Node Lifecycle Management

ROS 2 includes sophisticated lifecycle management capabilities that allow nodes to transition between different states in a controlled manner. The lifecycle states include Unconfigured, Inactive, Active, and Finalized, with well-defined transitions between states. This management system allows for controlled system startup, reconfiguration, and shutdown.

Lifecycle nodes provide several advantages for complex robotic systems. The state-based approach allows for proper initialization of complex systems where some nodes must be initialized before others. The reconfiguration capabilities allow nodes to change their behavior at runtime based on changing operational requirements.

The lifecycle management system also supports fault tolerance by allowing individual nodes to be restarted while the rest of the system continues operating. This capability is important for long-running physical AI systems that must maintain operation even when individual components experience problems.

### Creating Nodes with rclpy

The rclpy client library provides Python interfaces for creating ROS 2 nodes. Creating a basic node involves importing the rclpy library, creating a Node subclass, and implementing the required functionality. The rclpy library handles the underlying communication and middleware interaction, allowing developers to focus on application-specific logic.

```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Node initialized')
```

This simple example demonstrates the basic structure of a ROS 2 node. The node inherits from the Node class provided by rclpy, calls the parent constructor with a node name, and can then create publishers, subscribers, and other communication endpoints.

More complex nodes will typically include publishers, subscribers, timers, and other communication endpoints. The node's constructor will create these elements, and the node will implement callbacks to handle incoming messages and timer events. The node may also include service servers, action servers, and parameter management.

### Node Best Practices

Effective node design follows several best practices that improve system reliability and maintainability. Nodes should have a single, well-defined responsibility and should avoid complex internal state management when possible. This design makes nodes easier to test and more reliable.

Nodes should handle errors gracefully and provide appropriate logging for debugging and monitoring. The ROS 2 logging system provides different log levels and allows for structured logging that supports monitoring and analysis of system behavior.

Nodes should also be designed with configuration in mind. Parameters should be used for configuration values that may change between different deployments or operational modes. This allows nodes to be reused in different contexts with minimal code changes.

---

## 3.3 Topic-Based Communication: Publishers and Subscribers

### The Publish-Subscribe Pattern in ROS 2

The publish-subscribe communication pattern is the backbone of ROS 2's asynchronous communication system. In this pattern, nodes called **publishers** send messages to named **topics**, while nodes called **subscribers** receive messages from topics. This one-to-many communication pattern allows for flexible data distribution where multiple subscribers can receive the same data stream without the publisher needing to know about all subscribers.

The publish-subscribe pattern is particularly well-suited to sensorimotor processing in Physical AI systems. Sensor nodes can continuously publish sensor data without concern for which other nodes are using the data. Multiple processing nodes can subscribe to the same sensor data, each performing different types of processing. This loose coupling between publishers and subscribers enables flexible system design.

The topic-based communication system in ROS 2 provides several advantages for robotic systems. It enables real-time data distribution with predictable performance characteristics. The asynchronous nature of the communication means that slow consumers don't block fast producers, which is important for maintaining system responsiveness in real-time robotic applications.

### Implementation of Publishers and Subscribers

Creating a publisher in ROS 2 involves creating a publisher object within a node, specifying the topic name and message type. The publisher can then send messages by calling its publish method with a message object. The message object must be of the specified type and properly initialized with the data to be sent.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

This example demonstrates a simple publisher that sends a string message periodically. The publisher is created with a topic name ('topic'), a message type (String), and a queue size (10). The timer callback constructs a message, populates it with data, and publishes it to the topic.

Creating a subscriber is similarly straightforward. The subscriber is created with a topic name, message type, and callback function. When messages are received on the topic, the callback function is invoked with the received message.

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

This subscriber example shows how to create a subscription and process incoming messages. The create_subscription method specifies the topic name, message type, callback function, and queue size. The callback function receives the message object and can process its contents.

### Message Types and Serialization

ROS 2 provides a rich set of standard message types for common robotic applications, including sensor_msgs for sensor data, geometry_msgs for geometric information, and nav_msgs for navigation. These standard message types enable interoperability between different robotic systems and allow developers to focus on functionality rather than message format design.

For custom applications, developers can define their own message types using the ROS 2 message definition language (msg files). These custom message types can include primitive types like integers and floats, as well as arrays and nested message types. The ROS 2 build system automatically generates client library code for custom message types, making them easy to use in applications.

Message serialization and deserialization is handled automatically by the ROS 2 client libraries. When a message is published, the client library serializes it into a binary format for transmission. When a message is received, it is automatically deserialized into the appropriate message type for the application to use.

The serialization system supports both CDR (Common Data Representation) and other formats, ensuring compatibility across different platforms and programming languages. The serialization is efficient to minimize network bandwidth and processing time, which is important for real-time robotic applications.

### Quality of Service in Topic Communication

Quality of Service (QoS) settings allow developers to specify the behavior and reliability characteristics of topic-based communication. These settings control aspects such as reliability, durability, liveliness, and history, allowing different topics to be configured for their specific requirements.

The reliability QoS setting determines whether message delivery is best-effort or reliable. Best-effort delivery prioritizes low latency over message delivery, which is appropriate for sensor data where older messages become irrelevant quickly. Reliable delivery ensures that all messages are delivered, which is important for control commands where no messages can be lost.

The history QoS setting controls how many messages are stored for late-joining subscribers. Keep-all history stores all messages, which is useful for configuration parameters that must be available to all nodes. Keep-last history stores only the most recent messages, which is appropriate for streaming sensor data where only recent values are relevant.

The durability QoS setting determines whether messages are stored for nodes that were not active when the messages were sent. Transient-local durability stores messages in a special storage that late-joining nodes can access, which is useful for initial configuration information. Volatile durability does not store messages for late joiners, which is appropriate for streaming data.

These QoS settings are crucial for Physical AI systems where different types of information have different requirements. Sensor data may require best-effort delivery to maintain real-time performance, while configuration data may require reliable delivery to ensure consistent system behavior.

---

## 3.4 Service-Based Communication: Request-Response Patterns

### Understanding Services in ROS 2

Services in ROS 2 provide synchronous request-response communication between nodes, filling the gap left by the asynchronous nature of topic-based communication. A service node provides a specific function that can be requested by other nodes, responding with the result of the requested operation. This pattern is essential for operations that require immediate responses or for operations that change the state of the system.

The service pattern is implemented using service definition files that specify the request and response message types. When a client node wants to use a service, it sends a request message to the service server, which processes the request and returns a response message. The communication is synchronous from the client's perspective, meaning the client waits for the response before continuing execution.

Services are appropriate for operations that are relatively fast and have well-defined inputs and outputs. Examples include requesting robot configuration parameters, triggering specific calibration procedures, or requesting the robot to move to a specific position. The synchronous nature ensures that the client knows immediately whether the operation succeeded and what the result was.

However, services are not appropriate for long-running operations or operations that require ongoing feedback. For these cases, ROS 2 provides actions, which will be discussed later in this chapter. The service pattern is designed for operations that complete relatively quickly and return a single result.

### Service Definition and Implementation

Creating a service in ROS 2 involves defining the service interface using the service definition language (srv files). A service definition includes two parts: the request message format and the response message format. The request format defines what information the client must provide to the service, while the response format defines what information the service will return.

```
# Example service definition (example_service.srv)
string name      # Request: name to look up
---
int64 age       # Response: age if found, -1 if not found
```

This example service definition specifies that the service expects a string request (a name) and will return an integer response (an age). The service definition is placed in an srv directory within a ROS 2 package and is used to automatically generate the client and server code for the service.

Service servers are implemented within nodes and handle incoming service requests. The server must implement a callback function that processes the request and returns the appropriate response. The callback function receives the request message and must return a response message.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

This example shows a simple service server that adds two integers. The service is created with the service type, service name ('add_two_ints'), and callback function. The callback processes the request (adding the two integers) and sets the response value.

### Service Clients and Usage Patterns

Service clients make requests to service servers and wait for the responses. Creating a service client involves creating a client object within a node, specifying the service type and name. The client can then make requests using the service call interface.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

This client example shows how to create a service client and make service requests. The client waits for the service to become available, creates a request object, and then makes the service call asynchronously. The spin_until_future_complete function waits for the response.

Service clients should handle the possibility that service calls may fail due to network issues, server unavailability, or other problems. The service call interface provides mechanisms for handling these failures gracefully, including timeout mechanisms and error reporting.

### Service Design Considerations

Effective service design requires careful consideration of the service interface and behavior. Services should have well-defined, specific functions rather than being general-purpose. A service that "does everything" is difficult to implement reliably and maintain over time. Instead, multiple specific services should be created for different functions.

Services should also be designed to avoid blocking other system operations. If a service call takes too long to complete, it can block the calling thread and potentially affect other system functionality. Long-running operations should be implemented as actions rather than services.

The request and response message design should be clear and efficient. Avoid overly complex message structures that are difficult to implement and maintain. The service interface should be stable over time to maintain compatibility with existing clients.

Security considerations are also important for services, particularly when robots operate in networked environments. Service interfaces should authenticate clients and validate requests to prevent unauthorized access or malformed data.

---

## 3.5 Action-Based Communication: Asynchronous Goal-Oriented Systems

### Understanding Actions in ROS 2

Actions in ROS 2 provide goal-oriented communication for long-running operations that require feedback and status updates. Unlike services, which provide synchronous request-response communication, and topics, which provide asynchronous data streaming, actions bridge these approaches by providing asynchronous operations with ongoing feedback and status information.

An action consists of three message types: goal, feedback, and result. The goal defines what the client wants the action server to do. The feedback provides ongoing status information during the action execution. The result provides the final outcome when the action completes. This three-part structure enables rich interaction patterns where clients can monitor progress and potentially cancel long-running operations.

Actions are essential for Physical AI systems where many operations take significant time to complete. Examples include navigation to distant locations, complex manipulation tasks, or data collection over extended periods. The ability to monitor progress and potentially interrupt these operations is crucial for responsive robotic systems.

The action pattern also supports multiple active goals simultaneously. A single action server can be managing multiple goals from different clients, each in different stages of execution. This capability enables more sophisticated multi-tasking behavior in robotic systems.

### Action Definition and Implementation

Action definitions use a special action definition language (action files) that specify the three message types required for actions. The action definition includes goal, result, and feedback message formats in a single file with the .action extension.

```
# Example action definition (Fibonacci.action)
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

This example action definition creates a Fibonacci sequence generator. The goal includes the order of the sequence to generate, the result includes the complete sequence, and the feedback includes partial results as the sequence is computed.

Action servers implement the action interface within nodes and handle goal requests, feedback updates, and result reporting. The action server must implement callbacks for goal acceptance, goal execution, and cancellation handling.

```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(1)  # Simulate processing time

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))
        return result
```

This example shows a complete action server implementation. The server handles goal requests, cancellation requests, and executes the action while providing feedback. The asynchronous nature allows for proper handling of cancellation and progress updates.

### Action Clients and Interaction Patterns

Action clients interact with action servers by sending goals, receiving feedback, and retrieving results. Unlike service clients, action clients don't wait for immediate responses but instead maintain ongoing communication with the action server throughout the operation.

```python
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
```

This client example shows how to create an action client that sends goals, receives feedback, and retrieves the final result. The client must handle the asynchronous nature of action communication, with callbacks for different stages of the interaction.

Action clients should be designed to handle cancellation gracefully and to monitor action progress appropriately. The feedback mechanism allows clients to provide user interface updates and to make decisions about continuing or canceling the action based on the progress information.

### Advanced Action Concepts

Actions support several advanced features that enable sophisticated robotic applications. Goal preemption allows action servers to abort current goals when new goals are received. This is useful for operations where newer goals supersede older ones, such as navigation where a new destination should replace an in-progress navigation.

Result timeouts provide mechanisms for handling action servers that don't respond appropriately. Clients can specify timeouts for receiving results, and the action system will handle the timeout appropriately. This prevents clients from waiting indefinitely for actions that don't complete properly.

Goal and result persistence allow action servers to continue managing goals and results even across system restarts. This capability is important for long-running operations that might span system maintenance or restarts.

The action interface also supports streaming feedback for very long-running operations. Instead of sending feedback as a single message, servers can stream feedback messages throughout the operation, providing continuous updates on system status.

---

## 3.6 Quality of Service (QoS) Settings and Real-Time Considerations

### Understanding Quality of Service in ROS 2

Quality of Service (QoS) settings in ROS 2 provide mechanisms for controlling the behavior and reliability characteristics of message communication. These settings allow different aspects of communication to be tailored to specific application requirements, enabling ROS 2 systems to meet diverse performance and reliability needs.

QoS settings are particularly important for Physical AI systems where different types of data have different requirements. Control commands may require reliable delivery to ensure safe operation, while sensor data might prioritize low latency over delivery guarantee. The QoS system allows these different requirements to be specified and enforced.

The QoS system in ROS 2 is based on DDS (Data Distribution Service) specifications, providing industry-standard mechanisms for quality of service control. This provides ROS 2 with sophisticated capabilities for managing distributed real-time communication while maintaining compatibility with broader DDS ecosystems.

### Reliability Settings and Applications

The reliability QoS setting controls whether message delivery must be guaranteed or if best-effort delivery is acceptable. **Reliable** delivery ensures that all messages are delivered to subscribers, potentially retransmitting lost messages. **Best effort** delivery attempts to deliver messages but does not guarantee delivery, prioritizing low latency over reliability.

For Physical AI systems, the choice of reliability setting depends on the specific data being transmitted. Control commands and configuration parameters typically require reliable delivery to ensure that critical information is not lost. A navigation command telling a robot to stop must be delivered reliably to prevent safety issues.

Conversely, sensor data often uses best-effort delivery. In a video stream, if a frame is lost, the next frame will arrive soon and provides the most current information. Receiving an old frame late is typically less useful than receiving the current frame quickly.

The reliability setting also affects system performance and resource usage. Reliable delivery requires additional network overhead and processing to track delivery status and potentially retransmit messages. Best-effort delivery has lower overhead but with the corresponding risk of lost messages.

### Durability and History Policies

Durability settings determine how messages are handled for late-joining subscribers. **Volatile** durability means that only messages published after a subscriber joins are available to that subscriber. **Transient local** durability stores messages in a special storage that late-joining subscribers can access to get initial state information.

History policies control how many messages are stored for delivery. **Keep last** stores only the specified number of most recent messages. **Keep all** stores all messages published on the topic. These settings interact with durability settings to control message availability and memory usage.

For static configuration information, transient local durability with keep-all history might be appropriate to ensure that new nodes can receive all configuration data. For streaming sensor data, volatile durability with keep-last history is often more appropriate to limit memory usage while providing recent data to late joiners.

The choice of durability and history settings affects both memory usage and startup behavior. Systems with many topics using keep-all history can consume significant memory, while systems with volatile durability may require special handling for nodes that start after configuration data has been published.

### Deadline and Lifeliness QoS Settings

Deadline settings specify the maximum time interval between consecutive messages on a topic. This setting is important for monitoring systems to detect when data sources stop publishing as expected. If no messages are received within the deadline period, monitoring systems can take appropriate action.

Liveliness settings provide mechanisms for detecting when publishers or subscribers become unavailable. This is important for safety-critical Physical AI systems where the absence of critical data sources should trigger appropriate responses.

The deadline and liveliness settings work together to provide robustness to system failures. A navigation system might use deadlines to detect when a critical sensor stops providing data, and liveliness settings to detect when the sensor node itself becomes unavailable.

These settings are particularly important for Physical AI systems operating in complex environments where individual sensors or processing nodes might temporarily become unavailable due to environmental conditions or mechanical issues.

### Real-Time Performance Considerations

Real-time ROS 2 systems require careful attention to timing and performance characteristics. The QoS settings provide mechanisms for optimizing performance for real-time applications, but they must be chosen appropriately for the specific real-time requirements.

Message queue sizes affect the buffering of messages and can impact real-time performance. Large queues can absorb bursts of messages but may introduce latency. Small queues reduce latency but may result in message loss during bursts. The appropriate queue size depends on the expected message rates and burst characteristics.

The choice of middleware implementation can also affect real-time performance. Different DDS implementations have different performance characteristics, and some are better suited to real-time applications than others. The middleware choice may need to be optimized for specific real-time requirements.

Task scheduling and thread management within ROS 2 nodes also affect real-time performance. Nodes should be designed to avoid blocking operations on critical timing paths and to ensure that high-priority communication is not blocked by lower-priority processing.

---

## 3.7 Launch Files and Parameter Management

### Launch System Overview

The ROS 2 launch system provides mechanisms for starting and configuring groups of nodes as a coordinated system. Launch files specify which nodes to start, their parameters, and the relationships between them. This enables complex systems to be started with a single command and to be reconfigured using different launch files for different operational scenarios.

Launch files use XML or Python syntax to specify node configurations. The XML format provides a declarative approach that is easy to read and modify, while the Python format provides more flexibility for complex startup logic. Both formats support parameter specification, remapping of topics and services, and conditional startup based on runtime conditions.

The launch system also supports composition, where multiple nodes can be run within a single process to improve performance by eliminating inter-process communication overhead. This is particularly useful for nodes that communicate frequently with each other.

Launch files can include other launch files, enabling modular system design where common components are defined in shared files and specific configurations are defined in application-specific files. This modularity supports reuse of common components across different applications.

### Parameter Management in ROS 2

Parameters in ROS 2 provide a mechanism for configuring node behavior without recompiling code. Parameters can be specified at launch time, set during runtime, and saved for future runs. This flexibility allows systems to be configured for different environments and operational requirements without changing code.

ROS 2 parameters support several data types including integers, floats, strings, booleans, and arrays of these types. Parameters can be declared within nodes with default values, descriptions, and constraints. This declaration system enables parameter validation and provides documentation for the available parameters.

Parameter files can store parameter configurations for reuse across different system deployments. These files can be written in YAML format and loaded at launch time, providing a convenient way to manage system configurations.

```yaml
# Example parameter file
publisher_node:
  ros__parameters:
    publish_frequency: 10.0
    message_prefix: "Hello"
    enabled: true
```

This parameter file shows how parameters can be organized by node name, making it easy to manage parameter sets for complex systems with many nodes.

### Advanced Launch Concepts

ROS 2 launch files support several advanced features for complex system management. Conditional launching allows nodes to be started based on command-line arguments or other runtime conditions. This enables single launch files to support multiple operational modes.

Event handling in launch systems allows for sophisticated startup coordination. Nodes can be configured to start only after other nodes become available, or to restart automatically if they crash. This improves system reliability and reduces manual intervention requirements.

Remapping in launch files allows topic and service names to be changed without modifying the node code. This is useful for creating reusable nodes that can operate in different system contexts with different naming conventions.

Security policies can be specified in launch files to control access to system resources. This is important for Physical AI systems that may operate in networked environments where security is a concern.

### Best Practices for Launch and Parameter Management

Effective launch file management follows several best practices that improve system maintainability and reliability. Launch files should be organized hierarchically with common components in shared files and application-specific configurations in separate files.

Parameter naming should follow consistent conventions that make it clear which node a parameter applies to and what function it controls. This prevents naming conflicts and makes parameter configuration more intuitive.

Launch files should include error handling and validation to detect configuration problems during startup rather than during operation. This includes checking for required parameters and validating parameter ranges.

Parameters that control critical safety functions should be carefully managed with appropriate defaults and validation. These parameters may require special handling to prevent unsafe configurations.

---

## Summary

This chapter has provided a comprehensive exploration of the ROS 2 architecture that enables distributed sensorimotor processing essential for Physical AI systems:

1. **Computational graph fundamentals**: Understanding nodes and distributed architecture
2. **Topic-based communication**: Publishers, subscribers, and asynchronous messaging
3. **Service-based communication**: Synchronous request-response patterns
4. **Action-based communication**: Asynchronous goal-oriented systems
5. **Quality of Service settings**: Performance and reliability optimization
6. **Launch and parameter management**: System configuration and deployment

These architectural concepts provide the communication backbone that enables Physical AI systems to distribute processing across multiple nodes while maintaining the real-time performance required for embodied interaction. The modular design enables complex systems to be built from simple, focused components that communicate through standardized interfaces.

The key insight from this chapter is that ROS 2 architecture is specifically designed to support the distributed, real-time communication requirements of Physical AI systems. The different communication patterns (topics, services, actions) are appropriate for different types of sensorimotor processing, and the QoS system enables optimization for specific performance requirements.

## Key Terms

- **Node**: Fundamental computational unit in ROS 2 that communicates with other nodes
- **Topic**: Named bus for asynchronous message publication and subscription
- **Publisher**: Node that sends messages on a topic
- **Subscriber**: Node that receives messages from a topic
- **Service**: Synchronous request-response communication pattern
- **Action**: Asynchronous goal-oriented communication with feedback
- **Quality of Service (QoS)**: Settings that control communication behavior and reliability
- **Launch File**: Configuration file that specifies which nodes to start and how
- **Parameter**: Configurable value that can be set without recompiling code
- **DDS (Data Distribution Service)**: Middleware specification underlying ROS 2 communication

## Further Reading

- Quigley, M., Gerkey, B., & Smart, W. D. (2019). "Programming Robots with ROS: A Practical Introduction to the Robot Operating System." O'Reilly Media.
- ROS 2 Documentation Consortium. (2025). "ROS 2 Documentation." docs.ros.org
- Mettler, B. (2021). "ROS Robot Programming: Industrial and Educational Applications." Springer.
- Macenski, S. (2022). "ROS 2 with C++: A Practical Introduction."

---

**Chapter 4 Preview**: In the next chapter, we will explore how to connect Python-based AI agents to ROS 2 controllers using the rclpy client library, implementing the distributed AI-robot interaction that represents the core of Physical AI systems. We will examine how to integrate AI decision-making with real-time robot control while maintaining the performance and reliability requirements of embodied systems.