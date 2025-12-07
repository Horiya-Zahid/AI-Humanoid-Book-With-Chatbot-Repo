---
sidebar_position: 3
---

# Week 3: Mastering rclpy - Python API for ROS 2

## Learning Objectives
- Master the rclpy Python API for ROS 2
- Create publishers and subscribers in Python
- Implement services and clients
- Work with parameters and logging
- Understand node lifecycle management

## Introduction to rclpy

rclpy is the Python client library for ROS 2 that provides Python bindings for the ROS 2 client library (rcl). It allows Python developers to create ROS 2 nodes and interact with the ROS 2 ecosystem.

### Core Concepts:
- **Node**: The basic execution unit in ROS 2
- **Executor**: Manages the execution of callbacks
- **Context**: Encapsulates the ROS client library state

## Creating a Basic Node with rclpy

Let's examine the structure of a basic ROS 2 node using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Publishers
Publishers send messages to topics. They are created using the `create_publisher()` method:

```python
publisher = node.create_publisher(MessageType, 'topic_name', qos_profile)
```

### Subscribers
Subscribers receive messages from topics. They are created using the `create_subscription()` method:

```python
subscriber = node.create_subscription(
    MessageType,
    'topic_name',
    callback_function,
    qos_profile
)
```

## Services and Clients

### Service Servers
Service servers provide services that clients can call:

```python
service = node.create_service(RequestType, 'service_name', callback)
```

### Service Clients
Service clients call services provided by servers:

```python
client = node.create_client(RequestType, 'service_name')
```

## Working with Parameters

Parameters in ROS 2 allow nodes to be configured at runtime:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')

        # Get parameter value
        param_value = self.get_parameter('param_name').value
```

## Quality of Service (QoS) Profiles

QoS profiles control how messages are delivered between publishers and subscribers:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)
```

## Logging in ROS 2

ROS 2 provides a logging interface through the node:

```python
node.get_logger().debug('Debug message')
node.get_logger().info('Info message')
node.get_logger().warn('Warning message')
node.get_logger().error('Error message')
node.get_logger().fatal('Fatal message')
```

## Node Lifecycle

ROS 2 nodes can have lifecycle states (unconfigured, inactive, active, finalized) for complex systems:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState

class LifecycleNodeExample(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring node...')
        return TransitionCallbackReturn.SUCCESS
```

## Summary

This week focused on mastering the rclpy Python API for ROS 2. We explored how to create nodes, publishers, subscribers, services, and clients. We also covered parameters, QoS profiles, logging, and lifecycle management. These skills are essential for developing robust ROS 2 applications in Python.

## Self-Check Quiz

1. What is the purpose of rclpy in ROS 2?
2. How do you create a publisher and subscriber in rclpy?
3. What are QoS profiles and why are they important?
4. How do parameters enhance node configuration?