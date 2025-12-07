---
sidebar_position: 2
---

# Week 2: Foundations of ROS 2 Architecture

## Learning Objectives
- Understand the fundamental architecture of ROS 2
- Learn about nodes, topics, services, and actions
- Explore the DDS (Data Distribution Service) middleware
- Implement basic ROS 2 concepts

## ROS 2 Architecture Overview

ROS 2 is built on a distributed architecture that enables communication between different software components called "nodes". Unlike ROS 1, which relied on a central master node, ROS 2 uses a peer-to-peer discovery mechanism.

### Key Architecture Components:
- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication with feedback

## Nodes in ROS 2

A node is the fundamental unit of execution in ROS 2. Each node is a process that performs computation and communicates with other nodes.

### Node Characteristics:
- Each node has a unique name within the ROS graph
- Nodes can publish to topics, subscribe to topics, provide services, or call services
- Multiple nodes can run on the same machine or across different machines

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics enable asynchronous communication between nodes through a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from the topic.

### Topic Communication:
- **Publisher**: Sends messages to a topic
- **Subscriber**: Receives messages from a topic
- **Message Types**: Strongly typed data structures defined in `.msg` files

## Services and Request/Response

Services provide synchronous communication between nodes. A client sends a request and waits for a response from a server.

### Service Communication:
- **Service Server**: Provides a service by processing requests
- **Service Client**: Calls a service by sending requests
- **Service Types**: Defined in `.srv` files with request and response structures

## Actions for Long-Running Tasks

Actions are designed for long-running tasks that require feedback and the ability to cancel. They include goal, feedback, and result concepts.

### Action Communication:
- **Goal**: Request for a long-running task
- **Feedback**: Periodic updates on task progress
- **Result**: Final outcome of the task

## DDS Middleware

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. DDS provides:
- **Discovery**: Automatic discovery of nodes and topics
- **Quality of Service (QoS)**: Configurable reliability and performance settings
- **Platform independence**: Support for multiple DDS implementations

## Summary

This week covered the fundamental architecture of ROS 2, including nodes, topics, services, and actions. We explored how these components work together to enable distributed robotic systems. Understanding these concepts is crucial for developing complex robotic applications.

## Self-Check Quiz

1. What are the differences between nodes, topics, services, and actions?
2. How does ROS 2's architecture differ from ROS 1?
3. What role does DDS play in ROS 2?