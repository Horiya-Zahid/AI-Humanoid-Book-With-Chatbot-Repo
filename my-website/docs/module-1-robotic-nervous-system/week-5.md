---
sidebar_position: 5
---

# Week 5: Launch Files, Parameters, and ROS 2 Bags

## Learning Objectives
- Master ROS 2 launch files for system orchestration
- Understand parameter management and configuration
- Work with ROS 2 bags for data recording and playback
- Implement complex robot system startup
- Learn best practices for system deployment

## Introduction to Launch Files

Launch files in ROS 2 allow you to start multiple nodes with a single command, configure parameters, and manage the complete robot system. They replace the roslaunch system from ROS 1.

### Launch File Benefits:
- **System orchestration**: Start multiple nodes simultaneously
- **Parameter configuration**: Set parameters for all nodes
- **Conditional execution**: Start nodes based on conditions
- **Composable nodes**: Run multiple nodes in the same process

## Python Launch Files

ROS 2 uses Python launch files that provide full programming capabilities:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='turtlesim',
            executable='draw_square',
            name='draw_square',
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
```

## YAML Launch Files

For simpler configurations, you can also use YAML launch files:

```yaml
launch:
  - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "turtle_sim"
      parameters:
        - "turtlesim.yaml"
      remappings:
        - ["turtle1/cmd_vel", "cmd_vel"]
```

## Parameter Management

Parameters in ROS 2 can be managed in several ways:

### 1. Node-level parameters:
```python
node.declare_parameter('param_name', default_value)
param_value = node.get_parameter('param_name').value
```

### 2. Launch file parameters:
```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[
        {'param1': 'value1'},
        '/path/to/params.yaml'
    ]
)
```

### 3. Parameter files (YAML):
```yaml
my_node:
  ros__parameters:
    param1: 10
    param2: "string_value"
    param3: [1.0, 2.0, 3.0]
```

## ROS 2 Bags - Data Recording and Playback

ROS 2 bags are used to record and playback topic data for testing, analysis, and debugging.

### Recording Data:
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /topic1 /topic2

# Record with custom storage options
ros2 bag record -o my_bag_folder /topic1 /topic2
```

### Playing Back Data:
```bash
# Play back a recorded bag
ros2 bag play my_bag_folder

# Play with specific options
ros2 bag play --rate 0.5 my_bag_folder  # Play at half speed
ros2 bag play --clock 100 my_bag_folder  # Publish to /clock topic
```

### Inspecting Bags:
```bash
# List topics in a bag
ros2 bag info my_bag_folder

# Convert to CSV
ros2 bag convert -i my_bag_folder -o csv_output
```

## Advanced Launch Concepts

### Composable Nodes:
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                remappings=[('image', 'camera/image_raw'),
                           ('camera_info', 'camera/camera_info'),
                           ('image_rect', 'camera/image_rect')]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

### Conditional Launch:
```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

use_gui = LaunchConfiguration('use_gui')

Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    condition=IfCondition(use_gui)
)
```

## Quality of Service (QoS) in Launch Files

QoS settings can be configured in launch files:

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[
        {'qos_override.publisher.depth': 10},
        {'qos_override.subscriber.reliability': 'reliable'}
    ]
)
```

## Working with Different DDS Implementations

ROS 2 supports multiple DDS implementations (Fast DDS, Cyclone DDS, RTI Connext):

```python
# In launch file, you can set environment variables
from launch.actions import SetEnvironmentVariable

LaunchDescription([
    SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
    # ... other launch actions
])
```

## Best Practices for Launch Files

### 1. Modular Design:
Create reusable launch files that can be included in other launch files.

### 2. Parameter Validation:
Validate parameters at startup to catch configuration errors early.

### 3. Error Handling:
Use appropriate launch conditions and actions to handle startup errors.

### 4. Documentation:
Document launch arguments and their expected values.

## System Deployment Strategies

### 1. Machine-specific configurations:
Use launch arguments to adapt to different hardware configurations.

### 2. Environment variables:
Use environment variables for system-wide settings.

### 3. Configuration files:
Store complex configurations in external YAML files.

## Summary

This week covered essential ROS 2 system management tools: launch files for orchestrating complex robot systems, parameter management for configuration, and ROS 2 bags for data recording and analysis. These tools are crucial for deploying and operating real robotic systems in production environments.

## Self-Check Quiz

1. What are the advantages of using launch files over starting nodes individually?
2. How do you pass parameters to nodes in launch files?
3. What are the main use cases for ROS 2 bags?
4. Explain the difference between composable and regular nodes.