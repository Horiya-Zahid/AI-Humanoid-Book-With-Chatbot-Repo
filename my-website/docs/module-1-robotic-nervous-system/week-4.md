---
sidebar_position: 4
---

# Week 4: URDF and Xacro for Humanoid Robot Description

## Learning Objectives
- Understand URDF (Unified Robot Description Format)
- Master Xacro for complex robot descriptions
- Create humanoid robot models with joints and links
- Implement robot kinematics and dynamics
- Work with robot sensors and transmissions

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used to describe robots in ROS. It defines the physical and visual properties of a robot, including links, joints, and materials.

### URDF Elements:
- **Links**: Rigid parts of the robot
- **Joints**: Connections between links
- **Visual**: How the link appears in simulation
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Structure

Here's a simple URDF example:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Child link connected by a joint -->
  <link name="child_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting the links -->
  <joint name="base_to_child" type="continuous">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Xacro: XML Macros for Robots

Xacro is a macro language that extends URDF with features like variables, math expressions, and includes. It makes complex robot descriptions more manageable.

### Xacro Advantages:
- **Variables**: Define values once and reuse
- **Math expressions**: Calculate values dynamically
- **Includes**: Reuse parts from other files
- **Macros**: Define reusable robot components

## Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.08" />

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix *joint_pose">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <xacro:insert_block name="joint_pose"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Use the wheel macro -->
  <xacro:wheel prefix="front_left">
    <origin xyz="0.2 0.15 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
  </xacro:wheel>

</robot>
```

## Humanoid Robot Modeling

Humanoid robots require special considerations in URDF/Xacro:

### Key Components:
- **Torso**: Main body of the robot
- **Head**: With sensors like cameras
- **Arms**: With multiple joints for manipulation
- **Legs**: With joints for locomotion
- **Hands/Feet**: For interaction with the environment

### Joint Types for Humanoids:
- **Revolute**: Rotational joints (like elbows, knees)
- **Continuous**: Continuously rotating joints (like shoulders)
- **Prismatic**: Linear joints (less common in humanoids)
- **Fixed**: Non-moving connections

## Robot Kinematics

Kinematics describes the motion of the robot without considering forces:

### Forward Kinematics:
- Calculate end-effector position from joint angles

### Inverse Kinematics:
- Calculate joint angles needed to reach a desired position

## Robot Dynamics

Dynamics considers the forces and torques that cause motion:

### Key Properties:
- **Mass**: Inertial properties of links
- **Center of Mass**: Balance point of each link
- **Inertia Tensor**: Resistance to rotational motion
- **Joint Limits**: Physical constraints on movement

## Sensors in URDF

Sensors can be included in URDF for simulation:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Summary

This week covered URDF and Xacro for describing humanoid robots. We explored the structure of robot descriptions, how to use Xacro macros for complex models, and how to include kinematic, dynamic, and sensor properties. These tools are essential for creating realistic robot models for simulation and control.

## Self-Check Quiz

1. What is the difference between URDF and Xacro?
2. Name the three main elements of a URDF robot description.
3. How do you define a joint in URDF?
4. What are the advantages of using Xacro over plain URDF?