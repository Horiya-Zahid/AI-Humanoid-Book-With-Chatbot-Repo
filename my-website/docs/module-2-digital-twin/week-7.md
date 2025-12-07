---
sidebar_position: 2
---

# Week 7: Gazebo and Unity Integration with ROS 2

## Learning Objectives
- Master Gazebo simulation with ROS 2 integration
- Understand Unity robotics simulation capabilities
- Learn to create and configure simulation environments
- Implement sensor simulation and physics modeling
- Bridge simulation to real hardware using ROS 2

## Gazebo Simulation with ROS 2

Gazebo provides a powerful 3D simulation environment that integrates seamlessly with ROS 2. The integration is facilitated through Gazebo ROS packages that provide bridges between Gazebo's native APIs and ROS 2 topics and services.

### Gazebo ROS 2 Bridge Components:
- **gazebo_ros_pkgs**: Core ROS 2 packages for Gazebo integration
- **gazebo_ros2_control**: ROS 2 control interface for Gazebo
- **gazebo_ros2_ignition**: Bridge between Gazebo and ROS 2

## Setting Up Gazebo with ROS 2

### Installation and Dependencies:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros2-control-plugins
```

### Basic Gazebo Launch:
```xml
<launch>
  <!-- Start Gazebo server -->
  <node name="gazebo_server"
        pkg="gazebo_ros"
        exec="gzserver"
        args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"/>

  <!-- Start Gazebo client -->
  <node name="gazebo_client"
        pkg="gazebo_ros"
        exec="gzclient"
        output="screen"/>
</launch>
```

## Creating Gazebo Worlds

Gazebo worlds define the environment in which robots operate:

```xml
<sdf version="1.7">
  <world name="default">
    <!-- Include models from Gazebo model database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom environment -->
    <model name="my_environment">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 2 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Gazebo Sensors and Physics

### Sensor Simulation:
Gazebo provides realistic simulation of various sensors:

```xml
<sensor name="camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>800</width>
      <height>600</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_link</frame_name>
    <topic_name>image_raw</topic_name>
  </plugin>
</sensor>
```

### Physics Configuration:
```xml
<physics name="ode" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

## Unity Robotics Simulation

Unity provides an alternative simulation environment with high-fidelity graphics and physics:

### Unity Robotics Hub Components:
- **Unity Robot System (URS)**: Framework for robot simulation
- **ML-Agents**: Machine learning for robot training
- **ROS#**: ROS communication bridge
- **Physics Engine**: NVIDIA PhysX for accurate physics simulation

### Unity-ROS 2 Integration:
Unity integrates with ROS 2 through the ROS TCP Connector:

```csharp
using ROS2;

public class UnityRobotController : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Socket ros2Socket;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Unity.ROS2ServerURL = "127.0.0.1";
        ros2Unity.ROS2ServerPort = 8888;
        ros2Unity.Initialize();

        ros2Socket = ros2Unity.Ros2CreateSocket();
    }

    void Update()
    {
        // Publish sensor data to ROS 2
        var sensorData = GetSensorData();
        ros2Socket.Publish("sensor_data", sensorData);
    }
}
```

## Creating Unity Simulation Environments

Unity excels at creating photorealistic environments:

### Environment Design:
- **High-fidelity assets**: Realistic textures and materials
- **Lighting systems**: Dynamic lighting with shadows
- **Particle systems**: Environmental effects
- **Audio simulation**: Sound propagation and effects

### Procedural Generation:
Unity allows for procedural environment generation to create diverse testing scenarios:

```csharp
public class EnvironmentGenerator : MonoBehaviour
{
    public GameObject[] obstaclePrefabs;
    public int numberOfObstacles = 10;

    void Start()
    {
        GenerateEnvironment();
    }

    void GenerateEnvironment()
    {
        for(int i = 0; i < numberOfObstacles; i++)
        {
            Vector3 position = new Vector3(
                Random.Range(-10f, 10f),
                0.5f,
                Random.Range(-10f, 10f)
            );

            GameObject obstacle = Instantiate(
                obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)],
                position,
                Quaternion.identity
            );
        }
    }
}
```

## Sensor Simulation in Unity

Unity provides advanced sensor simulation capabilities:

### Camera Simulation:
```csharp
[RequireComponent(typeof(Camera))]
public class ROS2CameraPublisher : MonoBehaviour
{
    private Camera cam;
    private ROS2UnityComponent ros2;

    void Start()
    {
        cam = GetComponent<Camera>();
        ros2 = GetComponent<ROS2UnityComponent>();
        ros2.Initialize();
    }

    void Update()
    {
        // Capture image and publish to ROS 2
        Texture2D image = CaptureImage();
        // Convert and publish to ROS 2 topic
    }
}
```

## Physics Simulation Comparison

### Gazebo Physics:
- **ODE**: Open Dynamics Engine - good for basic physics
- **Bullet**: More advanced physics simulation
- **DART**: Dynamic Animation and Robotics Toolkit

### Unity Physics:
- **NVIDIA PhysX**: Industry-standard physics engine
- **Custom solutions**: Can integrate other physics engines
- **GPU acceleration**: Physics computation on GPU

## Sim-to-Real Transfer Strategies

### Domain Randomization:
```python
# In simulation, randomize parameters to improve robustness
class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization')
        self.randomize_parameters()

    def randomize_parameters(self):
        # Randomize friction coefficients
        friction_min, friction_max = 0.1, 1.0
        random_friction = random.uniform(friction_min, friction_max)

        # Randomize lighting conditions
        light_intensity = random.uniform(0.5, 2.0)

        # Apply randomizations to simulation
        self.apply_randomizations({
            'friction': random_friction,
            'lighting': light_intensity
        })
```

### System Identification:
Process of determining real-world parameters through experimentation and matching simulation behavior.

## Performance Optimization

### Gazebo Optimization:
- **Simplify collision meshes**: Use simpler meshes for collision detection
- **Reduce physics update rate**: Adjust based on required accuracy
- **Limit sensor update rates**: Match real sensor capabilities

### Unity Optimization:
- **LOD (Level of Detail)**: Reduce detail at distance
- **Occlusion culling**: Don't render hidden objects
- **Texture compression**: Optimize texture sizes

## Best Practices for Simulation

### 1. Model Validation:
Always validate simulation models against real-world data.

### 2. Progressive Fidelity:
Start with simple models and increase complexity as needed.

### 3. Scenario Testing:
Test in diverse simulated environments before real-world deployment.

### 4. Performance Monitoring:
Monitor simulation performance to ensure real-time operation.

## Integration Patterns

### 1. Twin Operation Mode:
Run simulation and real robot in parallel, comparing behaviors.

### 2. Shadow Mode:
Use simulation for prediction while controlling real robot.

### 3. Failover Mode:
Switch to simulation when real robot encounters issues.

## Summary

This week explored the integration of Gazebo and Unity with ROS 2 for digital twin applications. We covered the setup and configuration of both simulation environments, sensor simulation, physics modeling, and strategies for effective sim-to-real transfer. Understanding these tools is crucial for developing robust robotics applications that can be safely tested and validated in virtual environments.

## Self-Check Quiz

1. What are the main differences between Gazebo and Unity for robotics simulation?
2. How do you integrate Gazebo with ROS 2?
3. Explain domain randomization and its purpose in robotics.
4. What are the key considerations for effective sim-to-real transfer?