---
sidebar_position: 3
sidebar_label: "Ch.3: Sensor Simulation"
title: "Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)"
tags: ["sensors", "lidar", "depth-camera", "imu", "perception", "week-7"]
difficulty: "Intermediate"
module: 2
week: 7
prerequisites: ["Chapter 2", "ROS 2 Humble"]
estimated_time: "4-5 hours"
topics: ["lidar-simulation", "camera-simulation", "imu-simulation", "sensor-noise", "ros2-topics"]
---

# Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)

**Estimated time**: 4-5 hours | **Difficulty**: Intermediate | **Week**: 7, Days 1-3

## Learning Objectives

After completing this chapter, you will be able to:

1. **Explain why sensor simulation is essential** for developing robust perception algorithms
2. **Simulate LiDAR sensors** (2D scans and 3D point clouds) with realistic range and accuracy
3. **Configure RGB and depth cameras** with intrinsic parameters and noise models
4. **Simulate stereo camera systems** with baseline and disparity
5. **Configure IMU sensors** with accelerometer, gyroscope, and magnetometer noise models
6. **Add sensor plugins to robots** in SDF format
7. **Publish sensor data** over ROS 2 topics in standard message formats
8. **Visualize and validate** sensor output using RViz and Python analysis scripts

## Key Concepts

1. **Sensor fidelity**: How accurately simulation matches real-world sensor behavior (noise, range, accuracy)
2. **LiDAR**: Light Detection and Ranging—emits laser pulses, measures reflections to create point clouds
3. **Depth camera**: Captures RGB image + depth map (distance to each pixel)
4. **Stereo vision**: Two cameras with baseline separation compute depth via triangulation
5. **IMU**: Inertial Measurement Unit—measures acceleration and angular velocity
6. **Sensor noise**: Realistic Gaussian noise and bias that match real hardware specifications

---

## Perception Simulation Fundamentals

### Why Sensor Simulation Matters

Real sensors have imperfections:
- **Noise**: Random fluctuations (Gaussian noise)
- **Bias**: Systematic error (e.g., IMU always reads 0.1m/s² high)
- **Range limits**: Sensors work within min/max distances
- **Field of view (FOV)**: Limited angle of perception
- **Resolution**: Discrete measurements (not continuous)

Without simulating these, algorithms trained in perfect simulation fail on real hardware.

### Noise Modeling

Realistic sensor noise follows a **Gaussian distribution**:

```
measurement = true_value + noise
noise ~ N(mean=0, std_dev=σ)
```

Example: LiDAR with σ=0.01m adds ±0.01m random error to each measurement.

---

## LiDAR Simulation in Gazebo

### 2D LiDAR (Scanning Laser)

```xml
<!-- File: humanoid_with_sensors.sdf (in humanoid head link) -->
<sensor name="laser_scanner" type="ray">
  <pose>0 0 0.3 0 0 0</pose>  <!-- On head, facing forward -->
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>  <!-- 360 points per scan -->
        <resolution>1.0</resolution>  <!-- One per degree -->
        <min_angle>0</min_angle>
        <max_angle>6.283</max_angle>  <!-- 2π radians = 360° -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>  <!-- Minimum range: 10cm -->
      <max>30.0</max>  <!-- Maximum range: 30m -->
      <resolution>0.01</resolution>  <!-- 1cm resolution -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.05</stddev>  <!-- ±5cm noise -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>30</update_rate>  <!-- 30 Hz -->
  <visualize>true</visualize>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>laser_link</frame_name>
  </plugin>
</sensor>
```

**Verify in ROS 2**:
```bash
ros2 topic echo /scan
# Output: angles, ranges, intensities
```

### 3D LiDAR (Point Cloud)

```xml
<!-- Velodyne-style 3D LiDAR -->
<sensor name="lidar_3d" type="ray">
  <pose>0 0 0.5 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>400</samples>  <!-- 400 points per scan -->
        <min_angle>0</min_angle>
        <max_angle>6.283</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16 vertical lines -->
        <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.1</stddev>  <!-- ±10cm noise -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>  <!-- 10 Hz (matches Velodyne) -->
  <visualize>true</visualize>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <remapping>~/out:=points</remapping>
    </ros>
    <output_type>sensor_msgs/PointCloud2</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

---

## Camera Simulation

### RGB Camera

```xml
<sensor name="rgb_camera" type="camera">
  <pose>0 0 0.15 0 0 0</pose>  <!-- On head -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.01</near>
      <far>100.0</far>
    </clip>
    <!-- Intrinsic parameters (camera matrix) -->
    <lens>
      <intrinsics>
        <fx>320.0</fx>  <!-- Focal length X -->
        <fy>320.0</fy>  <!-- Focal length Y -->
        <cx>320.0</cx>  <!-- Principal point X -->
        <cy>240.0</cy>  <!-- Principal point Y -->
        <s>0.0</s>      <!-- Skew -->
      </intrinsics>
    </lens>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>false</visualize>
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>image_raw:=rgb/image_raw</remapping>
      <remapping>camera_info:=rgb/camera_info</remapping>
    </ros>
    <camera_name>rgb_camera</camera_name>
    <frame_name>camera_frame</frame_name>
  </plugin>
</sensor>
```

### Depth Camera (RealSense D435-style)

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0 0 0.15 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>   <!-- 10cm minimum range -->
      <far>10.0</far>    <!-- 10m maximum range -->
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>false</visualize>
  <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>image_raw:=depth/image_raw</remapping>
      <remapping>camera_info:=depth/camera_info</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_frame</frame_name>
  </plugin>
</sensor>
```

---

## IMU Sensor Simulation

### IMU with Realistic Noise

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.3 0 0 0</pose>  <!-- Torso -->
  <always_on>1</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz -->
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>  <!-- White noise -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00001</bias_stddev>  <!-- Bias drift -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>  <!-- Larger noise for acceleration -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>imu:=imu/data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

---

## Sensor Fusion: Combining Multiple Sensors

**Concept**: Use multiple sensor modalities together to improve robustness.

Example: **LiDAR + Depth Camera**
- LiDAR is good for long range but 2D
- Depth camera is good for close-range 3D but limited range
- Together: Far obstacles (LiDAR) + detailed near objects (depth)

```python
# File: src/sensor_fusion_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to both sensors
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)

        # Publish fused obstacle map
        self.map_pub = self.create_publisher(PointCloud2, '/obstacle_map', 10)

    def lidar_callback(self, msg):
        # Process LiDAR scan
        pass

    def depth_callback(self, msg):
        # Process depth image
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Validating Simulated Sensors

### Comparison to Real Hardware Specs

RealSense D435 specifications:
- Depth range: 0.1m - 10m
- Resolution: 640×480
- FOV: 87° × 58°
- Noise: ~1-2% of distance

RealSense D435 simulation (should match above):
- `<near>0.1</near>`, `<far>10.0</far>`
- `<width>640</width>`, `<height>480</height>`
- `<horizontal_fov>1.52</horizontal_fov>` (87°)
- `<stddev>0.01</stddev>` (1% noise)

### Validation Script

```python
# File: src/validate_sensors.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import numpy as np

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')
        self.lidar_sub = self.create_subscription(PointCloud2, '/points', self.validate_lidar, 1)
        self.stats = {'count': 0, 'mean_range': 0, 'std_range': 0}

    def validate_lidar(self, msg):
        # Extract point coordinates from PointCloud2
        points = np.array([(p[0], p[1], p[2]) for p in msg.data])
        ranges = np.linalg.norm(points, axis=1)

        self.stats['count'] += 1
        self.stats['mean_range'] = np.mean(ranges)
        self.stats['std_range'] = np.std(ranges)

        self.get_logger().info(f"LiDAR: {len(ranges)} points, "
                              f"mean range: {self.stats['mean_range']:.2f}m, "
                              f"std: {self.stats['std_range']:.4f}m")

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidator()
    rclpy.spin(node)
```

---

## Hands-On Labs

### Lab 1: Simulate LiDAR and Visualize Point Clouds

**Objective**: Add 3D LiDAR to humanoid, view point clouds in RViz.

**Steps**:
1. Add 3D LiDAR sensor (from section above) to your humanoid world file
2. Launch Gazebo: `gazebo humanoid_with_lidar.sdf`
3. In another terminal, view point cloud:
   ```bash
   rviz2
   # Add PointCloud2 display, subscribe to /points
   ```

**Verification**: Point cloud appears in RViz, moves with humanoid

### Lab 2: Depth Camera with Noise

**Objective**: Configure depth camera with realistic noise, compare noisy vs. ideal.

**Steps**:
1. Add depth camera plugin (from section above)
2. View depth image in RViz:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   # Select /humanoid_robot/depth/image_raw
   ```
3. Measure noise: Run `validate_sensors.py` and check std deviation

### Lab 3: IMU Noise Modeling

**Objective**: Record IMU data while robot is stationary, analyze noise characteristics.

**Steps**:
1. Add IMU plugin to humanoid
2. Record 10 seconds of data:
   ```bash
   ros2 bag record /humanoid_robot/imu/data
   ```
3. Analyze with Python:
   ```bash
   # Plot acceleration noise distribution
   python3 src/analyze_imu.py
   ```

---

## End-of-Chapter Exercises

### Exercise 1: LiDAR Navigation

**Task**: Humanoid navigates maze using simulated LiDAR. Detect obstacles at 2m, 5m, 10m.

### Exercise 2: Depth Camera Noise Analysis

**Task**: Configure depth camera with 3 noise levels (0.005m, 0.01m, 0.05m). Measure detection accuracy.

### Exercise 3: IMU Calibration

**Task**: Collect stationary IMU data, calculate bias and noise covariance.

### Exercise 4: Sensor Fusion Implementation

**Task**: Fuse LiDAR and depth camera into combined 3D obstacle map.

---

## Summary

✅ **LiDAR simulation**: 2D scans and 3D point clouds with noise

✅ **Camera simulation**: RGB and depth cameras with intrinsic parameters

✅ **IMU simulation**: Realistic acceleration and angular velocity with noise

✅ **Sensor noise**: Gaussian models matching real hardware

✅ **ROS 2 topics**: Standard message formats for all sensors

✅ **Validation**: Compare simulated to real sensor specs

---

## Next Steps

[Chapter 4: High-Fidelity Simulation in Unity](./chapter-4-unity-high-fidelity-simulation.md) covers rendering, dataset export, and high-fidelity visualization.

**Estimated time**: 4-5 hours

Ready? Head to Chapter 4!
