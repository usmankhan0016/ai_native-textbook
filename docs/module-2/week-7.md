---
sidebar_position: 6
sidebar_label: "Week 7: Sensors & Unity"
title: "Week 7 Practice Guide: Sensors & Unity"
tags: ["practice", "sensors", "unity", "week-7", "hands-on"]
difficulty: "Advanced"
module: 2
week: 7
prerequisites: ["Week 6", "Chapters 3-4"]
estimated_time: "18-20 hours"
topics: ["sensor-simulation", "unity-robotics", "ros2-communication", "rendering", "dataset-export"]
---

# Week 7 Practice Guide: Sensors & Unity

**Week 7 (5 days of study) | 18-20 hours total | Sensors in Gazebo + Unity high-fidelity rendering**

Welcome to Week 7! This week builds on your Week 6 Gazebo experience by adding **realistic perception sensors** and transitioning to **high-fidelity rendering in Unity**. By Friday, you'll have a complete Digital Twin combining physics accuracy (Gazebo) and visual realism (Unity).

## Week 7 Objectives

By the end of this week, you will:

1. ✅ Add and configure LiDAR, depth camera, and IMU sensors to your humanoid
2. ✅ Understand sensor noise models and their impact on algorithms
3. ✅ Publish sensor data over ROS 2 topics in standard formats
4. ✅ Visualize sensor data (point clouds, depth images, IMU) in RViz
5. ✅ Set up Unity for robotics with ROS 2 integration
6. ✅ Import humanoid model into Unity with materials and physics
7. ✅ Create photorealistic environments suitable for training data generation

## Daily Learning Path

```
Monday        Tuesday       Wednesday     Thursday       Friday
│             │              │             │              │
Sensors →  LiDAR + → RViz Visualization → Unity Setup → High-Fidelity
in Gazebo   Depth        & Sensor Fusion    & Import      Rendering
│             │              │             │              │
1.5 hours    1.5 hours      2 hours       2 hours       2.5 hours
```

---

## Monday: Introduction to Sensor Simulation (1.5 hours)

### Morning: Understand Sensor Simulation Fundamentals

**Task**: Review why sensor simulation matters and what realism looks like.

Key concepts:
- **Sensor fidelity**: How accurately simulation matches real sensors
- **Noise models**: Gaussian noise that matches real hardware
- **Range limits**: Minimum and maximum sensor ranges
- **Field of view**: Angular coverage of the sensor

**Real-world example**: RealSense D435 depth camera
- Range: 0.1m to 10m
- Noise: ~1-2% of distance
- Resolution: 640×480 pixels
- FOV: 87° × 58°

Our simulation should match these specs!

### Afternoon: Add Basic Sensors to Humanoid

**Task**: Create a new world with humanoid + camera + IMU sensors.

Create file `humanoid_with_sensors.sdf` by copying your Week 6 world and adding sensors to the humanoid link.

**Add camera sensor**:
```xml
<!-- Inside humanoid link in head area -->
<sensor name="rgb_camera" type="camera">
  <pose>0 0 0.15 0 0 0</pose>
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
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>false</visualize>
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>image_raw:=camera/image_raw</remapping>
    </ros>
    <camera_name>front_camera</camera_name>
    <frame_name>camera_frame</frame_name>
  </plugin>
</sensor>
```

**Add IMU sensor**:
```xml
<!-- Inside humanoid link in torso area -->
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.001</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>imu:=imu/data</remapping>
    </ros>
  </plugin>
</sensor>
```

**Launch and verify**:
```bash
gazebo humanoid_with_sensors.sdf

# In another terminal:
ros2 topic list | grep humanoid_robot
# Should see: /humanoid_robot/camera/image_raw, /humanoid_robot/imu/data
```

**Checkpoint**: Both sensors publish data without errors

---

## Tuesday: LiDAR and Depth Cameras (1.5 hours)

### Morning: Add 3D LiDAR Sensor

**Task**: Configure a 3D LiDAR (point cloud) sensor on the humanoid head.

Edit `humanoid_with_sensors.sdf`, add to humanoid head link:

```xml
<sensor name="lidar_3d" type="ray">
  <pose>0 0 0.3 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>400</samples>
        <min_angle>0</min_angle>
        <max_angle>6.283</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <min_angle>-0.2618</min_angle>
        <max_angle>0.2618</max_angle>
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
      <stddev>0.1</stddev>  <!-- 10cm noise -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <remapping>~/out:=points</remapping>
    </ros>
    <output_type>sensor_msgs/PointCloud2</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>
```

### Afternoon: Add Depth Camera

**Task**: Add a RealSense D435-style depth camera.

Edit `humanoid_with_sensors.sdf`, add to humanoid head link:

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0.05 0 0.15 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>false</visualize>
  <plugin name="depth_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>humanoid_robot</namespace>
      <remapping>image_raw:=depth/image_raw</remapping>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_frame</frame_name>
  </plugin>
</sensor>
```

**Launch and verify**:
```bash
gazebo humanoid_with_sensors.sdf

# Check topics
ros2 topic list | grep -E "depth|lidar|points"

# Visualize point cloud (next section)
```

**Checkpoint**: LiDAR and depth camera topics appear in `ros2 topic list`

---

## Wednesday: RViz Visualization and Sensor Fusion (2 hours)

### Morning: Visualize Sensors in RViz

**Task**: Set up RViz to display all sensor outputs.

**Launch RViz**:
```bash
rviz2

# In RViz:
# 1. File → Open Config → select default config
# 2. Add displays:
#    a) PointCloud2: subscribe to /points (from LiDAR)
#    b) Image: subscribe to /camera/image_raw (from RGB camera)
#    c) Image: subscribe to /depth/image_raw (from depth camera)
#    d) IMU: subscribe to /imu/data
# 3. Set Fixed Frame to "base_link"
```

**Observe**:
- LiDAR point cloud should appear as dots around humanoid
- RGB camera image should show rendered view from humanoid head
- Depth image should show grayscale (brighter = closer)
- IMU should show acceleration vectors

### Afternoon: Analyze Sensor Data

**Task**: Write Python script to analyze sensor output and understand noise.

Create `analyze_sensors.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class SensorAnalyzer(Node):
    def __init__(self):
        super().__init__('sensor_analyzer')
        self.lidar_sub = self.create_subscription(PointCloud2, '/points', self.analyze_lidar, 1)
        self.stats = {'count': 0, 'ranges': []}

    def analyze_lidar(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)))
        if len(points) > 0:
            ranges = np.linalg.norm(points, axis=1)
            self.get_logger().info(f'Points: {len(points)}, Range: '
                                  f'min={np.min(ranges):.2f}m, '
                                  f'max={np.max(ranges):.2f}m, '
                                  f'mean={np.mean(ranges):.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = SensorAnalyzer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Run**:
```bash
python3 analyze_sensors.py
# See statistics on LiDAR data
```

**Checkpoint**: You understand sensor data format and can analyze it

---

## Thursday: Set Up Unity and Import Humanoid (2 hours)

### Morning: Install and Configure Unity

**Task**: Install Unity 2022 LTS and set up robotics packages.

1. Download [Unity Hub](https://unity.com/download)
2. Install Unity 2022 LTS version
3. Create new 3D project
4. Install ROS2ForUnity:
   ```bash
   cd <unity_project>/Assets
   git clone https://github.com/ROS2ForUnity/ROS2-Unity.git
   ```
5. In Unity Editor, set up project:
   - Window → TextMesh Pro → Import TMP Essentials
   - Edit → Project Settings → Player → .NET Framework 4.7.1

### Afternoon: Import Humanoid and Test Communication

**Task**: Import humanoid URDF into Unity and verify ROS 2 communication.

1. **Add ROS2 Node component**:
   - Create empty GameObject "ROS2Manager"
   - Add component "ROS2Node"
   - Configure ROS Domain ID

2. **Create joint state publisher script**:
   ```csharp
   // Assets/Scripts/JointStatePublisher.cs
   using UnityEngine;
   using ROS2;
   using sensor_msgs.msg;

   public class JointStatePublisher : MonoBehaviour
   {
       private ROS2Node ros2Node;
       private IPublisher<JointState> pub;
       private Animator anim;

       void Start()
       {
           ros2Node = GetComponent<ROS2Node>();
           pub = ros2Node.CreatePublisher<JointState>("joint_states");
           anim = GetComponent<Animator>();
       }

       void Update()
       {
           var msg = new JointState();
           msg.Header.Stamp = ROS2.Clock.Now;
           msg.Name = new string[] { "left_hip_x", "right_hip_x" };
           msg.Position = new double[] { anim.GetFloat("LeftHip"), anim.GetFloat("RightHip") };
           pub.Publish(msg);
       }
   }
   ```

3. **Test communication**:
   ```bash
   # Terminal 1: Start Unity simulation
   # In Unity: Play button

   # Terminal 2: Monitor ROS 2
   ros2 topic echo /joint_states
   # Should see joint state messages from Unity
   ```

**Checkpoint**: Unity publishes joint states that appear in ROS 2

---

## Friday: High-Fidelity Rendering and Dataset Export (2.5 hours)

### Morning: Configure HDRP for Photorealism

**Task**: Switch to High-Definition Render Pipeline and configure realistic materials.

1. **Install HDRP**:
   - Window → Asset Store
   - Search "High Definition RP"
   - Import latest version

2. **Create HDRP scene**:
   - Assets → Rendering → High Definition Render Pipeline Asset
   - Set as default in Project Settings

3. **Configure materials**:
   ```csharp
   // Assets/Scripts/MaterialSetup.cs
   using UnityEngine;
   using UnityEngine.Rendering.HighDefinition;

   public class MaterialSetup : MonoBehaviour
   {
       void Start()
       {
           Renderer rend = GetComponent<Renderer>();
           Material mat = rend.material;

           // PBR material properties
           mat.SetColor("_BaseColor", new Color(0.5f, 0.5f, 0.5f));
           mat.SetFloat("_Metallic", 0.3f);   // 0=non-metal, 1=metal
           mat.SetFloat("_Smoothness", 0.7f); // 0=rough, 1=glossy
       }
   }
   ```

4. **Set up lighting**:
   - Create Directional Light (sun)
   - Adjust intensity and rotation for dramatic shadows
   - Add environment skybox for reflections

### Afternoon: Export Training Dataset

**Task**: Capture 100 frames with annotations for training data.

```csharp
// Assets/Scripts/DatasetExporter.cs
using UnityEngine;
using System.IO;
using System.Text;

public class DatasetExporter : MonoBehaviour
{
    public Camera captureCamera;
    public int frameCount = 100;
    public string outputPath = "Assets/Dataset/";

    void Start()
    {
        if (!Directory.Exists(outputPath))
            Directory.CreateDirectory(outputPath);
        StartCoroutine(ExportFrames());
    }

    System.Collections.IEnumerator ExportFrames()
    {
        for (int i = 0; i < frameCount; i++)
        {
            // Capture screenshot
            Texture2D tex = ScreenCapture.CaptureScreenshotAsTexture();
            byte[] imageBytes = tex.EncodeToPNG();
            string imagePath = $"{outputPath}/frame_{i:04d}.png";
            File.WriteAllBytes(imagePath, imageBytes);

            // Generate annotation
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("{");
            sb.AppendLine($"  \"frame_id\": {i},");
            sb.AppendLine($"  \"timestamp\": {Time.realtimeSinceStartup},");
            sb.AppendLine($"  \"humanoid_position\": [{gameObject.transform.position.x}, "
                        + $"{gameObject.transform.position.y}, "
                        + $"{gameObject.transform.position.z}]");
            sb.AppendLine("}");

            string annoPath = $"{outputPath}/frame_{i:04d}_annotation.json";
            File.WriteAllText(annoPath, sb.ToString());

            yield return new WaitForEndOfFrame();
        }

        Debug.Log($"Exported {frameCount} frames to {outputPath}");
    }
}
```

**Verify export**:
```bash
ls -la Assets/Dataset/ | wc -l
# Should show 200 files (100 PNG + 100 JSON)
```

**Checkpoint**: Training dataset successfully exported

---

## Week 7 Exercises (5 independent exercises, 1-2 hours each)

### Exercise 1: Configure LiDAR with Varying Noise

**Task**: Create 3 worlds with different LiDAR noise levels (0.05m, 0.1m, 0.2m). Measure detection accuracy at 10m range.

**Acceptance Criteria**:
- All 3 worlds launch successfully
- You measure/document noise impact on obstacle detection accuracy
- Document findings in a report

---

### Exercise 2: Depth Camera Accuracy Analysis

**Task**: Place objects at known distances (1m, 3m, 5m) and measure depth camera accuracy.

**Acceptance Criteria**:
- Capture depth images at 3 distances
- Analyze depth values (should match known distances ±5%)
- Explain any measurement errors

---

### Exercise 3: IMU Calibration

**Task**: Record 30 seconds of stationary IMU data and calculate bias and noise covariance.

**Acceptance Criteria**:
- Record IMU data to file
- Calculate: mean, std dev, bias for gyro and accelerometer
- Report values match real IMU specifications

---

### Exercise 4: Unity ROS 2 Two-Way Communication

**Task**: Implement command feedback loop:
1. Send joint commands from ROS 2 node
2. Unity receives and applies commands
3. Unity publishes back updated joint states

**Acceptance Criteria**:
- Commands sent from ROS 2 cause humanoid movement in Unity
- Updated states published back to ROS 2
- Zero message delays or errors

---

### Exercise 5: Sensor Fusion Implementation

**Task**: Combine LiDAR + depth camera data to create unified 3D obstacle map.

**Acceptance Criteria**:
- Read both sensor topics in ROS 2 node
- Merge point clouds into single coordinate frame
- Publish combined obstacle map
- Visualize in RViz (should see merged data)

---

## Week 7 Challenge Projects (2-3 hours each, optional)

### Challenge 1: Sensor-Based Obstacle Avoidance

**Objective**: Navigate humanoid through obstacle course using LiDAR and depth camera.

**Implementation**:
1. Create world with 5+ obstacles
2. Process LiDAR/depth data to detect obstacles
3. Simple algorithm: move forward if no obstacle, rotate if blocked
4. Success: complete course without crashes

---

### Challenge 2: High-Fidelity Rendering with Domain Randomization

**Objective**: Export 500-frame dataset with randomized lighting and materials.

**Implementation**:
1. Create Unity scene with humanoid and objects
2. Randomize lighting (sun intensity, color, direction)
3. Randomize materials (colors, roughness, metallic)
4. Export dataset with automated domain randomization
5. Validate annotations are consistent across variations

---

## Debugging Tips & Tricks

### Issue: ROS 2 topics not appearing from Gazebo sensors

**Solution**:
- Check gazebo_ros plugins are installed: `apt list --installed | grep gazebo-ros`
- Verify sensor `<plugin>` tags are correct in SDF
- Check ROS_DOMAIN_ID environment variable matches

### Issue: Unity ROS 2 bridge connection fails

**Solution**:
- Verify ROS 2 domain IDs match (Gazebo and Unity)
- Check `source /opt/ros/humble/setup.bash` before launching
- Ensure network allows localhost communication

### Issue: Point cloud visualization is slow in RViz

**Solution**:
- Reduce LiDAR sample density (fewer points per scan)
- Reduce update rate (from 10 Hz to 5 Hz)
- Use rviz2 instead of rviz (better performance)

---

## Resources & Links

- **Gazebo Sensor Plugins**: http://gazebosim.org/tutorials?tut=sensor_configuration
- **ROS 2 Sensor Messages**: http://docs.ros.org/en/humble/Concepts/About-ROS-2-Interfaces.html
- **Unity Robotics Documentation**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **RViz User Guide**: http://wiki.ros.org/rviz/UserGuide

---

## Week 7 Summary Checklist

By Friday, you should have:

- ✅ Added LiDAR, depth camera, and IMU sensors to humanoid
- ✅ Visualized all sensor data in RViz
- ✅ Understood sensor noise models and their effects
- ✅ Set up Unity for robotics with ROS 2 integration
- ✅ Imported humanoid into Unity with realistic materials
- ✅ Configured HDRP photorealistic rendering
- ✅ Exported training dataset with annotations
- ✅ Completed 5 exercises and at least 1 challenge project

**If you've checked all boxes, you've completed Module 2! 🎉**

---

## Module 2 Capstone: Week 8

You now have the skills for the **Week 8 Capstone Project**, which combines everything:

**Task**: Create a complete Digital Twin simulation where your humanoid:
1. Spawns in a Gazebo physics-accurate environment
2. Uses sensors (LiDAR, depth, IMU) for perception
3. Executes a task (walk, navigate, grasp)
4. Gets visualized in high-fidelity in Unity
5. Generates synthetic training data

**Estimated time**: 6-8 hours

**Success**: Humanoid completes task successfully in both Gazebo and Unity, with annotated dataset exported.

---

**Congratulations on completing Module 2!** 🚀

You're now ready for **Module 3: NVIDIA Isaac Platform**, where you'll use advanced simulation features, generate massive datasets, and prepare for sim-to-real transfer.

See you in Module 3!
