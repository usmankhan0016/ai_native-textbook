---
sidebar_position: 7
title: "Week 4 Practice Guide"
tags: [Week4, Practice, Packages, Launch Files, Parameters, Colcon]
difficulty: Beginner
module: 1
week: 4
estimated_time: "8-10 hours"
topics: ["ROS 2 packages", "Launch files", "Parameters", "Colcon build", "Gazebo integration"]
---

# Week 4 Practice Guide: Professional ROS 2 Development

**Focus**: Packages, launch files, and parameters (Chapter 3)

---

## Week Overview

This week, you'll level up your ROS 2 skills by learning professional development practices:
- Structuring code into reusable packages
- Orchestrating multi-node systems with launch files
- Managing configuration with parameters
- Integrating ROS 2 with Gazebo simulation

**Time Allocation**: 8-10 hours total
- 📖 **Reading**: 2-3 hours (Chapter 3)
- 💻 **Hands-on Practice**: 5-6 hours (exercises below)
- 🎯 **Challenge Projects**: 1-2 hours

---

## Learning Goals

By the end of Week 4, you should be able to:

✅ Create well-structured ROS 2 Python packages with proper dependencies
✅ Write launch files to start multiple nodes with a single command
✅ Manage parameters using YAML configuration files
✅ Use colcon to build and manage multi-package workspaces
✅ Launch Gazebo simulations integrated with ROS 2
✅ Organize projects following ROS 2 best practices

---

## Day-by-Day Breakdown

### Day 1 (Monday): Package Creation and Structure
**Duration**: 2-2.5 hours
**Goal**: Master ROS 2 package organization

#### Tasks
1. **Read**: Chapter 3, Sections 3.1-3.2 (Workspace Structure, Creating Packages)

2. **Create Multi-Package Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   # Create perception package
   ros2 pkg create humanoid_perception \
     --build-type ament_python \
     --dependencies rclpy std_msgs sensor_msgs geometry_msgs

   # Create control package
   ros2 pkg create humanoid_control \
     --build-type ament_python \
     --dependencies rclpy std_msgs geometry_msgs trajectory_msgs
   ```

3. **Understand package.xml**:
   - Open `humanoid_perception/package.xml`
   - Identify: `<name>`, `<version>`, `<depend>` tags
   - Add a new dependency:
     ```xml
     <depend>cv_bridge</depend>
     ```

4. **Configure setup.py**:
   ```python
   # In humanoid_perception/setup.py

   from setuptools import find_packages, setup
   import os
   from glob import glob

   package_name = 'humanoid_perception'

   setup(
       name=package_name,
       version='1.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           # Install launch files
           (os.path.join('share', package_name, 'launch'),
               glob(os.path.join('launch', '*.launch.py'))),
           # Install config files
           (os.path.join('share', package_name, 'config'),
               glob(os.path.join('config', '*.yaml'))),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       entry_points={
           'console_scripts': [
               'face_detector = humanoid_perception.face_detector:main',
           ],
       },
   )
   ```

5. **Build Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash

   # Verify packages are built
   ros2 pkg list | grep humanoid
   ```

**Success Criteria**: Both packages build successfully. You can explain the role of `package.xml` and `setup.py`.

---

### Day 2 (Tuesday): Your First Launch File
**Duration**: 2-2.5 hours
**Goal**: Write launch files to orchestrate multiple nodes

#### Tasks
1. **Read**: Chapter 3, Section 3.3 (Launch Files)

2. **Create Directory Structure**:
   ```bash
   cd ~/ros2_ws/src/humanoid_perception
   mkdir -p launch config
   ```

3. **Create Simple Nodes** (for testing launch file):
   ```python
   # File: ~/ros2_ws/src/humanoid_perception/humanoid_perception/camera_node.py

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import Header

   class CameraNode(Node):
       def __init__(self):
           super().__init__('camera_node')
           self.declare_parameter('frame_rate', 30.0)
           self.declare_parameter('camera_name', 'head_camera')

           rate = self.get_parameter('frame_rate').value
           cam_name = self.get_parameter('camera_name').value

           self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
           self.timer = self.create_timer(1.0/rate, self.publish_image)

           self.get_logger().info(f'{cam_name} started at {rate} Hz')

       def publish_image(self):
           msg = Image()
           msg.header = Header()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = 'camera_link'
           self.publisher_.publish(msg)

   def main(args=None):
       rclpy.init(args=args)
       node = CameraNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

   ```python
   # File: ~/ros2_ws/src/humanoid_perception/humanoid_perception/face_detector.py

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from geometry_msgs.msg import Point

   class FaceDetector(Node):
       def __init__(self):
           super().__init__('face_detector')
           self.subscription = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10
           )
           self.publisher_ = self.create_publisher(Point, '/detected_face', 10)
           self.get_logger().info('Face detector ready')

       def image_callback(self, msg):
           # Simulate face detection (dummy output)
           face_position = Point()
           face_position.x = 0.5
           face_position.y = 0.5
           face_position.z = 1.0
           self.publisher_.publish(face_position)
           self.get_logger().info('Face detected')

   def main(args=None):
       rclpy.init(args=args)
       node = FaceDetector()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

4. **Update setup.py** with entry points:
   ```python
   entry_points={
       'console_scripts': [
           'camera_node = humanoid_perception.camera_node:main',
           'face_detector = humanoid_perception.face_detector:main',
       ],
   },
   ```

5. **Create Launch File**:
   ```python
   # File: ~/ros2_ws/src/humanoid_perception/launch/perception.launch.py

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           # Camera node
           Node(
               package='humanoid_perception',
               executable='camera_node',
               name='camera',
               output='screen',
               parameters=[{
                   'frame_rate': 30.0,
                   'camera_name': 'head_camera'
               }]
           ),

           # Face detector node
           Node(
               package='humanoid_perception',
               executable='face_detector',
               name='face_detector',
               output='screen'
           ),
       ])
   ```

6. **Build and Test**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select humanoid_perception
   source install/setup.bash

   # Launch both nodes with one command
   ros2 launch humanoid_perception perception.launch.py

   # In another terminal, verify:
   ros2 node list
   ros2 topic list
   ros2 topic echo /detected_face
   ```

**Success Criteria**: Launch file starts both nodes. Face detector receives camera images and publishes detections.

---

### Day 3 (Wednesday): Parameter Management
**Duration**: 1.5-2 hours
**Goal**: Externalize configuration with YAML parameter files

#### Tasks
1. **Read**: Chapter 3, Section 3.4 (Parameter Management)

2. **Create Parameter File**:
   ```yaml
   # File: ~/ros2_ws/src/humanoid_perception/config/perception_params.yaml

   camera_node:
     ros__parameters:
       frame_rate: 60.0
       camera_name: 'left_eye_camera'
       resolution_width: 1280
       resolution_height: 720

   face_detector:
     ros__parameters:
       detection_confidence: 0.8
       tracking_enabled: true
       max_faces: 5
   ```

3. **Update Nodes to Use More Parameters**:
   ```python
   # Add to FaceDetector.__init__()
   self.declare_parameter('detection_confidence', 0.7)
   self.declare_parameter('tracking_enabled', False)
   self.declare_parameter('max_faces', 1)

   confidence = self.get_parameter('detection_confidence').value
   self.get_logger().info(f'Detection confidence: {confidence}')
   ```

4. **Update Launch File to Load YAML**:
   ```python
   # File: ~/ros2_ws/src/humanoid_perception/launch/perception_with_params.launch.py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       pkg_dir = get_package_share_directory('humanoid_perception')
       params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

       return LaunchDescription([
           Node(
               package='humanoid_perception',
               executable='camera_node',
               name='camera_node',
               output='screen',
               parameters=[params_file]
           ),

           Node(
               package='humanoid_perception',
               executable='face_detector',
               name='face_detector',
               output='screen',
               parameters=[params_file]
           ),
       ])
   ```

5. **Test Parameter Loading**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select humanoid_perception
   source install/setup.bash

   ros2 launch humanoid_perception perception_with_params.launch.py

   # Verify parameters are loaded:
   ros2 param list /camera_node
   ros2 param get /camera_node frame_rate
   # Expected: 60.0 (from YAML, not default)
   ```

6. **Runtime Parameter Updates**:
   ```bash
   # While launch file is running:
   ros2 param set /camera_node frame_rate 120.0
   ros2 param get /camera_node frame_rate
   # Expected: 120.0
   ```

**Success Criteria**: Parameters load from YAML. Nodes log correct parameter values. Runtime updates work.

---

### Day 4 (Thursday): Multi-Package Systems and Colcon
**Duration**: 2-2.5 hours
**Goal**: Build complex systems with multiple packages

#### Tasks
1. **Read**: Chapter 3, Sections 3.6-3.7 (Multi-Node Systems, Colcon)

2. **Create Control Package Nodes**:
   ```python
   # File: ~/ros2_ws/src/humanoid_control/humanoid_control/head_controller.py

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Point
   from std_msgs.msg import Float64MultiArray

   class HeadController(Node):
       def __init__(self):
           super().__init__('head_controller')
           self.subscription = self.create_subscription(
               Point, '/detected_face', self.face_callback, 10
           )
           self.publisher_ = self.create_publisher(
               Float64MultiArray, '/head_joint_commands', 10
           )
           self.get_logger().info('Head controller ready')

       def face_callback(self, msg):
           # Convert face position to head angles (simplified)
           angles = Float64MultiArray()
           angles.data = [msg.x - 0.5, msg.y - 0.5]  # Pan, Tilt
           self.publisher_.publish(angles)
           self.get_logger().info(f'Head angles: {angles.data}')

   def main(args=None):
       rclpy.init(args=args)
       node = HeadController()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

3. **Create Top-Level Launch File**:
   ```bash
   # Create bringup package
   cd ~/ros2_ws/src
   ros2 pkg create humanoid_bringup --build-type ament_python
   mkdir -p humanoid_bringup/launch
   ```

   ```python
   # File: ~/ros2_ws/src/humanoid_bringup/launch/full_system.launch.py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node

   def generate_launch_description():
       # Get package directories
       perception_pkg = get_package_share_directory('humanoid_perception')
       control_pkg = get_package_share_directory('humanoid_control')

       # Include perception launch file
       perception_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               os.path.join(perception_pkg, 'launch', 'perception_with_params.launch.py')
           ])
       )

       # Control nodes
       head_controller = Node(
           package='humanoid_control',
           executable='head_controller',
           name='head_controller',
           output='screen'
       )

       return LaunchDescription([
           perception_launch,
           head_controller,
       ])
   ```

4. **Build Entire Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash

   # Build specific packages with dependencies
   colcon build --packages-up-to humanoid_bringup
   ```

5. **Test Full System**:
   ```bash
   ros2 launch humanoid_bringup full_system.launch.py

   # Verify all nodes are running:
   ros2 node list

   # Check data flow:
   rqt_graph
   ```

**Success Criteria**: All 3 packages build. Full system launch file starts perception and control nodes. `rqt_graph` shows complete data flow.

---

### Day 5 (Friday): Gazebo Integration
**Duration**: 1.5-2 hours
**Goal**: Integrate ROS 2 with Gazebo simulation

#### Tasks
1. **Read**: Chapter 3, Section 3.5 (Gazebo Integration)

2. **Install Gazebo** (if not already installed):
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

3. **Create Simple Gazebo World**:
   ```xml
   <!-- File: ~/ros2_ws/src/humanoid_bringup/worlds/simple_room.world -->

   <?xml version="1.0"?>
   <sdf version="1.6">
     <world name="simple_room">
       <!-- Sun light -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- A table -->
       <include>
         <uri>model://table</uri>
         <pose>2 0 0 0 0 0</pose>
       </include>
     </world>
   </sdf>
   ```

4. **Create Gazebo Launch File**:
   ```python
   # File: ~/ros2_ws/src/humanoid_bringup/launch/gazebo_demo.launch.py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node

   def generate_launch_description():
       pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
       pkg_bringup = get_package_share_directory('humanoid_bringup')

       world_file = os.path.join(pkg_bringup, 'worlds', 'simple_room.world')

       # Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
           ]),
           launch_arguments={'world': world_file, 'verbose': 'true'}.items()
       )

       return LaunchDescription([
           gazebo,
       ])
   ```

5. **Update setup.py** to install worlds:
   ```python
   # In humanoid_bringup/setup.py
   data_files=[
       # ... existing data_files ...
       (os.path.join('share', package_name, 'worlds'),
           glob(os.path.join('worlds', '*.world'))),
       (os.path.join('share', package_name, 'launch'),
           glob(os.path.join('launch', '*.launch.py'))),
   ],
   ```

6. **Build and Launch**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select humanoid_bringup
   source install/setup.bash

   ros2 launch humanoid_bringup gazebo_demo.launch.py
   ```

**Success Criteria**: Gazebo opens with the simple room world. You understand how to include Gazebo in launch files.

---

## Hands-On Exercises

Complete at least **3 of these 5 exercises**:

### Exercise 1: Create a Package with Launch Files (Easy)
**Goal**: Practice package creation and launch file structure

Create a package `robot_sensors` with:
- 3 nodes: `camera_driver.py`, `lidar_driver.py`, `imu_driver.py`
- 1 launch file that starts all 3 sensors
- 1 parameter file with sensor configurations

**Acceptance**: `ros2 launch robot_sensors sensors.launch.py` starts all 3 nodes

---

### Exercise 2: Parameter Hot-Reload (Medium)
**Goal**: Implement parameter callback for runtime updates

Create a node that:
- Declares parameter `publish_rate`
- Uses parameter callback to update timer when parameter changes
- Logs "Rate changed to X Hz" when updated

**Acceptance**: Changing parameter with `ros2 param set` immediately updates publish rate

---

### Exercise 3: Multi-Robot Namespacing (Medium)
**Goal**: Launch multiple robot instances with namespaces

Create a launch file that starts 2 instances of the same node:
- Instance 1: namespace `/robot1`
- Instance 2: namespace `/robot2`

Each publishes to `/robotX/status`

**Acceptance**: `ros2 topic list` shows `/robot1/status` and `/robot2/status`

---

### Exercise 4: Launch File Arguments (Medium)
**Goal**: Make launch files configurable

Create a launch file with arguments:
- `robot_name` (default: "humanoid")
- `enable_visualization` (default: true)
- `use_sim_time` (default: false)

Use arguments to configure node parameters and conditionally launch RViz.

**Acceptance**: Can run with `ros2 launch pkg demo.launch.py robot_name:=robot1 enable_visualization:=false`

---

### Exercise 5: Workspace Overlay (Medium-Hard)
**Goal**: Understand workspace overlays

Create two workspaces:
1. `~/base_ws`: Contains `sensor_pkg`
2. `~/dev_ws`: Contains modified version of `sensor_pkg`

Test that `dev_ws` overrides `base_ws` when sourced.

**Acceptance**: Running node from `dev_ws` uses modified version

---

## Challenge Projects

Pick **one challenge** to test your skills:

### Challenge 1: Complete Perception Pipeline (Intermediate)
Build a 4-node perception system:
1. **Camera Node**: Publishes images at 30 Hz
2. **Image Processor**: Applies filtering
3. **Object Detector**: Detects objects in processed images
4. **Tracker**: Tracks detected objects over time

Requirements:
- All nodes in one package
- Single launch file with parameter file
- Parameters for each node (rates, thresholds, etc.)

**Bonus**: Add Gazebo camera plugin as image source

---

### Challenge 2: Multi-Package Humanoid System (Advanced)
Create a 3-package system:
1. **humanoid_description**: URDF files (dummy robot for now)
2. **humanoid_perception**: Vision nodes
3. **humanoid_control**: Control nodes
4. **humanoid_bringup**: Launch files and config

Top-level launch file should:
- Load URDF
- Start Gazebo
- Launch perception nodes
- Launch control nodes
- Start RViz

**Bonus**: Add namespace support for multi-robot scenarios

---

### Challenge 3: Dynamic Reconfigure System (Advanced)
Build a system where:
- **Configuration Manager Node**: Loads configs from YAML
- **Worker Nodes** (3): Subscribe to config updates
- Config changes propagate to all workers in real-time

Use services for configuration updates.

**Bonus**: Implement parameter validation (reject invalid configs)

---

## Weekly Checklist

Track your progress:

- [ ] Completed Chapter 3 reading
- [ ] Created multi-package workspace
- [ ] Wrote and tested launch files
- [ ] Created parameter YAML files
- [ ] Loaded parameters from launch files
- [ ] Built entire workspace with colcon
- [ ] Launched Gazebo with ROS 2 integration
- [ ] Used `IncludeLaunchDescription` to compose launch files
- [ ] Completed at least 3 hands-on exercises
- [ ] Completed 1 challenge project
- [ ] Can explain package structure to someone else

---

## Debugging Tips

### Issue: "Package share directory not found"
```bash
# Solution: Rebuild and source workspace
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

### Issue: Launch file not found
```bash
# Ensure setup.py installs launch files:
data_files=[
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
]

# Rebuild after modifying setup.py
colcon build --packages-select <pkg>
```

### Issue: Parameters not loading from YAML
```bash
# Check YAML syntax (indentation matters!)
# Verify setup.py installs config files
# Check param file path in launch file with:
pkg_dir = get_package_share_directory('pkg_name')
print(f"Package directory: {pkg_dir}")
```

### Issue: Colcon build fails
```bash
# Clean build
rm -rf build/ install/ log/
colcon build

# Build with verbose output
colcon build --event-handlers console_direct+
```

---

## Resources

- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

---

## Next Week Preview

**Week 5** focuses on:
- Creating URDF models for humanoid robots
- Using Xacro to eliminate code duplication
- Visualizing robots in RViz
- Simulating robots in Gazebo with physics

Get ready to build your first humanoid robot model! 🤖

---

**Keep practicing!** Launch files and packages are the foundation of professional ROS 2 development.
