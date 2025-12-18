---
sidebar_position: 8
title: "Week 5 Practice Guide"
tags: [Week5, Practice, URDF, Xacro, RViz, Gazebo, Capstone]
difficulty: Beginner
module: 1
week: 5
estimated_time: "10-12 hours"
topics: ["URDF", "Xacro", "RViz", "Gazebo", "Robot modeling", "Capstone project"]
---

# Week 5 Practice Guide: Robot Modeling and Capstone

**Focus**: URDF for humanoid robots and Module 1 Capstone Project (Chapter 4)

---

## Week Overview

This is the culmination of Module 1! This week you'll:
- Learn to describe robot structure using URDF
- Use Xacro to eliminate code duplication
- Visualize robots in RViz
- Simulate robots in Gazebo with physics
- **Start the Module 1 Capstone Project**: Humanoid Head Tracking System

**Time Allocation**: 10-12 hours total
- 📖 **Reading**: 2-3 hours (Chapter 4)
- 💻 **Hands-on Practice**: 4-5 hours (exercises below)
- 🎯 **Capstone Project**: 4-5 hours

---

## Learning Goals

By the end of Week 5, you should be able to:

✅ Write URDF files describing robot links, joints, and properties
✅ Create humanoid robot models with realistic kinematics
✅ Use Xacro macros to reduce duplication in robot descriptions
✅ Visualize robot models in RViz with joint state publisher
✅ Load URDF into Gazebo with physics simulation
✅ Integrate all Module 1 skills in a complete capstone project

---

## Day-by-Day Breakdown

### Day 1 (Monday): URDF Fundamentals
**Duration**: 2-2.5 hours
**Goal**: Understand URDF structure and basic robot modeling

#### Tasks
1. **Read**: Chapter 4, Sections 4.1-4.2 (URDF Fundamentals, Humanoid URDF Example)

2. **Create URDF Package**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create my_robot_description --build-type ament_python
   cd my_robot_description
   mkdir -p urdf meshes config launch
   ```

3. **Write Simple Robot URDF**:
   ```xml
   <!-- File: ~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf -->

   <?xml version="1.0"?>
   <robot name="simple_arm">

     <!-- Base link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.2 0.2 0.1"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 0.8 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.2 0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Upper arm -->
     <link name="upper_arm">
       <visual>
         <origin xyz="0 0 0.15" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.05" length="0.3"/>
         </geometry>
         <material name="red">
           <color rgba="0.8 0 0 1"/>
         </material>
       </visual>
       <collision>
         <origin xyz="0 0 0.15" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.05" length="0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2.0"/>
         <origin xyz="0 0 0.15"/>
         <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Shoulder joint -->
     <joint name="shoulder_joint" type="revolute">
       <parent link="base_link"/>
       <child link="upper_arm"/>
       <origin xyz="0 0 0.05" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
     </joint>

     <!-- Forearm -->
     <link name="forearm">
       <visual>
         <origin xyz="0 0 0.125" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.04" length="0.25"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <origin xyz="0 0 0.125" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.04" length="0.25"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <origin xyz="0 0 0.125"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Elbow joint -->
     <joint name="elbow_joint" type="revolute">
       <parent link="upper_arm"/>
       <child link="forearm"/>
       <origin xyz="0 0 0.3" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="0" upper="2.5" effort="80" velocity="2.0"/>
     </joint>

   </robot>
   ```

4. **Validate URDF**:
   ```bash
   # Install URDF tools
   sudo apt install liburdfdom-tools

   # Check URDF validity
   check_urdf ~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf

   # Expected output:
   # robot name is: simple_arm
   # ---------- Successfully Parsed XML ---------------
   ```

5. **Visualize URDF Structure**:
   ```bash
   urdf_to_graphiz ~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf
   # Creates simple_arm.pdf with kinematic tree diagram
   ```

**Success Criteria**: URDF validates successfully. You understand links, joints, and coordinate frames.

---

### Day 2 (Tuesday): RViz Visualization
**Duration**: 2-2.5 hours
**Goal**: View robot models in RViz with interactive joint control

#### Tasks
1. **Read**: Chapter 4, Section 4.4 (Visualizing in RViz)

2. **Create RViz Launch File**:
   ```python
   # File: ~/ros2_ws/src/my_robot_description/launch/view_robot.launch.py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       pkg_dir = get_package_share_directory('my_robot_description')
       urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf')

       # Read URDF file
       with open(urdf_file, 'r') as file:
           robot_description = file.read()

       # Robot State Publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='screen',
           parameters=[{'robot_description': robot_description}]
       )

       # Joint State Publisher GUI
       joint_state_publisher_gui = Node(
           package='joint_state_publisher_gui',
           executable='joint_state_publisher_gui',
           output='screen'
       )

       # RViz
       rviz_config = os.path.join(pkg_dir, 'config', 'view_robot.rviz')
       rviz = Node(
           package='rviz2',
           executable='rviz2',
           output='screen',
           arguments=['-d', rviz_config]
       )

       return LaunchDescription([
           robot_state_publisher,
           joint_state_publisher_gui,
           rviz,
       ])
   ```

3. **Create RViz Config**:
   ```yaml
   # File: ~/ros2_ws/src/my_robot_description/config/view_robot.rviz

   Panels:
     - Class: rviz_common/Displays
       Name: Displays

   Visualization Manager:
     Displays:
       - Class: rviz_default_plugins/Grid
         Name: Grid
         Reference Frame: base_link

       - Class: rviz_default_plugins/RobotModel
         Name: RobotModel
         Description Topic: /robot_description
         Visual Enabled: true
         Collision Enabled: false

       - Class: rviz_default_plugins/TF
         Name: TF
         Show Axes: true
         Show Names: true
         Frame Timeout: 15

     Global Options:
       Fixed Frame: base_link

     Views:
       Current:
         Class: rviz_default_plugins/Orbit
         Distance: 1.5
         Focal Point:
           X: 0.0
           Y: 0.0
           Z: 0.3
   ```

4. **Update setup.py**:
   ```python
   # In setup.py
   data_files=[
       ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'urdf'),
           glob('urdf/*.urdf')),
       (os.path.join('share', package_name, 'launch'),
           glob('launch/*.launch.py')),
       (os.path.join('share', package_name, 'config'),
           glob('config/*.rviz')),
   ],
   ```

5. **Build and Launch**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_description
   source install/setup.bash

   # Install joint_state_publisher_gui if needed
   sudo apt install ros-humble-joint-state-publisher-gui

   # Launch RViz
   ros2 launch my_robot_description view_robot.launch.py
   ```

6. **Interact with Robot**:
   - Use GUI sliders to move shoulder and elbow joints
   - Observe TF frames updating in RViz
   - Toggle visual/collision geometries

**Success Criteria**: RViz shows robot model. Joint state publisher GUI controls joint positions. TF frames are visible.

---

### Day 3 (Wednesday): Xacro for Code Reuse
**Duration**: 2-2.5 hours
**Goal**: Use Xacro to eliminate duplication and parameterize robots

#### Tasks
1. **Read**: Chapter 4, Section 4.3 (Xacro: Reducing Code Duplication)

2. **Convert Simple Arm to Xacro**:
   ```xml
   <!-- File: ~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf.xacro -->

   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm">

     <!-- Properties -->
     <xacro:property name="base_mass" value="5.0"/>
     <xacro:property name="arm_mass" value="2.0"/>
     <xacro:property name="upper_arm_length" value="0.3"/>
     <xacro:property name="forearm_length" value="0.25"/>

     <!-- Inertia macros -->
     <xacro:macro name="cylinder_inertia" params="mass radius length">
       <inertial>
         <mass value="${mass}"/>
         <inertia
           ixx="${mass * (3 * radius * radius + length * length) / 12}"
           ixy="0" ixz="0"
           iyy="${mass * (3 * radius * radius + length * length) / 12}"
           iyz="0"
           izz="${mass * radius * radius / 2}"/>
       </inertial>
     </xacro:macro>

     <xacro:macro name="box_inertia" params="mass x y z">
       <inertial>
         <mass value="${mass}"/>
         <inertia
           ixx="${mass * (y*y + z*z) / 12}"
           ixy="0" ixz="0"
           iyy="${mass * (x*x + z*z) / 12}"
           iyz="0"
           izz="${mass * (x*x + y*y) / 12}"/>
       </inertial>
     </xacro:macro>

     <!-- Base link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.2 0.2 0.1"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 0.8 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.2 0.1"/>
         </geometry>
       </collision>
       <xacro:box_inertia mass="${base_mass}" x="0.2" y="0.2" z="0.1"/>
     </link>

     <!-- Upper arm -->
     <link name="upper_arm">
       <visual>
         <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.05" length="${upper_arm_length}"/>
         </geometry>
         <material name="red">
           <color rgba="0.8 0 0 1"/>
         </material>
       </visual>
       <collision>
         <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.05" length="${upper_arm_length}"/>
         </geometry>
       </collision>
       <xacro:cylinder_inertia mass="${arm_mass}" radius="0.05" length="${upper_arm_length}"/>
     </link>

     <joint name="shoulder_joint" type="revolute">
       <parent link="base_link"/>
       <child link="upper_arm"/>
       <origin xyz="0 0 0.05" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
     </joint>

     <!-- Forearm -->
     <link name="forearm">
       <visual>
         <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.04" length="${forearm_length}"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="0.04" length="${forearm_length}"/>
         </geometry>
       </collision>
       <xacro:cylinder_inertia mass="1.5" radius="0.04" length="${forearm_length}"/>
     </link>

     <joint name="elbow_joint" type="revolute">
       <parent link="upper_arm"/>
       <child link="forearm"/>
       <origin xyz="0 0 ${upper_arm_length}" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="0" upper="2.5" effort="80" velocity="2.0"/>
     </joint>

   </robot>
   ```

3. **Update Launch File to Process Xacro**:
   ```python
   # Modify view_robot.launch.py
   import xacro

   # Instead of reading URDF file:
   xacro_file = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf.xacro')
   robot_description = xacro.process_file(xacro_file).toxml()
   ```

4. **Test Xacro Compilation**:
   ```bash
   # Convert xacro to URDF manually (for debugging)
   ros2 run xacro xacro ~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf.xacro > /tmp/output.urdf
   check_urdf /tmp/output.urdf

   # Launch with xacro
   cd ~/ros2_ws
   colcon build --packages-select my_robot_description
   source install/setup.bash
   ros2 launch my_robot_description view_robot.launch.py
   ```

5. **Experiment with Parameters**:
   - Change `upper_arm_length` to 0.5
   - Rebuild and observe longer arm in RViz

**Success Criteria**: Xacro compiles to valid URDF. Changing properties updates robot geometry.

---

### Day 4 (Thursday): Gazebo Simulation
**Duration**: 2-2.5 hours
**Goal**: Simulate robot with physics in Gazebo

#### Tasks
1. **Read**: Chapter 4, Section 4.5 (Gazebo Physics Simulation)

2. **Add Gazebo Properties to Xacro**:
   ```xml
   <!-- File: ~/ros2_ws/src/my_robot_description/urdf/simple_arm_gazebo.urdf.xacro -->

   <?xml version="1.0"?>
   <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm_gazebo">

     <!-- Include base robot -->
     <xacro:include filename="$(find my_robot_description)/urdf/simple_arm.urdf.xacro"/>

     <!-- Gazebo properties -->
     <gazebo reference="base_link">
       <material>Gazebo/Blue</material>
       <mu1>0.9</mu1>
       <mu2>0.9</mu2>
     </gazebo>

     <gazebo reference="upper_arm">
       <material>Gazebo/Red</material>
     </gazebo>

     <gazebo reference="forearm">
       <material>Gazebo/Red</material>
     </gazebo>

     <!-- ROS 2 Control plugin -->
     <gazebo>
       <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
         <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
       </plugin>
     </gazebo>

   </robot>
   ```

3. **Create Gazebo Launch File**:
   ```python
   # File: ~/ros2_ws/src/my_robot_description/launch/gazebo_robot.launch.py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node
   import xacro

   def generate_launch_description():
       pkg_dir = get_package_share_directory('my_robot_description')
       pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

       # Process xacro
       xacro_file = os.path.join(pkg_dir, 'urdf', 'simple_arm_gazebo.urdf.xacro')
       robot_description = xacro.process_file(xacro_file).toxml()

       # Robot State Publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           output='screen',
           parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
       )

       # Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
           ])
       )

       # Spawn robot
       spawn_robot = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-entity', 'simple_arm',
               '-topic', '/robot_description',
               '-x', '0', '-y', '0', '-z', '0.5'
           ],
           output='screen'
       )

       return LaunchDescription([
           gazebo,
           robot_state_publisher,
           spawn_robot,
       ])
   ```

4. **Build and Launch**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_description
   source install/setup.bash

   ros2 launch my_robot_description gazebo_robot.launch.py
   ```

5. **Interact with Simulation**:
   - Robot should fall due to gravity
   - Use Gazebo GUI to apply forces
   - Observe physics simulation

**Success Criteria**: Gazebo opens with robot. Physics simulation is active (robot falls if not supported).

---

### Day 5 (Friday): Capstone Project Kickoff
**Duration**: 3-4 hours
**Goal**: Start Module 1 Capstone - Humanoid Head Tracking System

#### Tasks
1. **Read**: Chapter 4, Capstone Integration section

2. **Capstone Requirements Review**:
   The capstone integrates all Module 1 skills:
   - **ROS 2 Nodes** (Chapter 2): Face detector, head controller
   - **Packages & Launch Files** (Chapter 3): Multi-package system
   - **URDF & Gazebo** (Chapter 4): Humanoid upper body model

3. **Create Capstone Workspace**:
   ```bash
   mkdir -p ~/capstone_ws/src
   cd ~/capstone_ws/src
   ```

4. **Plan Package Structure**:
   ```
   capstone_ws/src/
   ├── humanoid_description/      # URDF, meshes, config
   ├── humanoid_perception/        # Face detection node
   ├── humanoid_control/           # Head tracking controller
   └── humanoid_bringup/           # Launch files, params
   ```

5. **Create Packages**:
   ```bash
   cd ~/capstone_ws/src

   ros2 pkg create humanoid_description --build-type ament_python
   ros2 pkg create humanoid_perception --build-type ament_python \
     --dependencies rclpy sensor_msgs geometry_msgs
   ros2 pkg create humanoid_control --build-type ament_python \
     --dependencies rclpy geometry_msgs std_msgs
   ros2 pkg create humanoid_bringup --build-type ament_python
   ```

6. **Create Directory Structure**:
   ```bash
   # Description package
   cd humanoid_description
   mkdir -p urdf meshes config launch

   # Perception package
   cd ../humanoid_perception
   mkdir -p launch config

   # Control package
   cd ../humanoid_control
   mkdir -p launch config

   # Bringup package
   cd ../humanoid_bringup
   mkdir -p launch config worlds
   ```

7. **Start URDF Model** (from Chapter 4 example):
   ```bash
   # Copy humanoid upper body URDF from Chapter 4 as starting point
   # You'll extend this over the weekend
   ```

8. **Define Capstone Milestones**:
   - [ ] Milestone 1: URDF humanoid upper body complete
   - [ ] Milestone 2: RViz visualization working
   - [ ] Milestone 3: Face detector node (simulated)
   - [ ] Milestone 4: Head controller node
   - [ ] Milestone 5: Launch file integration
   - [ ] Milestone 6: Gazebo simulation with camera
   - [ ] Milestone 7: Full system integration

**Success Criteria**: Workspace created. Packages scaffolded. You have a clear plan for the capstone.

---

## Hands-On Exercises

Complete at least **2 of these 4 exercises** before starting capstone:

### Exercise 1: Mobile Robot URDF (Easy)
**Goal**: Create a differential drive robot

Build URDF with:
- Rectangular base (0.5m x 0.3m x 0.1m)
- Two wheels (0.1m radius, continuous joints)
- Caster wheel (sphere, fixed joint)

**Acceptance**: Validates with `check_urdf`, displays in RViz

---

### Exercise 2: Xacro Arm Macro (Medium)
**Goal**: Create reusable arm macro

Write xacro macro `humanoid_arm` with parameters:
- `side` ("left" or "right")
- `reflect` (1 or -1 for mirroring)
- `length` (arm length)

Use macro to create both arms.

**Acceptance**: Macro creates symmetric left/right arms

---

### Exercise 3: Sensor Integration (Medium)
**Goal**: Add sensors to URDF

Add to robot model:
- Camera on head (Gazebo camera plugin)
- IMU on torso (Gazebo IMU plugin)

Verify sensors publish in Gazebo.

**Acceptance**: `/camera/image_raw` and `/imu/data` topics publish

---

### Exercise 4: Collision Optimization (Medium-Hard)
**Goal**: Compare visual vs collision geometry performance

Create robot with:
- Detailed visual mesh (high polygon count)
- Simplified collision geometry (boxes/cylinders)

Measure Gazebo FPS with both.

**Acceptance**: Document FPS improvement with simplified collision

---

## Module 1 Capstone Project

### Overview
Build a **Humanoid Head Tracking System** that demonstrates all Module 1 skills.

### System Requirements

1. **URDF Model** (Chapter 4):
   - Humanoid upper body (torso, head, arms)
   - Head with 2-DOF neck (pan + tilt)
   - Camera sensor on head
   - Realistic joint limits and inertia

2. **ROS 2 Nodes** (Chapter 2):
   - **Face Detector**: Subscribes to camera, publishes detected face positions
   - **Head Controller**: Subscribes to face positions, publishes head joint commands
   - **Joint Command Publisher**: Converts commands to Gazebo joint control

3. **Package Structure** (Chapter 3):
   - 4 packages: description, perception, control, bringup
   - Launch file starts entire system
   - Parameters in YAML files

4. **Simulation** (Chapters 3-4):
   - Gazebo world with humanoid robot
   - Camera publishes images
   - Head tracks detected faces

### Implementation Steps

#### Step 1: Create Humanoid URDF (4-5 hours)
- Base: From Chapter 4, Section 4.2
- Extend with:
  - 2-DOF neck (pan joint + tilt joint)
  - Camera link on head
  - Gazebo camera plugin
- Test in RViz with joint_state_publisher_gui

#### Step 2: Implement Face Detector Node (1-2 hours)
```python
# Simplified face detector (simulated detection)
class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher_ = self.create_publisher(Point, '/detected_face_position', 10)

    def image_callback(self, msg):
        # Simulate face detection at center of image
        face_pos = Point()
        face_pos.x = 0.5  # Normalized coordinates
        face_pos.y = 0.5
        face_pos.z = 1.0  # Distance estimate
        self.publisher_.publish(face_pos)
```

#### Step 3: Implement Head Controller (1-2 hours)
```python
# Convert face position to head joint angles
class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')
        self.subscription = self.create_subscription(
            Point, '/detected_face_position', self.face_callback, 10
        )
        self.publisher_ = self.create_publisher(
            JointState, '/head_joint_commands', 10
        )

    def face_callback(self, msg):
        # Convert face position to pan/tilt angles
        pan_angle = (msg.x - 0.5) * 1.57  # ±90 degrees
        tilt_angle = (msg.y - 0.5) * 0.785  # ±45 degrees

        joint_cmd = JointState()
        joint_cmd.name = ['head_pan_joint', 'head_tilt_joint']
        joint_cmd.position = [pan_angle, tilt_angle]
        self.publisher_.publish(joint_cmd)
```

#### Step 4: Create Launch File (1 hour)
```python
def generate_launch_description():
    # Load URDF
    # Start Gazebo
    # Spawn robot
    # Launch face_detector
    # Launch head_controller
    # Start RViz
    pass
```

#### Step 5: Integration and Testing (1-2 hours)
- Test each node individually
- Test full system integration
- Debug and refine

### Acceptance Criteria

✅ Humanoid robot displays in RViz and Gazebo
✅ Camera publishes images from Gazebo
✅ Face detector publishes face positions
✅ Head controller commands head joints
✅ Head tracks face (visual confirmation in Gazebo)
✅ All nodes start with single launch command
✅ System runs without errors for 1+ minutes

### Bonus Challenges

🌟 Add real face detection with OpenCV
🌟 Implement PID control for smooth tracking
🌟 Add second camera (eyes) for stereo vision
🌟 Record and playback tracking data
🌟 Add arm control (wave at detected face)

---

## Weekly Checklist

Final Module 1 checklist:

- [ ] Completed Chapter 4 reading
- [ ] Created and validated URDF files
- [ ] Used xacro macros
- [ ] Visualized robots in RViz
- [ ] Simulated robots in Gazebo
- [ ] Completed at least 2 hands-on exercises
- [ ] Started capstone project
- [ ] Created all 4 capstone packages
- [ ] Implemented URDF humanoid model
- [ ] **Capstone system fully integrated and tested**

---

## Debugging Tips

### Issue: URDF validation fails
```bash
# Check XML syntax
# Ensure all <link> have <inertial> tags
# Verify joint parent/child links exist
check_urdf --verbose my_robot.urdf
```

### Issue: Robot not visible in RViz
```bash
# Check robot_state_publisher is running
ros2 node list

# Verify /robot_description topic
ros2 topic echo /robot_description

# Check Fixed Frame in RViz matches URDF base_link
```

### Issue: Gazebo spawn fails
```bash
# Check URDF is valid
# Ensure use_sim_time:=True
# Verify spawn_entity.py arguments

# Debug: manually publish robot_description
ros2 topic pub /robot_description std_msgs/msg/String "{data: '...'}"
```

### Issue: Xacro compilation error
```bash
# Test xacro manually:
ros2 run xacro xacro my_robot.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

# Check for unclosed tags, typos in properties
```

---

## Resources

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [RViz User Guide](https://github.com/ros2/rviz)
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html)

---

## Congratulations! 🎉

You've completed **Module 1: The Robotic Nervous System (ROS 2)**!

### What You've Achieved
- Mastered ROS 2 fundamentals (nodes, topics, services, actions)
- Built professional ROS 2 packages with launch files and parameters
- Created URDF models for humanoid robots
- Integrated RViz and Gazebo for visualization and simulation
- **Delivered a complete humanoid head tracking system** (capstone)

### Next Steps

**Module 2: Digital Twin Simulation** (Weeks 6-8)
- Advanced Gazebo simulation techniques
- Unity integration for photorealistic rendering
- Sensor simulation and synthetic data generation

**Continue Learning**: You now have a solid ROS 2 foundation. Keep building, experimenting, and creating!

---

**Finish strong with your capstone!** This is your chance to showcase everything you've learned. Good luck! 🤖
