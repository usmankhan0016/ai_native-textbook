---
sidebar_position: 5
sidebar_label: "Week 6: Gazebo Fundamentals"
title: "Week 6 Practice Guide: Gazebo Fundamentals"
tags: ["practice", "gazebo", "week-6", "hands-on"]
difficulty: "Intermediate"
module: 2
week: 6
prerequisites: ["Chapters 1-2"]
estimated_time: "18-20 hours"
topics: ["gazebo", "physics-simulation", "world-creation", "ros2-integration"]
---

# Week 6 Practice Guide: Gazebo Fundamentals

**Week 6 (5 days of study) | 18-20 hours total | Gazebo physics simulation**

Welcome to Week 6! This week focuses on **hands-on Gazebo simulation**. You'll progress from installing Gazebo to creating complex simulation worlds with humanoid robots. By Friday, you'll have a fully functional physics simulation controlled via ROS 2.

## Week 6 Objectives

By the end of this week, you will:

1. ✅ Install and verify Gazebo on Ubuntu 22.04
2. ✅ Understand SDF/URDF file formats and when to use each
3. ✅ Create custom Gazebo worlds with physics parameters and obstacles
4. ✅ Load humanoid robots from URDF files into your simulations
5. ✅ Configure basic sensors (camera, IMU) and publish data over ROS 2
6. ✅ Interact with simulations using ROS 2 commands
7. ✅ Debug physics issues and tune parameters for stable behavior

## Daily Learning Path

```
Monday       Tuesday      Wednesday     Thursday      Friday
│            │             │             │             │
Install   → SDF/URDF → Custom Worlds → Humanoid → ROS 2 Integration
Gazebo      Basics       & Physics        Load        & Debugging
│            │             │             │             │
1 hour      1.5 hours     2 hours       2 hours      2.5 hours
```

---

## Monday: Gazebo Installation and Orientation (1 hour)

### Morning: Install Gazebo

**Task**: Install Gazebo on Ubuntu 22.04 and verify it works.

```bash
# Update package list
sudo apt-get update

# Install Gazebo and related tools
sudo apt-get install gazebo gazebo-dev gazebo-plugins ros-humble-gazebo-ros

# Verify installation
gazebo --version
# Output: Gazebo 11.13.0 (or similar)

# Test Gazebo launch
gazebo
# Window should open with empty world
```

**Checkpoint**: Gazebo GUI opens successfully, shows ground plane and sun

### Afternoon: Explore Gazebo Interface

**Task**: Familiarize yourself with the Gazebo GUI.

In the Gazebo window:
- **Top menu**: File (load/save), Edit, View, Plugins
- **Left panel**: Model list (shows all objects and robots in scene)
- **Center**: 3D viewport where simulation runs
- **Bottom**: Playback controls (pause, play, step forward)
- **Tools menu**: View center of mass, collision geometry, joints

**Exercise**:
1. Open Tools → View and toggle "Show Center of Mass" on/off
2. Add a model: Insert tab → Create shapes (cube, sphere, cylinder)
3. Grab objects with mouse and drag them around
4. Use scroll wheel to zoom in/out

**Checkpoint**: You can navigate the 3D view and understand the interface layout

---

## Tuesday: SDF and URDF Basics (1.5 hours)

### Morning: Understand SDF Format

**Task**: Create a simple SDF world file and launch it in Gazebo.

Create file `my_first_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_first_world">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple red cube -->
    <model name="red_cube">
      <static>false</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <iyy>0.01</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch**:
```bash
gazebo my_first_world.sdf
```

**Checkpoint**: Red cube appears in Gazebo, falls due to gravity, lands on ground

### Afternoon: Review URDF Format

**Task**: Review your Module 1 humanoid.urdf file.

Understand key URDF elements:
- `<link>`: Rigid body with mass, inertia, collision, visual geometry
- `<joint>`: Connects two links, allows rotation/translation
- `<inertial>`: Mass and inertia tensor for physics
- `<collision>`: Shape for collision detection
- `<visual>`: Shape for visualization

**Example snippet**:
```xml
<link name="base_link">
  <inertial>
    <mass value="70.0" />
    <inertia ixx="5.0" ixy="0" ixz="0" iyy="5.0" iyz="0" izz="2.0" />
  </inertial>
  <collision>
    <geometry>
      <cylinder length="0.5" radius="0.2" />
    </geometry>
  </collision>
  <visual>
    <geometry>
      <cylinder length="0.5" radius="0.2" />
    </geometry>
  </visual>
</link>

<joint name="left_hip" type="revolute">
  <parent link="base_link" />
  <child link="left_thigh" />
  <axis xyz="0 1 0" />
  <limit lower="-0.785" upper="0.785" />
</joint>
```

**Checkpoint**: You understand URDF structure and how to read it

---

## Wednesday: Create Custom Worlds with Physics (2 hours)

### Morning: Create Multi-Object World

**Task**: Create a world with ground plane, platform, and 3 objects of different masses.

Create file `physics_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_world">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 1 kg box -->
    <model name="light_box">
      <pose>0 0 1.0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <iyy>0.01</iyy>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>  <!-- Green -->
          </material>
        </visual>
      </link>
    </model>

    <!-- 5 kg cylinder -->
    <model name="medium_cylinder">
      <pose>0.3 0 1.0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.05</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>  <!-- Yellow -->
          </material>
        </visual>
      </link>
    </model>

    <!-- 10 kg sphere -->
    <model name="heavy_sphere">
      <pose>0.6 0 1.0 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.4</ixx>
            <iyy>0.4</iyy>
            <izz>0.4</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>  <!-- Blue -->
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Launch and observe**:
```bash
gazebo physics_world.sdf
```

All three objects should fall at the same rate (gravity affects all equally). Mass affects inertia but not fall time.

### Afternoon: Tune Physics Parameters

**Task**: Modify friction and collision parameters.

Modify physics section:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>  <!-- Increased from 50 -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <contact_surface_layer>0.002</contact_surface_layer>  <!-- Increased -->
    </constraints>
  </ode>
</physics>
```

**Checkpoint**: Objects settle stably on ground without jittering

---

## Thursday: Load Humanoid Robot (2 hours)

### Morning: Spawn Humanoid in World

**Task**: Add your humanoid.urdf to the physics world.

Edit `physics_world.sdf`, add before `</world>`:

```xml
<!-- Load humanoid robot -->
<include>
  <uri>file:///path/to/humanoid.urdf</uri>
  <name>humanoid</name>
  <pose>0 0 1.5 0 0 0</pose>  <!-- Start above ground -->
</include>
```

**Launch**:
```bash
gazebo physics_world.sdf
```

**Observe**: Humanoid appears above ground and falls. It should land stably on its feet.

**Checkpoint**: Humanoid spawns without errors and lands on ground

### Afternoon: Verify Joint Structure

**Task**: Inspect humanoid joints in Gazebo and ROS 2.

**In Gazebo**:
- Tools → View → Show Joints (see joint axes)
- Tools → View → Show Center of Mass (see COM on each link)

**In ROS 2**:
```bash
ros2 topic echo /joint_states
# Shows all joint names and angles
```

Verify:
- ✅ Joint names match your URDF
- ✅ Angles are reasonable (0 rad for neutral pose)
- ✅ No error messages

**Checkpoint**: Joint structure is correct and humanoid is stable

---

## Friday: ROS 2 Integration and Debugging (2.5 hours)

### Morning: Publish and Subscribe to Joint States

**Task**: Write ROS 2 nodes to control humanoid joints.

**Node 1: View joint states** (`listener_node.py`):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Listener(Node):
    def __init__(self):
        super().__init__('joint_listener')
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Joints: {len(msg.name)}')
        for name, pos in zip(msg.name, msg.position):
            self.get_logger().info(f'  {name}: {pos:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Run**:
```bash
# Terminal 1: Start Gazebo
gazebo physics_world.sdf

# Terminal 2: Run listener
ros2 run <package> listener_node

# Should see joint angles updating
```

### Afternoon: Debugging Tools

**Task**: Use Gazebo and ROS 2 tools to debug simulation.

**Gazebo debugging**:
- Tools → View → Show Center of Mass (verify humanoid balance)
- Tools → View → Show Collisions (verify collision geometry)
- Right-click model → Properties (inspect mass, inertia)

**ROS 2 debugging**:
```bash
# List all topics
ros2 topic list

# Get topic info
ros2 topic info /joint_states

# Check message details
ros2 interface show sensor_msgs/JointState

# Monitor specific joint
ros2 topic echo /joint_states --filter "m.name[0]"
```

**Checkpoint**: You can inspect and debug the simulation using both tools

---

## Week 6 Exercises (5 independent exercises, 1-2 hours each)

### Exercise 1: Custom World with Obstacles

**Task**: Create a Gazebo world with:
- 2m × 2m ground plane
- 3 obstacles (boxes, cylinders, or spheres) at different positions
- Humanoid robot spawned at origin

**Acceptance Criteria**:
- World loads in Gazebo without errors
- All objects are visible
- Objects have realistic physics (don't sink or fly away)

**Hint**: Use the `physics_world.sdf` template and add more models

---

### Exercise 2: Physics Parameter Tuning

**Task**: Create two identical worlds with different physics parameters:

**World A**:
```xml
<ode>
  <solver>
    <iters>50</iters>
  </solver>
  <constraints>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</ode>
```

**World B**:
```xml
<ode>
  <solver>
    <iters>100</iters>
  </solver>
  <constraints>
    <contact_surface_layer>0.002</contact_surface_layer>
  </constraints>
</ode>
```

**Task**: Drop a box in each world. Document differences:
- Stability (jittering yes/no?)
- Behavior on ground (bouncing yes/no?)
- Performance (simulation speed)

**Acceptance Criteria**:
- Both worlds load successfully
- You can describe 3 observable differences
- Document findings in a text file

---

### Exercise 3: Joint State Monitoring

**Task**: Write a Python ROS 2 node that:
1. Subscribes to `/joint_states`
2. Logs the maximum and minimum joint angles
3. Calculates average velocity for each joint

**Acceptance Criteria**:
- Node runs without errors
- Logs appear as humanoid moves
- Statistics are reasonable (angles within [-π, π], velocities match movement)

---

### Exercise 4: Compare Physics Engines

**Task**: Run the same world with two different physics engines (ODE and Bullet).

**Create `ode_world.sdf`**:
```xml
<physics type="ode">
  ...
</physics>
```

**Create `bullet_world.sdf`**:
```xml
<physics type="bullet">
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
    </solver>
  </bullet>
</physics>
```

**Compare**: Drop humanoid in each world. Note:
- Speed (which is faster?)
- Stability (which is more stable?)
- Movement quality (which looks more realistic?)

---

### Exercise 5: Sensor Data Publishing

**Task**: Add a camera and IMU to humanoid, publish sensor data.

Edit `physics_world.sdf`, add to humanoid link:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <remapping>image_raw:=camera/image</remapping>
    </ros>
  </plugin>
</sensor>
```

**Verify**:
```bash
ros2 topic list | grep camera
ros2 topic echo /humanoid_robot/camera/image
```

---

## Week 6 Challenge Projects (2-3 hours each, optional)

### Challenge 1: Humanoid Walking Simulation

**Objective**: Make the humanoid walk forward using ROS 2 joint commands.

**Steps**:
1. Create a world with flat ground and humanoid at origin
2. Write a ROS 2 node that publishes joint commands:
   - Left leg forward (positive hip angle)
   - Right leg back (negative hip angle)
   - Alternate every 0.5 seconds
3. Observe humanoid attempting to walk

**Success Criteria**:
- Humanoid moves forward (even if awkwardly)
- No falls or instability
- Completes at least 1 meter of movement

---

### Challenge 2: Obstacle Avoidance Navigation

**Objective**: Navigate humanoid through an obstacle course using a simple algorithm.

**Setup**:
- Create a world with 5 obstacle boxes arranged in a line
- Humanoid starts at one end

**Algorithm**:
- Detect obstacles using visual or touch sensors
- Command humanoid to move forward if no obstacle
- Rotate humanoid if obstacle detected

**Success Criteria**:
- Humanoid navigates through obstacle course
- Completes in `<60` seconds simulation time
- No crashes into obstacles

---

## Debugging Tips & Tricks

### Issue: Humanoid sinks through ground

**Solution**:
- Check URDF collision geometry matches visual geometry
- Increase contact_surface_layer in ODE settings
- Reduce max_step_size (use 0.001 or smaller)

### Issue: Jittering or vibration when at rest

**Solution**:
- Increase friction coefficient (mu to 1.0 or higher)
- Increase contact damping in ODE constraints
- Verify inertia tensors are realistic

### Issue: Simulation running slowly

**Solution**:
- Use Bullet physics instead of DART
- Reduce ODE iterations from 100 to 50
- Simplify collision geometry (fewer objects)

### Issue: Objects falling at different speeds

**Solution**: This shouldn't happen. Gravity affects all objects equally regardless of mass. Check:
- Friction isn't too high
- Collision geometry isn't stuck

---

## Resources & Links

- **Gazebo Tutorial**: https://gazebosim.org/docs/fortress/getstarted/
- **SDF Format Specification**: https://github.com/gazebosim/sdf_tutorials
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **Gazebo ROS 2 Plugins**: http://wiki.ros.org/gazebo_ros_pkgs

---

## Week 6 Summary Checklist

By Friday, you should have:

- ✅ Installed Gazebo successfully
- ✅ Created multiple SDF world files
- ✅ Loaded humanoid robot into simulation
- ✅ Published and subscribed to joint states via ROS 2
- ✅ Tuned physics parameters for stable simulation
- ✅ Debugged simulation using Gazebo and ROS 2 tools
- ✅ Completed 5 exercises and at least 1 challenge project

**If you've checked all boxes, you're ready for Week 7!**

---

## Next Week: Sensors & Unity

Week 7 focuses on:
- Sensor simulation (LiDAR, depth camera, IMU)
- Visualizing sensor data in RViz
- Unity setup and ROS 2 integration
- Photorealistic rendering
- Training dataset export

See you next week! 🚀
