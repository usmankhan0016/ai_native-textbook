---
sidebar_position: 2
sidebar_label: "Ch1: Introduction"
title: "Chapter 1: Introduction to ROS 2"
tags: [ROS2, Fundamentals, Beginner, Week3]
difficulty: Beginner
module: 1
week: 3
prerequisites: ["Ubuntu 22.04", "Python basics", "Command line"]
estimated_time: "2-3 hours"
topics: ["ROS 2 architecture", "DDS middleware", "ROS 1 vs ROS 2", "Simulator integration"]
---

# Chapter 1: Introduction to ROS 2

## Learning Objectives

After completing this chapter, you will be able to:

1. **Explain** what ROS 2 is and its critical role in modern humanoid robotics systems
2. **Compare** ROS 2 and ROS 1, highlighting key architectural improvements
3. **Describe** how ROS 2 integrates with simulation platforms (Gazebo, Unity, NVIDIA Isaac)
4. **Identify** appropriate ROS 2 distributions for educational and production use
5. **Recognize** real-world humanoid robots using ROS 2 in industry and research

## Key Concepts

Before diving deep, here are the core ideas you'll encounter in this chapter:

1. **ROS 2 as Middleware**: ROS 2 is not an operating system—it's a middleware layer that sits between your robot's hardware/sensors and your high-level control algorithms, facilitating communication and coordination.

2. **DDS (Data Distribution Service)**: ROS 2 uses DDS as its underlying communication protocol, enabling real-time, reliable, and scalable data exchange across distributed robot systems.

3. **Publish-Subscribe Architecture**: The core communication pattern in ROS 2 where components (nodes) publish data to topics that other nodes subscribe to, creating a decoupled, flexible system.

4. **Multi-Platform Support**: Unlike ROS 1 (Linux-only), ROS 2 works on Linux, Windows, and macOS, making it accessible for diverse development environments.

5. **Real-Time Capable**: ROS 2 is designed for real-time robotics applications with deterministic communication and Quality of Service (QoS) policies.

6. **Ecosystem Integration**: ROS 2 seamlessly connects with modern simulation engines (Gazebo, Unity, Isaac Sim), enabling digital twin workflows and sim-to-real transfer.

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is an open-source middleware framework designed to simplify the development of complex robotic systems. Despite its name, ROS 2 is **not an operating system** like Linux or Windows. Instead, it's a collection of libraries, tools, and conventions that provide:

- **Communication infrastructure**: Enable different parts of your robot (sensors, actuators, decision-making algorithms) to talk to each other
- **Standard interfaces**: Common message formats for sensors (cameras, LiDAR, IMUs) and actuators (motors, grippers)
- **Build system**: Tools to compile and manage multi-package robot projects
- **Debugging tools**: Visualization (RViz), logging, and introspection utilities
- **Community ecosystem**: Thousands of open-source packages for navigation, manipulation, perception, and more

### Why ROS 2 for Humanoid Robotics?

Humanoid robots are among the most complex robotic systems, with:

- **20-30+ degrees of freedom** (joints in legs, arms, torso, head)
- **Multiple sensor modalities** (cameras, depth sensors, IMUs, force-torque sensors, tactile sensors)
- **Real-time control requirements** (balance control at 100+ Hz, vision processing at 30+ Hz)
- **Coordination across subsystems** (locomotion, manipulation, perception, planning)

ROS 2 addresses these challenges by providing:

1. **Modular Architecture**: Break down complex humanoid control into independent nodes (e.g., balance controller, arm motion planner, vision processor) that communicate via topics and services

2. **Scalability**: Run compute-intensive tasks (like deep learning for object recognition) on powerful GPUs while running real-time controllers on embedded hardware—all seamlessly connected via ROS 2

3. **Simulation Integration**: Develop and test humanoid behaviors in realistic simulators (Gazebo, Unity, NVIDIA Isaac) before deploying to hardware, saving time and preventing damage

4. **Community Support**: Leverage existing packages for humanoid-specific tasks (footstep planning, whole-body control, human-robot interaction)

## ROS 1 vs ROS 2: What Changed and Why?

If you've heard of ROS 1 (sometimes called "ROS Melodic" or "ROS Noetic"), you might wonder why ROS 2 exists. Here are the key differences:

### Architectural Improvements

| Aspect | ROS 1 | ROS 2 | Why It Matters for Humanoids |
|--------|-------|-------|------------------------------|
| **Communication** | Custom TCPROS/UDPROS | DDS (Data Distribution Service) | DDS is an industry standard with proven real-time performance |
| **Master Node** | Required centralized master | No master (peer-to-peer) | No single point of failure—critical for safety |
| **Real-Time Support** | Limited | Built-in QoS policies | Balance control and reactive behaviors need deterministic timing |
| **Multi-Platform** | Linux only | Linux, Windows, macOS | Develop on Windows/Mac, deploy to Linux robots |
| **Security** | Minimal | DDS Security (SROS2) | Protect robots from unauthorized control |
| **Discovery** | XML-RPC master | DDS discovery | Automatic node discovery without centralized broker |

### Key Differences Explained

**1. No Master Node**

In ROS 1, a master node (`roscore`) had to run for any communication to work. If it crashed, your entire robot system failed. ROS 2 eliminates this single point of failure—nodes discover each other automatically via DDS's peer-to-peer discovery.

**2. DDS Middleware**

ROS 2 doesn't implement its own communication protocol. Instead, it uses **DDS (Data Distribution Service)**, a mature standard used in aerospace, defense, and industrial automation. This means:
- Proven reliability for mission-critical systems
- Better performance for large data (e.g., camera streams)
- Built-in Quality of Service (QoS) configuration

**3. Quality of Service (QoS) Policies**

ROS 2 lets you specify how data should be transmitted:
- **Reliability**: Reliable (guaranteed delivery) vs Best-Effort (allow message loss for low latency)
- **Durability**: Transient-Local (late subscribers receive historical data) vs Volatile (only current data)
- **History**: Keep-Last-N messages vs Keep-All

For humanoids, this is critical:
- Use **reliable** QoS for joint commands (can't lose motor control messages!)
- Use **best-effort** QoS for camera streams (latest frame is more important than every frame)

**4. Real-Time Capabilities**

While ROS 1 could be used with real-time operating systems, ROS 2 is designed from the ground up for real-time:
- Compatible with real-time Linux kernels (PREEMPT_RT)
- Executor model supports deterministic callback scheduling
- Memory management optimized to avoid dynamic allocation in critical paths

:::tip[When to Use ROS 1 vs ROS 2]

**Use ROS 2 if**:
- Starting a new robotics project (recommended!)
- Need Windows/Mac development support
- Require real-time guarantees for safety-critical systems
- Building commercial or production robots

**Use ROS 1 if**:
- Maintaining legacy codebases
- Need packages that haven't been ported to ROS 2 yet (increasingly rare)
- Working in environments that only support ROS 1

For this course, **we exclusively use ROS 2 Humble**, the latest long-term support (LTS) release.
:::

## Integration with Simulation Platforms

One of ROS 2's greatest strengths is its seamless integration with modern simulation engines. This enables **digital twin** workflows where you develop and test robot behaviors in simulation before deploying to hardware.

### Gazebo (Gazebo Garden/Harmonic)

**Gazebo** is an open-source 3D robotics simulator with physics engines (ODE, Bullet, Simbody) for realistic simulation.

**Why use Gazebo with ROS 2?**
- **Physics-accurate**: Simulate gravity, collisions, friction, joint dynamics
- **Sensor simulation**: Virtual cameras, LiDAR, depth sensors, IMUs produce realistic data
- **Seamless ROS 2 integration**: `ros_gz_bridge` connects Gazebo topics to ROS 2 topics
- **Free and open-source**: No licensing costs, large community

**Typical workflow**:
1. Create URDF (Unified Robot Description Format) file describing your humanoid's structure
2. Spawn robot in Gazebo world
3. ROS 2 nodes send motor commands → Gazebo simulates physics → sensors publish data back to ROS 2
4. Develop controllers, test walking gaits, debug perception—all in simulation

### Unity and Unity Robotics Hub

**Unity** is a professional game engine with photorealistic rendering, ideal for vision-based AI training.

**Why use Unity with ROS 2?**
- **Photorealistic rendering**: Train vision models (object detection, semantic segmentation) on realistic scenes
- **Synthetic data generation**: Generate labeled training data faster than manual annotation
- **High-performance physics**: Unity's PhysX engine for complex interactions
- **Unity Robotics Hub**: Official ROS 2 integration via TCP connection

**Typical workflow**:
1. Design humanoid robot and environment in Unity
2. Use `ROS-TCP-Connector` to bridge Unity and ROS 2
3. Publish sensor data (RGB cameras, depth) from Unity to ROS 2
4. ROS 2 vision algorithms process data and send control commands back

### NVIDIA Isaac Sim

**NVIDIA Isaac Sim** is a GPU-accelerated robotics simulator built on NVIDIA Omniverse.

**Why use Isaac Sim with ROS 2?**
- **GPU acceleration**: Physics, rendering, and sensor simulation all on GPU for massive speed (100x+ faster than real-time)
- **Synthetic data at scale**: Generate millions of labeled images for training perception models
- **Domain randomization**: Automatically vary lighting, textures, physics to create robust models
- **Isaac ROS**: Native ROS 2 integration with GPU-accelerated perception nodes

**Typical workflow**:
1. Import humanoid robot URDF into Isaac Sim
2. Enable Isaac ROS integration for sensor topics
3. Train vision-language-action (VLA) models using synthetic data
4. Deploy trained models to ROS 2 nodes on real robot

:::note[Simulation to Reality Gap]

While simulations are invaluable for development, there's always a "sim-to-real gap"—behaviors that work perfectly in simulation may fail on hardware due to:
- Inaccurate physics models (friction, contact dynamics)
- Sensor noise and latency not captured in simulation
- Actuator limitations (motor torque limits, backlash)

To minimize this gap:
- Use high-fidelity physics engines (Gazebo, Isaac Sim)
- Add realistic sensor noise in simulation
- Validate critical behaviors on hardware early
:::

## ROS 2 Distributions: Which One to Use?

ROS 2 follows a time-based release schedule with new distributions every 6 months. Distributions are named alphabetically after turtles (Humble, Iron, Jazzy...).

### Currently Supported Distributions (as of 2025)

| Distribution | Release Date | Support End | Type | Recommendation |
|--------------|--------------|-------------|------|----------------|
| **Humble Hawksbill** | May 2022 | May 2027 | **LTS** | ✅ **Use for this course** |
| **Iron Irwini** | May 2023 | November 2024 | Non-LTS | Not recommended (support ended) |
| **Jazzy Jalisco** | May 2024 | May 2026 | Non-LTS | Cutting-edge features, less stable |

### Why We Use Humble Hawksbill

**ROS 2 Humble** is a **Long-Term Support (LTS) release** with:
- ✅ **5 years of support** (through May 2027)
- ✅ **Widest package compatibility** (most third-party packages support Humble)
- ✅ **Stable APIs** (less breaking changes compared to non-LTS releases)
- ✅ **Industry adoption** (companies building production robots use LTS releases)
- ✅ **Best documentation** (tutorials and examples primarily target Humble)

**For educational and professional use, always prefer LTS releases.**

### Installation

If you haven't installed ROS 2 Humble yet, follow the official guide:
[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

After installation, verify your setup:

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
# Expected output: ros2 cli version 0.18.x

# Check for core packages
ros2 pkg list | grep rclpy
# Should show rclpy (Python client library)
```

## Real-World Humanoid Robots Using ROS 2

To motivate your learning, let's look at real humanoid robots that use ROS 2 in production and research.

### Unitree G1

**Manufacturer**: Unitree Robotics (China)
**Height**: 1.3m | **Weight**: 35kg | **DOF**: 23-44 (depending on config)

The **Unitree G1** is an affordable humanoid robot designed for research and education, starting at ~$16,000. It uses ROS 2 for:
- Low-level motor control (joint position/velocity/torque commands)
- Sensor fusion (IMU + cameras for localization)
- High-level task planning (manipulation, navigation)

Unitree provides ROS 2 packages (`unitree_ros2`) for direct control, making it popular in academic labs.

### Toyota Human Support Robot (HSR)

**Manufacturer**: Toyota Research Institute (Japan)
**Purpose**: Assistive robotics for elderly care

The **Toyota HSR** is a single-arm mobile manipulator designed to assist people with limited mobility. It uses ROS 2 for:
- Autonomous navigation in home environments
- Object grasping and manipulation (fetching items)
- Human-robot interaction (voice commands, gesture recognition)

Toyota has released ROS 2 simulation packages for the HSR, enabling researchers worldwide to develop applications before accessing the physical robot.

### PAL Robotics TALOS

**Manufacturer**: PAL Robotics (Spain)
**Height**: 1.75m | **Weight**: 95kg | **DOF**: 32

**TALOS** is a high-performance humanoid robot for research in bipedal locomotion and whole-body control. It uses ROS 2 for:
- Whole-body trajectory optimization
- Balance control during dynamic walking
- Bimanual manipulation tasks

TALOS is used by research institutions (LAAS-CNRS, IRI) to advance the state-of-the-art in humanoid control.

### Why These Examples Matter

These robots share common patterns in their ROS 2 usage:
1. **Modular design**: Separate nodes for perception, planning, and control
2. **Standard interfaces**: All use `sensor_msgs` for cameras and IMUs, `geometry_msgs` for velocities
3. **Simulation-first development**: Extensive Gazebo/Isaac Sim testing before hardware trials
4. **Community contributions**: Open-source packages benefit the entire robotics community

By learning ROS 2, you're gaining skills directly applicable to these cutting-edge robots.

## End-of-Chapter Exercises

Test your understanding with these exercises. Solutions and hints are provided at the end.

### Exercise 1: Conceptual Understanding (Easy)

**Question**: In your own words, explain why ROS 2 is called "middleware" rather than an operating system. What role does it play in a robotic system?

**Acceptance Criteria**: Your explanation should mention that ROS 2 sits between the robot's hardware/OS and application logic, facilitating communication.

### Exercise 2: ROS 1 vs ROS 2 (Easy)

**Question**: Name three key advantages of ROS 2 over ROS 1 and explain why each matters for humanoid robotics.

**Acceptance Criteria**: Mention at least three from: no master node (fault tolerance), DDS (real-time), QoS (reliability), multi-platform (dev flexibility).

### Exercise 3: Use Case Selection (Medium)

**Scenario**: You're building a humanoid robot that needs to:
- Maintain balance in real-time (1kHz control loop)
- Process camera streams for object detection (30 fps)
- Navigate using A* path planning (updates every 2 seconds)

**Question**: For each of these tasks, would you use **reliable** or **best-effort** QoS? Justify your choice.

**Acceptance Criteria**: Correct QoS selection with valid reasoning (balance: reliable, camera: best-effort, planning: reliable).

### Exercise 4: Simulation Selection (Medium)

**Question**: You need to:
- (A) Train a vision model on 100,000 synthetic images of humanoids grasping objects
- (B) Test bipedal walking gaits with realistic physics and contact dynamics
- (C) Visualize sensor data from a robot in a photorealistic lab environment

Which simulator (Gazebo, Unity, Isaac Sim) would you choose for each task? Explain why.

**Acceptance Criteria**: Reasonable choices (A: Isaac Sim or Unity for synthetic data, B: Gazebo or Isaac for physics, C: Unity for photorealism).

### Exercise 5: Real-World Application (Medium)

**Research Task**: Choose one of the three humanoid robots mentioned (Unitree G1, Toyota HSR, PAL TALOS) and find:
1. One open-source ROS 2 package related to that robot
2. One research paper or demo video showing its capabilities
3. One specific task the robot can perform that requires ROS 2's features (QoS, real-time, etc.)

**Acceptance Criteria**: Valid GitHub repo, paper/video link, and clear task description.

### Exercise 6: Distribution Selection (Easy)

**Question**: Your company is building a production humanoid robot that will be deployed in 2025 and maintained through 2029. Which ROS 2 distribution would you choose? Why?

**Acceptance Criteria**: Humble (LTS support through 2027, with migration path) or waiting for next LTS. Justification mentions support timeline.

## Capstone Integration

**How This Chapter Connects to the Final Humanoid Project**

In the final capstone project, you'll build a simulated humanoid robot that can:
- Navigate autonomously in a home environment
- Manipulate objects using vision-guided grasping
- Respond to natural language commands via Vision-Language-Action (VLA) models

**Concepts from this chapter that directly apply**:
1. **ROS 2 as Middleware**: Your humanoid will have 10+ nodes (navigation, perception, manipulation, planning) all communicating via ROS 2
2. **Simulation Integration**: You'll develop the robot in Gazebo/Isaac Sim before deploying to hardware (if available)
3. **QoS Policies**: Motor commands will use reliable QoS, camera streams will use best-effort
4. **DDS Discovery**: Your distributed nodes (some on robot hardware, some on GPU workstation) will automatically find each other

## Summary

In this chapter, you learned:

✅ **What ROS 2 is**: A middleware framework that simplifies robotics development by providing communication infrastructure, standard interfaces, and a rich ecosystem

✅ **Why it matters for humanoids**: Modular architecture, scalability, and simulation integration address the unique challenges of complex humanoid systems

✅ **ROS 2 vs ROS 1**: DDS-based communication, no master node, real-time support, and multi-platform compatibility make ROS 2 the future of robotics

✅ **Simulation integration**: Gazebo (physics-accurate), Unity (photorealistic), and Isaac Sim (GPU-accelerated) enable digital twin workflows

✅ **Distribution choice**: ROS 2 Humble is the recommended LTS release with support through 2027

✅ **Real-world examples**: Unitree G1, Toyota HSR, and PAL TALOS demonstrate ROS 2's capabilities in production humanoid robots

## What's Next?

Now that you understand **what** ROS 2 is and **why** it matters, it's time to get hands-on!

In **[Chapter 2: ROS 2 Architecture →](./chapter-2-ros2-architecture.md)**, you'll:
- Write your first ROS 2 node in Python
- Implement publishers and subscribers for topic communication
- Create services for request-response patterns
- Build action servers for long-running humanoid tasks

Let's start coding! 🚀

---

**Additional Resources**:
- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Documentation](https://design.ros2.org/)
- [DDS Specification (OMG)](https://www.omg.org/spec/DDS/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
