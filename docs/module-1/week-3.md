---
sidebar_position: 6
title: "Week 3 Practice Guide"
tags: [Week3, Practice, ROS2, Nodes, Topics]
difficulty: Beginner
module: 1
week: 3
estimated_time: "8-10 hours"
topics: ["ROS 2 setup", "Nodes", "Topics", "CLI tools", "Publisher/Subscriber"]
---

# Week 3 Practice Guide: ROS 2 Fundamentals

**Focus**: ROS 2 concepts, architecture, and communication patterns (Chapters 1-2)

---

## Week Overview

This week, you'll build a solid foundation in ROS 2 by:
- Understanding what ROS 2 is and why it's used for humanoid robotics
- Writing your first ROS 2 nodes
- Implementing publisher/subscriber communication
- Mastering ROS 2 command-line tools

**Time Allocation**: 8-10 hours total
- 📖 **Reading**: 3-4 hours (Chapters 1-2)
- 💻 **Hands-on Practice**: 4-5 hours (exercises below)
- 🎯 **Challenge Projects**: 1-2 hours

---

## Learning Goals

By the end of Week 3, you should be able to:

✅ Explain what ROS 2 is and its key differences from ROS 1
✅ Create and run simple ROS 2 nodes using rclpy
✅ Implement topic-based communication (publishers and subscribers)
✅ Use `ros2` CLI tools to inspect running systems
✅ Understand Quality of Service (QoS) policies
✅ Debug common ROS 2 issues

---

## Day-by-Day Breakdown

### Day 1 (Monday): ROS 2 Introduction and Setup
**Duration**: 1.5-2 hours
**Goal**: Understand ROS 2 and verify your development environment

#### Tasks
1. **Read**: [Chapter 1: Introduction to ROS 2](./chapter-1-introduction-to-ros2.md)
   - Focus on sections 1.1-1.3 (What is ROS 2, Why for humanoids, ROS 1 vs ROS 2)

2. **Verify Installation**:
   ```bash
   # Check ROS 2 Humble is installed
   ros2 --version
   # Expected: ros2 cli version: 0.18.x

   # Source ROS 2 (add to ~/.bashrc for persistence)
   source /opt/ros/humble/setup.bash

   # Verify with demo nodes
   ros2 run demo_nodes_py talker
   # Open new terminal:
   ros2 run demo_nodes_py listener
   ```

3. **Explore ROS 2 CLI**:
   ```bash
   # List running nodes
   ros2 node list

   # List active topics
   ros2 topic list

   # Echo messages on /chatter topic
   ros2 topic echo /chatter

   # Get topic info
   ros2 topic info /chatter
   ```

4. **Exercise**: Run the turtlesim demo and practice CLI commands
   ```bash
   ros2 run turtlesim turtlesim_node
   # New terminal:
   ros2 run turtlesim turtle_teleop_key

   # In another terminal, inspect the system:
   ros2 node list
   ros2 topic list
   ros2 topic echo /turtle1/cmd_vel
   ```

**Success Criteria**: You can start demo nodes, see topics being published, and understand the ROS 2 graph.

---

### Day 2 (Tuesday): Your First ROS 2 Node
**Duration**: 2-2.5 hours
**Goal**: Write and run your first custom ROS 2 node

#### Tasks
1. **Read**: Chapter 2, Section 2.1 (Nodes and the Computation Graph)

2. **Create Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

3. **Create Package**:
   ```bash
   ros2 pkg create my_first_robot \
     --build-type ament_python \
     --dependencies rclpy std_msgs
   ```

4. **Write Minimal Node** (see Chapter 2, Section 2.2):
   ```python
   # File: ~/ros2_ws/src/my_first_robot/my_first_robot/hello_ros2.py

   import rclpy
   from rclpy.node import Node

   class HelloROS2(Node):
       def __init__(self):
           super().__init__('hello_ros2')
           self.counter = 0
           self.timer = self.create_timer(1.0, self.timer_callback)
           self.get_logger().info('Hello ROS 2 node started!')

       def timer_callback(self):
           self.counter += 1
           self.get_logger().info(f'Hello ROS 2! Count: {self.counter}')

   def main(args=None):
       rclpy.init(args=args)
       node = HelloROS2()

       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

5. **Update setup.py**:
   ```python
   entry_points={
       'console_scripts': [
           'hello_ros2 = my_first_robot.hello_ros2:main',
       ],
   },
   ```

6. **Build and Run**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_first_robot
   source install/setup.bash
   ros2 run my_first_robot hello_ros2
   ```

**Success Criteria**: Your node runs and prints messages every second. You can see it in `ros2 node list`.

---

### Day 3 (Wednesday): Publisher and Subscriber
**Duration**: 2-2.5 hours
**Goal**: Implement topic-based communication

#### Tasks
1. **Read**: Chapter 2, Section 2.2 (Topics and Messages)

2. **Create Publisher Node** (Talker):
   ```python
   # File: ~/ros2_ws/src/my_first_robot/my_first_robot/robot_talker.py

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class RobotTalker(Node):
       def __init__(self):
           super().__init__('robot_talker')
           self.publisher_ = self.create_publisher(String, '/robot_status', 10)
           self.timer = self.create_timer(0.5, self.publish_status)
           self.count = 0

       def publish_status(self):
           msg = String()
           msg.data = f'Robot status: Active (count: {self.count})'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Published: "{msg.data}"')
           self.count += 1

   def main(args=None):
       rclpy.init(args=args)
       node = RobotTalker()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

3. **Create Subscriber Node** (Listener):
   ```python
   # File: ~/ros2_ws/src/my_first_robot/my_first_robot/robot_listener.py

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class RobotListener(Node):
       def __init__(self):
           super().__init__('robot_listener')
           self.subscription = self.create_subscription(
               String,
               '/robot_status',
               self.status_callback,
               10
           )

       def status_callback(self, msg):
           self.get_logger().info(f'Received: "{msg.data}"')

   def main(args=None):
       rclpy.init(args=args)
       node = RobotListener()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

4. **Add Entry Points and Build**:
   ```python
   # In setup.py:
   entry_points={
       'console_scripts': [
           'hello_ros2 = my_first_robot.hello_ros2:main',
           'robot_talker = my_first_robot.robot_talker:main',
           'robot_listener = my_first_robot.robot_listener:main',
       ],
   },
   ```

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_first_robot
   source install/setup.bash
   ```

5. **Test Communication**:
   ```bash
   # Terminal 1: Start talker
   ros2 run my_first_robot robot_talker

   # Terminal 2: Start listener
   ros2 run my_first_robot robot_listener

   # Terminal 3: Inspect topics
   ros2 topic list
   ros2 topic info /robot_status
   ros2 topic echo /robot_status
   ros2 topic hz /robot_status
   ```

**Success Criteria**: Listener receives all messages from talker. `ros2 topic hz` shows ~2 Hz publish rate.

---

### Day 4 (Thursday): Services and QoS
**Duration**: 2-2.5 hours
**Goal**: Learn request-response patterns and Quality of Service

#### Tasks
1. **Read**: Chapter 2, Sections 2.3 (Services) and 2.5 (QoS)

2. **Create Service Server**:
   ```python
   # File: ~/ros2_ws/src/my_first_robot/my_first_robot/add_service.py

   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import AddTwoInts

   class AddService(Node):
       def __init__(self):
           super().__init__('add_service')
           self.service = self.create_service(
               AddTwoInts,
               '/add_two_ints',
               self.add_callback
           )
           self.get_logger().info('Add service ready')

       def add_callback(self, request, response):
           response.sum = request.a + request.b
           self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
           return response

   def main(args=None):
       rclpy.init(args=args)
       node = AddService()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   ```

3. **Test Service with CLI**:
   ```bash
   # Build and run server
   cd ~/ros2_ws
   colcon build --packages-select my_first_robot
   source install/setup.bash
   ros2 run my_first_robot add_service

   # In another terminal, call the service:
   ros2 service list
   ros2 service type /add_two_ints
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
   # Expected: sum: 8
   ```

4. **Experiment with QoS**:
   ```python
   # Modify robot_talker.py to use BEST_EFFORT QoS
   from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

   qos_profile = QoSProfile(
       reliability=QoSReliabilityPolicy.BEST_EFFORT,
       history=QoSHistoryPolicy.KEEP_LAST,
       depth=1
   )

   self.publisher_ = self.create_publisher(String, '/robot_status', qos_profile)
   ```

   Rebuild and observe message delivery behavior.

**Success Criteria**: Service responds correctly. You understand the difference between RELIABLE and BEST_EFFORT QoS.

---

### Day 5 (Friday): Actions and System Integration
**Duration**: 1.5-2 hours
**Goal**: Understand actions for long-running tasks

#### Tasks
1. **Read**: Chapter 2, Section 2.4 (Actions)

2. **Test Built-in Action** (Fibonacci example from Chapter 2):
   ```bash
   # Terminal 1: Run action server
   ros2 run action_tutorials_py fibonacci_action_server

   # Terminal 2: Send goal
   ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

   # Observe feedback and final result
   ```

3. **Inspect Running System**:
   ```bash
   ros2 action list
   ros2 action info /fibonacci
   ros2 interface show action_tutorials_interfaces/action/Fibonacci
   ```

4. **Complete Chapter 2 Exercises**:
   - Exercise 1: Multi-topic publisher (Easy)
   - Exercise 2: Service with validation (Easy-Medium)

**Success Criteria**: You can run action servers/clients and understand when to use topics vs services vs actions.

---

## Hands-On Exercises

Complete at least **3 of these 5 exercises** to reinforce your learning:

### Exercise 1: Humanoid Joint State Publisher (Easy)
**Goal**: Simulate a humanoid robot publishing joint states

Create a node that publishes joint positions for a humanoid:
- Topic: `/joint_states`
- Message type: `sensor_msgs/msg/JointState`
- Joints: `['head_pan', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow']`
- Publish at 50 Hz

**Acceptance**: `ros2 topic hz /joint_states` shows ~50 Hz

---

### Exercise 2: Sensor Data Logger (Easy-Medium)
**Goal**: Subscribe to a topic and log data to a file

Create a node that:
- Subscribes to `/robot_status` from Day 3
- Writes received messages to `robot_log.txt` with timestamps
- Logs at least 100 messages, then exits

**Acceptance**: File contains 100+ timestamped messages

---

### Exercise 3: Multi-Sensor Fusion (Medium)
**Goal**: Subscribe to multiple topics and combine data

Create a node that subscribes to:
- `/camera/image` (simulated with String messages)
- `/imu/data` (simulated with String messages)
- `/lidar/scan` (simulated with String messages)

Combine and republish to `/sensor_fusion/status` every 1 second.

**Acceptance**: Fusion node receives from all 3 topics and publishes combined status

---

### Exercise 4: Parameter-Based Publisher (Medium)
**Goal**: Create a configurable publisher using ROS 2 parameters

Node should:
- Accept parameters: `topic_name`, `publish_rate`, `message_prefix`
- Publish String messages to the specified topic
- Update behavior when parameters change

**Acceptance**: Can change `publish_rate` at runtime with `ros2 param set`

---

### Exercise 5: Emergency Stop Service (Medium-Hard)
**Goal**: Implement a safety service for a robot

Create two nodes:
1. **Robot Controller**: Publishes `/robot/velocity` at 10 Hz
2. **Emergency Stop Service**: Service `/emergency_stop` that stops the robot

When service is called:
- Controller stops publishing velocity
- Publishes `/robot/status` = "STOPPED"

**Acceptance**: Calling service stops velocity publications

---

## Challenge Projects

Pick **one challenge project** to test your full understanding:

### Challenge 1: Humanoid Head Tracker (Beginner)
Build a simulated head tracking system:
- **Input**: `/target_position` (geometry_msgs/Point)
- **Output**: `/head_pan_tilt` (Float64MultiArray with 2 values)
- **Logic**: Convert XYZ target to pan/tilt angles
- **Constraints**: Pan: ±90°, Tilt: ±45°

**Bonus**: Visualize with `rqt_plot`

---

### Challenge 2: Multi-Node Communication System (Intermediate)
Create a 4-node system:
1. **Sensor Node**: Publishes simulated sensor data (temperature, humidity)
2. **Processor Node**: Subscribes to sensors, computes averages
3. **Decision Node**: Subscribes to averages, publishes commands
4. **Actuator Node**: Subscribes to commands, logs actions

**Bonus**: Add QoS profiles (BEST_EFFORT for sensors, RELIABLE for commands)

---

### Challenge 3: Simple Finite State Machine (Advanced)
Implement a robot state machine with states:
- `IDLE` → `MOVING` → `WORKING` → `IDLE`

Transitions:
- `/start` service: IDLE → MOVING
- `/work` service: MOVING → WORKING
- `/stop` service: Any state → IDLE

Publish current state to `/robot/state` at 5 Hz.

**Bonus**: Add timeout transitions (auto-return to IDLE after 30s)

---

## Weekly Checklist

Use this checklist to track your progress:

- [ ] Completed Chapter 1 reading
- [ ] Completed Chapter 2 reading
- [ ] Created first ROS 2 package
- [ ] Wrote and ran a minimal node
- [ ] Implemented publisher/subscriber communication
- [ ] Created and tested a service
- [ ] Experimented with QoS policies
- [ ] Tested action servers/clients
- [ ] Completed at least 3 hands-on exercises
- [ ] Completed 1 challenge project
- [ ] Can explain nodes, topics, services, actions to someone else

---

## Debugging Tips

Common issues this week:

### Issue: "Package not found"
```bash
# Solution: Source your workspace
source ~/ros2_ws/install/setup.bash
# Add to ~/.bashrc for persistence
```

### Issue: "No executable found"
```bash
# Solution: Check entry_points in setup.py
# Rebuild: colcon build --packages-select <pkg>
```

### Issue: "Topic not receiving messages"
```bash
# Check if publisher/subscriber are running:
ros2 node list

# Check topic name matches exactly:
ros2 topic list

# Verify message type compatibility:
ros2 topic info /topic_name
```

### Issue: QoS incompatibility
```bash
# Error: "Could not find a matching QoS profile"
# Solution: Publisher and subscriber must have compatible QoS
# Either both RELIABLE or both BEST_EFFORT
```

---

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 CLI Cheat Sheet](https://github.com/ros2/ros2cli/blob/rolling/ros2cli/doc/cheat_sheet.rst)
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Understanding ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

## Next Week Preview

**Week 4** focuses on:
- Creating professional ROS 2 packages
- Writing launch files to orchestrate multiple nodes
- Managing parameters with YAML files
- Integrating with Gazebo simulation

Get ready to build more complex systems! 🚀

---

**Questions or stuck?** Review the chapter exercises or check the ROS 2 documentation. Practice is key to mastering ROS 2!
