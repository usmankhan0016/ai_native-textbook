# Chapter 4: VLA Control Architecture & Deployment

**Module**: 4 | **Week**: 13 (Part 1) | **Difficulty**: Advanced | **Estimated Time**: 8–10 hours

---

## Learning Objectives

After completing this chapter, you will be able to:

1. **Design and implement** ROS 2 control nodes for navigation (Nav2) and manipulation (MoveIt)
2. **Integrate perception → planning → control** into a unified VLA orchestrator
3. **Implement safety mechanisms** including watchdogs, emergency stops, and collision avoidance
4. **Design deployment strategies** for real hardware including model optimization and latency budgeting
5. **Explain failure recovery mechanisms** and replanning for robust task execution
6. **Deploy optimized VLA systems** to resource-constrained platforms (Jetson)

---

## Key Concepts

- **ROS 2 Action Servers**: Asynchronous, goal-oriented communication for long-running tasks (navigation, manipulation)
- **Nav2**: ROS 2 navigation stack providing path planning and autonomous mobile manipulation
- **MoveIt**: Motion planning library for arm manipulation with collision avoidance
- **VLA Orchestrator**: Central node that coordinates perception, planning, and action
- **Behavior Trees**: Hierarchical task control with conditional logic and error recovery
- **Safety Watchdogs**: Monitoring systems that enforce timing constraints and kill switches
- **Deployment Optimization**: Model quantization, TensorRT inference, edge processing

---

## Part 1: ROS 2 Control Fundamentals

### Action Servers

ROS 2 actions provide **goal-oriented communication** for long-running tasks:

```
Client (LLM Planner) → Goal → Action Server (Navigator/Manipulator) → Feedback/Result
```

**Key differences from topics/services**:
- **Topics**: One-way streaming (no synchronization)
- **Services**: Request/response (synchronous, blocks)
- **Actions**: Goal → feedback loop → result (asynchronous)

**Example: Navigation Action**

```python
# File: navigation_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')

        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute_callback
        )

        self.get_logger().info('Navigation Action Server started')

    def execute_callback(self, goal_handle):
        """Execute navigation to target pose."""
        goal = goal_handle.request
        self.get_logger().info(f'Received goal: {goal.pose.pose.position}')

        # Simulate navigation (in real implementation, call Nav2)
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result(error_code=1)

            # Publish feedback
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 1.0 - (i / 10.0)  # Decrease distance
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}')
            import time
            time.sleep(0.5)

        # Return result
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.error_code = 0  # Success
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

### Nav2 Navigation Stack

Nav2 provides production-ready autonomous navigation:

```python
# File: nav2_client.py
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class Nav2Client:
    def __init__(self, node):
        self._action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, x, y, theta, frame_id='map'):
        """
        Navigate to target pose.

        Args:
            x, y, theta: Target position and orientation
            frame_id: Coordinate frame
        """
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        return future
```

### MoveIt Manipulation

MoveIt enables arm motion planning with collision avoidance:

```python
# File: moveit_manipulation.py
from moveit_commander import MoveGroupCommander, RobotCommander
import geometry_msgs.msg as geometry_msgs

class ManipulationPlanner:
    def __init__(self):
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander('right_arm')

        # Set planning constraints
        self.arm_group.set_goal_position_tolerance(0.01)  # 1cm
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_max_velocity_scaling_factor(0.5)

    def plan_to_position(self, x, y, z):
        """Plan arm motion to 3D position."""
        target_pose = geometry_msgs.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        # Set orientation (gripper points down)
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()

        return plan

    def execute_plan(self, plan):
        """Execute planned motion."""
        self.arm_group.execute(plan, wait=True)

    def open_gripper(self, gripper_force=0.0):
        """Open gripper."""
        # Command gripper to open (0 force = fully open)
        pass

    def close_gripper(self, gripper_force=1.0):
        """Close gripper with specified force."""
        # Command gripper to close with force
        pass
```

---

## Part 2: VLA Pipeline Integration

### Complete VLA Orchestrator

The orchestrator coordinates perception → planning → control:

```python
# File: vla_orchestrator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import time

class VLAOrchestrator(Node):
    """
    Main VLA system orchestrator that:
    1. Receives voice commands
    2. Runs perception pipeline
    3. Calls LLM for task decomposition
    4. Executes ROS 2 actions (Nav2, MoveIt)
    5. Monitors execution and replans on failure
    """

    def __init__(self):
        super().__init__('vla_orchestrator')

        # Subscribe to inputs
        self.command_subscription = self.create_subscription(
            String, '/user/voice_command', self.command_callback, 10
        )

        # State tracking
        self.current_task = None
        self.execution_state = 'idle'  # idle, executing, failed

        # Initialize sub-components
        self.nav_client = self._create_nav_client()
        self.manipulation = self._create_manipulation_client()
        self.llm = self._create_llm_client()

        self.get_logger().info('VLA Orchestrator initialized')

    def command_callback(self, msg):
        """Process user command."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        try:
            # 1. Get scene understanding from perception
            scene = self._get_scene_understanding()

            # 2. Call LLM to decompose task
            task_plan = self._decompose_task(command, scene)

            # 3. Execute task plan with error recovery
            self._execute_task_plan(task_plan)

            self.get_logger().info('Task completed successfully')

        except Exception as e:
            self.get_logger().error(f'Task failed: {e}')
            self.execution_state = 'failed'

    def _get_scene_understanding(self):
        """Get current scene from perception module."""
        # Wait for latest perception message
        # (In real implementation, subscribe and cache)
        scene = {
            'objects': [
                {'id': 'cup_0', 'class': 'cup', 'position': [0.5, 0.3, 0.8]},
                {'id': 'table_0', 'class': 'table', 'position': [0.5, 0.0, 0.0]}
            ]
        }
        return scene

    def _decompose_task(self, command, scene):
        """Call LLM to decompose task."""
        prompt = f"""
        Robot scene: {json.dumps(scene)}
        User command: {command}
        Decompose into robot skills.
        """

        # Call LLM API (mock for now)
        task_plan = {
            'steps': [
                {'skill': 'search', 'params': {'object': 'red cup'}},
                {'skill': 'moveto', 'params': {'position': [0.5, 0.3]}},
                {'skill': 'pick', 'params': {'gripper_force': 'moderate'}},
                {'skill': 'moveto', 'params': {'position': [0.8, 1.0]}},
                {'skill': 'place', 'params': {'release_speed': 'slow'}}
            ]
        }
        return task_plan

    def _execute_task_plan(self, task_plan):
        """Execute task plan step-by-step."""
        for i, step in enumerate(task_plan['steps']):
            skill = step['skill']
            params = step['params']

            self.get_logger().info(f'Step {i+1}: {skill} {params}')

            try:
                # Execute skill
                if skill == 'moveto':
                    self._execute_moveto(params['position'])
                elif skill == 'pick':
                    self._execute_pick(params.get('gripper_force', 'moderate'))
                elif skill == 'place':
                    self._execute_place(params.get('release_speed', 'moderate'))
                elif skill == 'search':
                    self._execute_search(params['object'])

                time.sleep(0.1)  # Brief delay between steps

            except Exception as e:
                self.get_logger().error(f'Step {i+1} failed: {e}')
                # Trigger replanning (simplified)
                self.get_logger().info('Replanning...')
                raise

    def _execute_moveto(self, position):
        """Execute navigation."""
        self.get_logger().info(f'Navigating to {position}')
        # Call Nav2 action client

    def _execute_pick(self, gripper_force):
        """Execute grasping."""
        self.get_logger().info(f'Picking with force {gripper_force}')
        # Call MoveIt + gripper

    def _execute_place(self, release_speed):
        """Execute placement."""
        self.get_logger().info(f'Placing with speed {release_speed}')
        # Call MoveIt + gripper

    def _execute_search(self, object_name):
        """Execute search (perception + navigation)."""
        self.get_logger().info(f'Searching for {object_name}')
        # Spin in place, capture images, detect object

    def _create_nav_client(self):
        # Create Nav2 client
        return None

    def _create_manipulation_client(self):
        # Create MoveIt client
        return None

    def _create_llm_client(self):
        # Create LLM API client
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VLAOrchestrator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

---

## Part 3: Safety & Failure Recovery

### Safety Mechanisms

**Watchdogs**: Monitor control loop timing and enforce emergency stops.

```python
# File: safety_watchdog.py
import rclpy
from rclpy.node import Node
import time
import threading

class SafetyWatchdog(Node):
    """Monitor robot safety and enforce watchdog constraints."""

    def __init__(self, timeout_seconds=2.0):
        super().__init__('safety_watchdog')

        self.timeout = timeout_seconds
        self.last_heartbeat = time.time()
        self.emergency_stop_active = False

        # Subscription to monitor activity
        self.monitor_subscription = self.create_subscription(
            String, '/vla/status', self.status_callback, 10
        )

        # Start watchdog thread
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def status_callback(self, msg):
        """Update heartbeat on activity."""
        self.last_heartbeat = time.time()

    def _watchdog_loop(self):
        """Watchdog loop: check for timeout."""
        while True:
            time.sleep(0.1)

            elapsed = time.time() - self.last_heartbeat
            if elapsed > self.timeout and not self.emergency_stop_active:
                self.get_logger().warning(f'Watchdog timeout after {elapsed:.2f}s')
                self._activate_emergency_stop()

    def _activate_emergency_stop(self):
        """Activate emergency stop (halt all motors)."""
        self.emergency_stop_active = True
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        # Publish emergency stop command to all actuators
```

### Failure Recovery

**Replanning**: When a skill fails, generate an alternative plan.

```python
def execute_with_replanning(task_plan, max_retries=2):
    """Execute task plan with replanning on failure."""
    for step_idx, step in enumerate(task_plan['steps']):
        retries = 0

        while retries < max_retries:
            try:
                execute_skill(step)
                break  # Success, move to next step

            except SkillFailureException as e:
                retries += 1
                logger.warning(f"Step {step_idx} failed: {e}")

                if retries < max_retries:
                    # Attempt to replan from this point
                    logger.info(f"Replanning from step {step_idx}...")
                    remaining_steps = task_plan['steps'][step_idx:]
                    replanned = replan_from_failure(remaining_steps, current_state)
                    task_plan['steps'] = task_plan['steps'][:step_idx] + replanned
                else:
                    raise  # Give up after max retries
```

---

## Part 4: Deployment & Optimization

### Deployment Design Considerations

For physical robot deployment:

1. **Model Optimization**:
   - Quantization (FP32 → INT8): 4x memory reduction, minimal accuracy loss
   - Pruning: Remove unused weights (~30-40% size reduction)
   - Distillation: Train smaller model from larger model

2. **Inference Optimization**:
   - TensorRT (NVIDIA): Optimized inference engine (~2-3x speedup)
   - ONNX: Cross-platform model format with optimizations
   - Edge devices: Jetson platforms provide GPU-accelerated inference

3. **Latency Budgeting**:
   - Perception: `<1`00ms` (YOLO inference)
   - Planning: `<1`s` (LLM reasoning)
   - Control: `<5`0ms` (ROS 2 action execution)
   - Total cycle: `<3`3ms` for real-time control (~30 FPS)

**Example Deployment Profile** (Jetson Orin Nano):
- **Perception**: YOLO + depth (100ms, GPU)
- **LLM**: Cloud API call (1-2s latency acceptable for planning)
- **Control**: ROS 2 action (10-20ms per step, CPU)
- **Total cycle time**: ~2.5 seconds per high-level command (acceptable for household tasks)

### Jetson Deployment

```bash
# Install ROS 2 on Jetson
sudo apt-get update
sudo apt-get install -y ros-humble-desktop

# Install perception libraries
pip install ultralytics opencv-python torch torchvision

# Install control libraries
pip install rclpy MoveIt2 Nav2

# Deploy VLA system
# Copy ROS 2 packages and configuration to Jetson
# Run: ros2 launch vla_system full_pipeline.launch.py
```

---

## Labs & Exercises

### Lab 1: Build Perception-Aware Navigation (1.5 hours)

**Objective**: Navigate while avoiding obstacles detected by perception.

**Example code structure**:
```python
class PerceptionAwareNavigator:
    def navigate_avoiding_obstacles(self, goal_position, obstacle_threshold=0.3):
        # Get perception data
        scene = self.get_scene()

        # Check for obstacles in path
        obstacles = [obj for obj in scene['objects'] if obj['distance'] < obstacle_threshold]

        if obstacles:
            # Replan around obstacles
            alternative_path = self.compute_alternative_path(goal_position, obstacles)
            return self.nav_client.navigate(alternative_path)
        else:
            return self.nav_client.navigate(goal_position)
```

### Lab 2: Integrate LLM Planning with Control (1.5 hours)

**Objective**: Connect LLM task decomposition to ROS 2 execution.

**Evaluation**: Successfully execute a multi-step household task from voice command.

### Lab 3: Implement Safety Watchdogs (1 hour)

**Objective**: Build a safety monitoring system that halts execution on timeout.

**Requirements**:
- Monitor control loop frequency
- Detect missed deadlines
- Activate emergency stop on failure

### Lab 4: Design Deployment Optimization Strategy (1.5 hours)

**Objective**: Analyze latency, create optimization plan, estimate performance gains.

**Deliverables**:
- Latency breakdown (perception, planning, control)
- Optimization strategy (which components to quantize/optimize)
- Estimated speedups

---

## Capstone Integration

This chapter brings together **all VLA pillars**: perception (Ch 2), planning (Ch 3), and control (Ch 4). Your capstone project will:

1. Deploy a complete VLA system in simulation
2. Execute household tasks from natural language commands
3. Demonstrate replanning and failure recovery
4. Discuss deployment to physical hardware

---

## Summary

VLA control architecture requires tight integration of perception, planning, and action. Key takeaways:

- **ROS 2 actions** provide goal-oriented communication for long-running tasks
- **Nav2 + MoveIt** offer production-ready navigation and manipulation
- **VLA orchestrators** coordinate all components into seamless task execution
- **Safety watchdogs** and **replanning** enable robust, fault-tolerant behavior
- **Deployment optimization** makes VLA practical on resource-constrained platforms

---

## References

1. Macenski, S., et al. (2020). "Nav2: A Customizable Autonomous Navigation System." arXiv:2010.04738
2. Sucan, I., & Chitta, S. (2023). "MoveIt2: A Modern C++ Motion Planning Framework." IEEE RA-L
3. Sierra, B., et al. (2024). "Jetson Deployment Best Practices." NVIDIA Developer Blog

---

**Next Section**: [Week 13: Capstone Sprint & Final Integration](./week-13.md)
