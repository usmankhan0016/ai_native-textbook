---
sidebar_position: 5
sidebar_label: "Ch. 5: Jetson Deployment"
title: "Chapter 5: End-to-End Deployment: From Sim to Real Hardware"
tags: ["jetson", "deployment", "tensorrt", "optimization", "safety", "production"]
difficulty: "Intermediate"
module: 3
week: 10
prerequisites: ["Chapter 1-4", "ROS 2", "Docker basics"]
estimated_time: "4-5 hours"
topics: ["Jetson hardware", "model optimization", "real-time control", "safety mechanisms", "deployment", "production"]
---

# Chapter 5: End-to-End Deployment: From Sim to Real Hardware

**Duration**: 4-5 hours | **Difficulty**: Intermediate | **Week**: 10

Deploy your trained AI models to real hardware with safety guarantees.

---

## Learning Objectives

By completing this chapter, you will be able to:

1. **Design Hardware Abstraction Layers** — Write code that runs identically in simulation and on real hardware
2. **Optimize Models for Jetson** — Convert PyTorch models to TensorRT with quantization for 10x speedup
3. **Implement Real-Time Control** — Design latency-critical perception-control loops
4. **Deploy Safety Mechanisms** — Add watchdogs, emergency stops, graceful failure handling
5. **Measure Performance** — Profile deployment on physical hardware
6. **Debug Hardware Issues** — Troubleshoot common deployment problems
7. **Document Procedures** — Write clear deployment guides for production
8. **Handle Hardware Failures** — Implement robust error recovery

---

## Key Concepts

### **Hardware Abstraction Layer (HAL)**
Software interface that isolates robot control code from hardware differences. Same ROS 2 code runs in Isaac Sim and on physical Jetson.

### **Jetson**
NVIDIA's edge AI processor. Models Orin Nano (8GB, $200), Orin AGX (64GB, $2000), Xavier (8GB, ~$300). Combines CPU + GPU + AI accelerators (VIC, DLA).

### **Model Optimization**
Reducing model size/latency without significant accuracy loss. TensorRT enables: quantization (FP32→INT8), operator fusion, memory optimization.

### **Real-Time Latency Budget**
Allocation of time across perception → inference → control. Example: 100 ms total = 30 ms perception + 50 ms inference + 20 ms control.

### **Safety Mechanisms**
Watchdog timers, emergency stops, heartbeat monitoring. Ensure robot fails safely if any component freezes.

### **Graceful Degradation**
Operating with reduced capability when systems fail. Example: If vision fails, use LiDAR as fallback.

---

## Part 1: Model Optimization for Jetson

### Converting PyTorch to TensorRT

```python
# File: optimize_model.py
"""
Convert trained object detection model to TensorRT for Jetson deployment.
"""

import torch
import tensorrt as trt
from torch2trt import torch2trt, TRTModule

def load_trained_model():
    """Load PyTorch model trained in Chapter 4."""
    import torchvision.models as models
    model = models.detection.fasterrcnn_resnet50_fpn(pretrained=False, num_classes=91)
    model.load_state_dict(torch.load("model_synthetic_trained.pt"))
    model.eval()
    model.cuda()
    return model

def convert_to_tensorrt(model, input_shape=(1, 3, 640, 480)):
    """Convert PyTorch model to TensorRT engine."""
    print(f"Converting model to TensorRT...")

    # Create dummy input matching expected shape
    example_input = torch.randn(input_shape).cuda()

    # Convert PyTorch → TensorRT
    # Enables: FP16 inference, operator fusion, memory optimization
    model_trt = torch2trt(
        model,
        [example_input],
        fp16_mode=True,  # 16-bit floating point (2x speedup, minimal accuracy loss)
        max_workspace_size=1 << 32,  # 4 GB workspace for optimization
    )

    return model_trt

def quantize_to_int8(model_trt):
    """Further quantize to INT8 for extreme speed (optional)."""
    # INT8 quantization: 32-bit → 8-bit integers
    # Trade-off: 8x speedup vs. accuracy loss
    print("Note: INT8 quantization requires calibration dataset")
    print("This step is optional - FP16 usually sufficient for real-time")
    # For production use, provide calibration images

def benchmark_model(model_trt, warmup_runs=10, benchmark_runs=100):
    """Benchmark model speed on Jetson."""
    dummy_input = torch.randn(1, 3, 640, 480).cuda()

    # Warmup
    for _ in range(warmup_runs):
        with torch.no_grad():
            _ = model_trt(dummy_input)

    # Benchmark
    torch.cuda.synchronize()
    t_start = time.perf_counter()

    for _ in range(benchmark_runs):
        with torch.no_grad():
            _ = model_trt(dummy_input)

    torch.cuda.synchronize()
    t_end = time.perf_counter()

    latency_ms = (t_end - t_start) / benchmark_runs * 1000
    fps = 1000 / latency_ms

    print(f"\n=== Jetson Inference Benchmark ===")
    print(f"Latency:  {latency_ms:.2f} ms")
    print(f"FPS:      {fps:.1f}")
    print(f"Budget:   30 ms (for 33 FPS target)")
    if latency_ms < 30:
        print("✅ Meets real-time requirement")
    else:
        print("⚠️  Exceeds real-time budget - consider INT8 quantization")

if __name__ == "__main__":
    import time

    # Load trained model from Chapter 4
    model = load_trained_model()
    print(f"Loaded PyTorch model")
    print(f"  Original size: ~200 MB (FP32)")

    # Convert to TensorRT FP16
    model_trt = convert_to_tensorrt(model)
    print(f"Converted to TensorRT FP16")
    print(f"  Optimized size: ~100 MB (50% reduction)")

    # Benchmark on Jetson
    benchmark_model(model_trt)

    # Save optimized model for deployment
    torch.jit.save(model_trt, "model_jetson_optimized.pt")
    print(f"\n✅ Saved optimized model: model_jetson_optimized.pt")
```

### Quantization Strategy

| Precision | Size | Latency | Accuracy | Use Case |
|-----------|------|---------|----------|----------|
| **FP32** | 200 MB | 120 ms | 100% | Development/cloud |
| **FP16** | 100 MB | 40 ms | 99.5% | Jetson AGX (fast GPU) |
| **INT8** | 50 MB | 15 ms | 95% | Jetson Nano (limited compute) |
| **Binary** | 25 MB | 8 ms | 85% | Extreme edge (research) |

---

## Part 2: Real-Time Control Loop

### Latency Budget Allocation

```python
# File: realtime_control_loop.py
"""
Real-time control loop with latency budgeting.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

class RealtimeControlNode(Node):
    """Control loop running at fixed rate with latency monitoring."""

    def __init__(self):
        super().__init__('realtime_control')

        # Latency budget (ms)
        self.budget = {
            'perception': 20,      # Image processing + inference
            'control_logic': 5,    # Decision making
            'command_publish': 5,  # Send command to robot
            'buffer': 3,           # Safety margin
            'total': 33,           # 30 FPS = 33.3 ms per frame
        }

        self.latencies = {}

        # Subscribe/publish
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer at fixed rate (30 Hz)
        self.timer = self.create_timer(0.033, self.control_loop)  # 30 Hz
        self.loop_count = 0

    def image_callback(self, msg: Image):
        """Called when image arrives (async)."""
        self.latest_image = msg

    def control_loop(self):
        """Main control loop at 30 Hz."""
        self.loop_count += 1
        loop_start = time.perf_counter()

        # ==================== PERCEPTION ====================
        percept_start = time.perf_counter()

        # 1. Get image
        if not hasattr(self, 'latest_image'):
            return

        # 2. Run inference (TensorRT optimized)
        detections = self.run_inference(self.latest_image)

        percept_time = (time.perf_counter() - percept_start) * 1000
        self.latencies['perception'] = percept_time

        # Check budget
        if percept_time > self.budget['perception']:
            self.get_logger().warn(
                f"Perception slow: {percept_time:.1f}ms (budget: {self.budget['perception']}ms)")

        # ==================== CONTROL LOGIC ====================
        control_start = time.perf_counter()

        # 3. Make decision based on detections
        target_velocity = self.decide_action(detections)

        control_time = (time.perf_counter() - control_start) * 1000
        self.latencies['control'] = control_time

        # ==================== COMMAND PUBLISH ====================
        pub_start = time.perf_counter()

        # 4. Publish command
        cmd_msg = Twist()
        cmd_msg.linear.x = target_velocity[0]
        cmd_msg.angular.z = target_velocity[1]
        self.cmd_pub.publish(cmd_msg)

        pub_time = (time.perf_counter() - pub_start) * 1000
        self.latencies['publish'] = pub_time

        # ==================== MONITORING ====================
        total_time = (time.perf_counter() - loop_start) * 1000

        # Monitor latency
        if self.loop_count % 30 == 0:  # Every 1 second
            self.report_latency(total_time)

    def run_inference(self, image_msg):
        """Run TensorRT inference on image."""
        # Convert ROS message to tensor
        image_array = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            image_msg.height, image_msg.width, 3)

        # Run inference (very fast on Jetson with TensorRT)
        with torch.no_grad():
            output = self.model_trt(torch.tensor(image_array).cuda())

        return output

    def decide_action(self, detections):
        """Simple control logic: move toward detected object."""
        if len(detections) == 0:
            return [0.0, 0.0]  # Stop

        # Move toward center of detections
        center_x = np.mean([d[0] for d in detections])
        center_y = np.mean([d[1] for d in detections])

        # Simple proportional control
        v_linear = 0.5 if len(detections) > 0 else 0.0
        v_angular = (center_x - 320) / 320 * 0.5  # Turn toward object

        return [v_linear, v_angular]

    def report_latency(self, total_time):
        """Report latency metrics."""
        print(f"\n=== Latency Report (Loop {self.loop_count}) ===")
        for component, latency in self.latencies.items():
            budget = self.budget.get(component, 10)
            status = "✅" if latency < budget else "⚠️ "
            print(f"{status} {component:12s}: {latency:6.2f}ms (budget: {budget}ms)")

        print(f"{'✅' if total_time < self.budget['total'] else '⚠️ '} Total:        {total_time:6.2f}ms (budget: {self.budget['total']}ms)")

        # Alert if exceeding budget
        if total_time > self.budget['total']:
            self.get_logger().warn(f"LATENCY EXCEEDED: {total_time:.1f}ms > {self.budget['total']}ms")
            self.activate_fallback()

    def activate_fallback(self):
        """Fallback if latency budget exceeded."""
        self.get_logger().error("Activating fallback mode: slower control")
        # Reduce update rate, use cached detections, etc.
```

---

## Part 3: Safety Mechanisms

### Watchdog Timer

```python
# File: safety_monitor.py
"""
Watchdog and safety monitoring for autonomous robot.
"""

import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SafetyMonitor(Node):
    """Monitor robot health and implement failsafes."""

    def __init__(self):
        super().__init__('safety_monitor')

        self.watchdog_timeout = 0.5  # 500 ms heartbeat required
        self.last_heartbeat = time.time()
        self.is_healthy = True

        # Subscribe to heartbeat from control node
        self.heartbeat_sub = self.create_subscription(
            Bool, '/heartbeat', self.heartbeat_callback, 10)

        # Publish emergency stop status
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Watchdog timer (check health every 100 ms)
        self.timer = self.create_timer(0.1, self.check_health)

    def heartbeat_callback(self, msg: Bool):
        """Received heartbeat from control node."""
        self.last_heartbeat = time.time()
        self.is_healthy = True

    def check_health(self):
        """Check if control node is alive."""
        time_since_heartbeat = time.time() - self.last_heartbeat

        if time_since_heartbeat > self.watchdog_timeout:
            # Control node is frozen/dead
            self.is_healthy = False
            self.get_logger().error(
                f"WATCHDOG TIMEOUT: No heartbeat for {time_since_heartbeat:.2f}s")
            self.emergency_stop()
        else:
            self.is_healthy = True

    def emergency_stop(self):
        """Execute emergency stop."""
        self.get_logger().critical("EMERGENCY STOP ACTIVATED")

        # Publish stop command
        stop_msg = Bool()
        stop_msg.data = True  # True = emergency stop active
        self.estop_pub.publish(stop_msg)

        # In real system:
        # - Cut power to motors
        # - Engage brakes
        # - Alert operator
        # - Log failure

def main(args=None):
    rclpy.init(args=args)
    monitor = SafetyMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Emergency Stop Implementation

```python
# File: emergency_stop_node.py
"""
Hardware-level emergency stop handler.
"""

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EmergencyStopNode(Node):
    """Hardware emergency stop."""

    def __init__(self):
        super().__init__('emergency_stop_hardware')

        # GPIO pin for motor enable
        self.MOTOR_ENABLE_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR_ENABLE_PIN, GPIO.OUT)
        GPIO.output(self.MOTOR_ENABLE_PIN, GPIO.HIGH)  # Enabled by default

        # Subscribe to emergency stop topic
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)

        self.get_logger().info("Emergency stop handler ready")

    def estop_callback(self, msg: Bool):
        """Received emergency stop command."""
        if msg.data:  # True = activate stop
            self.get_logger().critical("🚨 EMERGENCY STOP 🚨")
            GPIO.output(self.MOTOR_ENABLE_PIN, GPIO.LOW)  # Disable motors
        else:  # False = resume
            self.get_logger().info("Emergency stop cleared - motors re-enabled")
            GPIO.output(self.MOTOR_ENABLE_PIN, GPIO.HIGH)  # Enable motors

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Part 4: Deployment on Physical Jetson

### Step 1: Prepare Jetson

```bash
# SSH into Jetson
ssh nvidia@jetson-orin-nano.local

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install ros-humble-ros-core -y

# Install Python dependencies
pip install torch tensorrt torch2trt numpy opencv-python

# Clone your robot package
git clone https://github.com/your-repo/humanoid-ai.git
cd humanoid-ai
```

### Step 2: Deploy Models

```bash
# Copy optimized TensorRT model
scp model_jetson_optimized.pt nvidia@jetson:/home/nvidia/humanoid-ai/models/

# Verify model loads
python3 << 'EOF'
import torch
model = torch.jit.load("models/model_jetson_optimized.pt")
print(f"✅ Model loaded: {model}")
EOF
```

### Step 3: Run on Real Hardware

```bash
# Start ROS 2
source /opt/ros/humble/setup.bash

# Launch robot system
ros2 launch humanoid_ai sim_to_real.launch.py

# In another terminal, verify topics
ros2 topic list
# Should show: /camera/image_raw, /detections, /cmd_vel, /emergency_stop, etc.

# Monitor latency
ros2 run humanoid_ai latency_monitor.py
```

---

## Part 5: Hands-On Labs

### Lab 1: Model Optimization (1 hour)

Convert Chapter 4 trained model to TensorRT. Benchmark on GPU.

**Acceptance**: FP16 model shows 3-5x latency reduction

### Lab 2: Real-Time Control Loop (1 hour)

Implement latency budgeting control loop. Verify all components within budget.

**Acceptance**: All loop components `<33ms` total (30 FPS target)

### Lab 3: Safety Mechanisms (1 hour)

Implement watchdog + emergency stop. Test by killing control node.

**Acceptance**: Watchdog detects failure, emergency stop activates within 100ms

### Lab 4: Jetson Deployment (1 hour)

Deploy to physical Jetson Orin Nano. Run end-to-end pipeline.

**Acceptance**: Model runs at >10 FPS with safety monitoring

---

## Part 6: Troubleshooting Guide

### Issue: Model runs slow on Jetson

**Solutions**:
1. Verify TensorRT optimization: `model_jetson_optimized.pt` size ~50 MB?
2. Check input shape: Is inference expecting (1,3,640,480)?
3. Profile with `latency_profiler.py` - which component is slow?
4. Consider INT8 quantization for 8x speedup

### Issue: Latency spikes every few seconds

**Solutions**:
1. Check garbage collection: `gc.disable()` in hot loop
2. Verify CPU is not thermal-throttling: Monitor `/sys/class/thermal/`
3. Look for ROS 2 buffer copies: Use zero-copy transports
4. Reduce image resolution: 640×480 → 320×240

### Issue: Control loop misses deadline occasionally

**Solutions**:
1. Increase thread priority: `taskset -c 1-4 ros2 run ...` (pin to CPU cores)
2. Reduce perception quality: Lower inference confidence threshold
3. Implement fallback: Use cached predictions if deadline missed

---

## Part 7: End-of-Chapter Exercises

### Exercise 1: Optimize Your Model (1 hour)

Convert your Chapter 4 model to TensorRT INT8. Measure speedup and accuracy loss.

**Acceptance**: 5-10x speedup, `<5%` accuracy loss

### Exercise 2: Deploy End-to-End (1.5 hours)

Run complete pipeline on Jetson: perception → control → motor commands.

**Acceptance**: Robot responds to detected objects in real time (>10 FPS)

### Exercise 3: Safety Testing (1 hour)

Test watchdog by killing control node. Verify emergency stop triggers.

**Acceptance**: System reaches emergency stop state within 100ms

### Exercise 4: Performance Documentation (1 hour)

Write deployment guide: model optimization → Jetson setup → safety procedures.

**Acceptance**: 2-3 page document with latency metrics, procedures, troubleshooting

---

## Part 8: Capstone Integration

Your Humanoid AI Assistant capstone is complete:

✅ **Chapter 1**: Isaac ecosystem foundation
✅ **Chapter 2**: Digital twin with sensors
✅ **Chapter 3**: Real-time perception (object detection, SLAM, fusion)
✅ **Chapter 4**: Synthetic data generation for training
✅ **Chapter 5**: Production deployment with safety ← FINAL CHAPTER

**Capstone Project Completion Checklist**:
- [ ] Humanoid digital twin running in Isaac Sim
- [ ] Object detection model trained on 10K+ synthetic images
- [ ] Real-time perception pipeline (>10 FPS)
- [ ] Safety monitoring (watchdog, emergency stop)
- [ ] Deployed to Jetson or cloud GPU
- [ ] Demo video (4-5 min) showing perception + control
- [ ] Documentation: architecture, deployment, latency metrics

---

## Next Steps

**Congratulations!** You've completed Module 3.

### Optional: Chapter 6 (Advanced Topics)

If interested in deeper dives:
- **Reinforcement Learning**: Train policies in Isaac Lab
- **Multi-Robot Coordination**: Swarm robotics
- **Physics-Informed Networks**: Hybrid learning
- **Digital Twin Maintenance**: Calibration, update

### Continue to Module 4

Next module covers **Vision-Language-Action Robotics**:
- Large language models (GPT) for task planning
- Whisper for speech understanding
- End-to-end learned policies

---

**Chapter Summary**: 4-5 hours | Difficulty: Intermediate | Week 10

---

## Additional Resources

- [Jetson Developer Guide](https://docs.nvidia.com/jetson/jetson-agx-orin-developer-kit/index.html)
- [TensorRT Best Practices](https://docs.nvidia.com/deeplearning/tensorrt/best-practices/)
- [ROS 2 Real-Time](https://docs.ros.org/en/foxy/Concepts/About-Real-Time.html)
- [Safety in Robotics](https://www.iso.org/standard/74272.html)

---

**End of Module 3**. Great work! 🤖
