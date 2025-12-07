---
sidebar_position: 7
sidebar_label: "Week 9-10: AI & Capstone"
title: "Week 9-10 Practice Guide: AI Perception, Data & Deployment (Chapters 3-5 + Capstone)"
tags: ["practice-guide", "week-9", "week-10", "capstone", "deployment", "perception"]
difficulty: "Intermediate"
module: 3
week: "9-10"
prerequisites: ["Week 8 complete", "Chapter 1-2 mastered"]
estimated_time: "16-20 hours"
topics: ["Isaac ROS", "perception", "SLAM", "synthetic data", "deployment", "capstone project"]
---

# Week 9-10 Practice Guide: AI Perception, Data & Deployment

**Weeks**: 9-10 | **Duration**: 16-20 hours | **Chapters**: 3-5 + Capstone Sprint

Integrate perception AI, generate training datasets, and deploy to production.

---

## Daily Breakdown (Week 9)

### Monday: Isaac ROS Setup & Docker Environment (1 hour)

**Morning Tasks**:
1. Understand Isaac ROS (15 min)
   - [ ] Read Chapter 3 "Isaac ROS Architecture"
   - [ ] Understand: CUDA acceleration, perception modules, Jetson deployment

2. Install Isaac ROS Docker (30 min)
   - [ ] Docker installed? `docker --version`
   - [ ] Pull Isaac ROS image: `docker pull nvcr.io/nvidia/isaac/ros:humble`
   - [ ] Verify NVIDIA Docker runtime: `docker run --rm --runtime nvidia nvidia/cuda:12.0-runtime nvidia-smi`

3. Test perception modules (15 min)
   - [ ] Launch Isaac ROS container
   - [ ] Verify CUDA available inside: `nvcc --version`
   - [ ] List perception modules: `ros2 pkg list | grep isaac`

**Success Criteria**:
- [ ] Docker running with NVIDIA GPU support
- [ ] Isaac ROS packages installed
- [ ] Can access CUDA from within container

---

### Tuesday: Object Detection Deployment (1.5 hours)

**Morning Tasks**:
1. Load pre-trained detection model (20 min)
   - [ ] Use Chapter 3 code: `object_detection_node.py`
   - [ ] Load YOLOv5 small model
   - [ ] Verify model loads: ~80MB FP32

2. Connect to humanoid camera (30 min)
   - [ ] From Week 8, use Isaac Sim with humanoid + RGB-D camera
   - [ ] Publish `/camera/image_raw` from Isaac Sim
   - [ ] In separate terminal, run detection node
   - [ ] Verify detections published to `/detections`

3. Test real-time performance (20 min)
   - [ ] Run `ros2 topic hz /detections`
   - [ ] Verify >10 FPS (100ms per frame)
   - [ ] Check CPU/GPU load: `nvidia-smi`
   - [ ] Identify bottlenecks if slow

4. Visualize detections (20 min)
   - [ ] Use RViz: `ros2 run rviz2 rviz2`
   - [ ] Add `/camera/image_raw` display
   - [ ] Add `/detections` visualization (bounding boxes)
   - [ ] Observe: Are detections correct?

**Success Criteria**:
- [ ] Detection model loaded
- [ ] Real-time inference >10 FPS
- [ ] Detections visualized in RViz
- [ ] Model correctly identifies objects

**Challenge**:
Deploy multiple detection models (YOLO, ResNet). Compare accuracy vs. speed.

---

### Wednesday: Visual SLAM & Sensor Fusion (1.5 hours)

**Morning Tasks**:
1. Visual SLAM setup (20 min)
   - [ ] Review Chapter 3 SLAM explanation
   - [ ] Use `visual_slam_node.py` code
   - [ ] Understand: feature matching, pose estimation, mapping

2. Run SLAM with moving humanoid (30 min)
   - [ ] In Isaac Sim, animate humanoid walking
   - [ ] Launch SLAM node: `ros2 run ... visual_slam_node.py`
   - [ ] Monitor: `/map` and `/pose` topics
   - [ ] In RViz, add Map and Pose visualizations
   - [ ] Observe: Map building in real-time

3. Test sensor fusion (30 min)
   - [ ] Launch fusion node with camera + LiDAR + IMU
   - [ ] Subscribe to all three sensor streams
   - [ ] Fuse with Kalman filter (Chapter 3 code)
   - [ ] Visualize fused estimate in RViz
   - [ ] Block camera → verify LiDAR compensates

4. Measure accuracy (10 min)
   - [ ] Compare fused vs. camera-only pose
   - [ ] Fused should be smoother (less jittery)
   - [ ] Fused should be robust to sensor failures

**Success Criteria**:
- [ ] SLAM node running, building map
- [ ] Sensor fusion working (3+ sensors)
- [ ] Fused estimate smoother than individual sensors
- [ ] Robust to sensor dropouts

**Measurement**:
Document: Is sensor fusion noticeably better than single sensor? By how much?

---

### Thursday: Isaac Lab & Domain Randomization (1.5 hours)

**Morning Tasks**:
1. Understand Isaac Lab (15 min)
   - [ ] Read Chapter 4 "Isaac Lab Framework"
   - [ ] Understand: gymnasium environments, domain randomization

2. Set up Isaac Lab environment (20 min)
   - [ ] Install Isaac Lab: `pip install omni-isaac-lab`
   - [ ] Verify installation: `python3 -c "from omni.isaac.lab.envs import ManagerBasedEnv"`
   - [ ] Review `humanoid_task.py` code

3. Create simple randomized task (30 min)
   - [ ] Create 2-3 versions of humanoid scene
   - [ ] Version 1: No randomization (baseline)
   - [ ] Version 2: Light randomization (colors, materials)
   - [ ] Version 3: Full randomization (physics, scene layout)

4. Generate synthetic dataset (20 min)
   - [ ] Use `generate_dataset.py` from Chapter 4
   - [ ] Generate 5,000 frames (should take ~5 min)
   - [ ] Verify RGB + depth + segmentation exported
   - [ ] Check file sizes reasonable (~5-10 MB for 5K frames)

**Success Criteria**:
- [ ] Isaac Lab environments created
- [ ] Domain randomization working (observe differences between versions)
- [ ] 5,000+ frames exported
- [ ] Dataset ready for ML training

---

### Friday: Data Export & Training Integration (1.5 hours)

**Morning Tasks**:
1. Prepare full dataset (20 min)
   - [ ] Generate larger dataset: 10,000 frames
   - [ ] Export RGB, depth, segmentation
   - [ ] Create COCO format annotations (use Chapter 4 code)
   - [ ] Verify annotations valid JSON

2. Create PyTorch dataset loader (20 min)
   - [ ] Use Chapter 4 code: `SyntheticDataset` class
   - [ ] Load 10,000 frames with COCO labels
   - [ ] Test DataLoader: `batch = next(iter(loader))`
   - [ ] Verify batch shapes correct

3. Quick training test (20 min)
   - [ ] Load pre-trained detection model
   - [ ] Train for 1 epoch on synthetic data
   - [ ] Monitor loss: Should decrease over batch
   - [ ] Save checkpoint

4. Prepare for transfer learning (15 min)
   - [ ] Save synthetic-trained model checkpoint
   - [ ] Document: Accuracy on synthetic test set
   - [ ] Ready for fine-tuning on real data (Chapter 4)

**Success Criteria**:
- [ ] 10K+ frame dataset generated
- [ ] COCO annotations valid
- [ ] PyTorch DataLoader working
- [ ] Training runs without errors
- [ ] Model checkpoint saved

---

## Daily Breakdown (Week 10)

### Monday-Tuesday: Model Optimization for Jetson (2 hours)

**Morning Tasks**:
1. Review optimization techniques (15 min)
   - [ ] Understand FP32 vs. FP16 vs. INT8 tradeoffs (Chapter 5)
   - [ ] Know speedup targets: 3-5x FP16, 8x INT8

2. Convert to TensorRT FP16 (30 min)
   - [ ] Use `optimize_model.py` from Chapter 5
   - [ ] Load synthetic-trained model
   - [ ] Convert to TensorRT FP16
   - [ ] Verify model size: ~50% reduction

3. Benchmark on GPU (20 min)
   - [ ] Run `benchmark_model()` function
   - [ ] Measure latency: FP32 vs. FP16
   - [ ] Document speedup: Should be 2-3x
   - [ ] Verify accuracy unchanged (`<1%` loss)

4. Optional: INT8 Quantization (20 min)
   - [ ] If interested, convert to INT8
   - [ ] Benchmark again: Should be 8x faster
   - [ ] Measure accuracy loss: `<5%` acceptable?
   - [ ] For deployment, choose FP16 or INT8

5. Test on Jetson (if available) (35 min)
   - [ ] Copy model to physical Jetson Orin Nano
   - [ ] Run inference: Should be >15 FPS
   - [ ] Profile latency (use Chapter 5 profiler)
   - [ ] Verify model runs in real-time

**Success Criteria**:
- [ ] Model converted to TensorRT
- [ ] Latency reduced 3-5x vs. FP32
- [ ] Accuracy loss `<5%`
- [ ] Ready for deployment

---

### Wednesday: Jetson Simulator Deployment (2 hours)

**Morning Tasks**:
1. Set up Jetson simulator environment (30 min)
   - [ ] Review Chapter 5 deployment architecture
   - [ ] Use Docker container simulating Jetson Orin Nano
   - [ ] Copy optimized TensorRT model into container

2. Deploy Isaac ROS perception (20 min)
   - [ ] Launch Isaac ROS perception nodes in Docker
   - [ ] Start humanoid simulation (Isaac Sim)
   - [ ] Bridge sim camera to Docker container
   - [ ] Run inference inside Docker

3. Implement real-time control loop (40 min)
   - [ ] Use Chapter 5 code: `realtime_control_loop.py`
   - [ ] Implement latency budgeting
   - [ ] Monitor all components `<33ms` total
   - [ ] Detect and alert if budget exceeded

4. Test safety mechanisms (30 min)
   - [ ] Implement watchdog (Chapter 5)
   - [ ] Test: Kill control process → watchdog triggers
   - [ ] Implement emergency stop
   - [ ] Verify robot halts immediately

**Success Criteria**:
- [ ] Optimized model running in Docker
- [ ] Real-time control loop `<33ms` per frame
- [ ] Watchdog working
- [ ] Emergency stop functional

---

### Thursday-Friday: Capstone Sprint (4 hours)

**Capstone Project: "Humanoid AI Assistant"**

Integrate everything into one complete system.

#### Part 1: System Architecture Setup (1 hour)

**Tasks**:
1. Define system architecture (20 min)
   - [ ] Create diagram: Isaac Sim → ROS 2 → Perception → Control → Humanoid
   - [ ] Document data flow (camera → detection → decision → motion)
   - [ ] Identify latency bottlenecks

2. Implement HAL (Hardware Abstraction Layer) (20 min)
   - [ ] Write interface: `class RobotController`
   - [ ] Methods: `move_to(target)`, `stop()`, `check_safety()`
   - [ ] Works identically in sim and on real Jetson

3. Set up ROS 2 launch file (20 min)
   - [ ] Create `humanoid_ai.launch.py`
   - [ ] Launch all nodes in correct order:
     - Isaac Sim bridge
     - Perception node
     - Control node
     - Safety monitor
     - (optional) Visualization (RViz)

**Success Criteria**:
- [ ] Architecture documented
- [ ] HAL implemented
- [ ] Launch file created and tested

---

#### Part 2: AI Perception System (1 hour)

**Tasks**:
1. Object detection (20 min)
   - [ ] Deploy TensorRT-optimized detector
   - [ ] Detect humanoids, obstacles, target objects
   - [ ] Confidence threshold tuning
   - [ ] Real-time performance >10 FPS

2. Visual SLAM (20 min)
   - [ ] SLAM node mapping environment
   - [ ] Localization: Know robot's position in map
   - [ ] Combine with object detection: "Object at position [x,y,z]"

3. Sensor fusion (20 min)
   - [ ] Fuse camera + LiDAR + IMU
   - [ ] Robust to sensor failures
   - [ ] Clean, smooth perception output

**Success Criteria**:
- [ ] Object detection >80% accuracy
- [ ] SLAM mapping real-time
- [ ] Sensor fusion robust

---

#### Part 3: Decision & Control (1 hour)

**Tasks**:
1. Decision logic (20 min)
   - [ ] State machine or behavior tree
   - [ ] States: Idle, Searching, Approaching, Grasping
   - [ ] Transitions based on perceptions

2. Motion control (20 min)
   - [ ] Implement reach, grasp, lift behaviors
   - [ ] Real-time control loop (Chapter 5)
   - [ ] Safety constraints: Joint limits, collision avoidance

3. Integration test (20 min)
   - [ ] Launch full system
   - [ ] Place target object
   - [ ] Humanoid should detect → approach → grasp
   - [ ] Measure success rate over 10 trials

**Success Criteria**:
- [ ] Decision logic working
- [ ] Motion commands correct
- [ ] Full system integrated and tested

---

#### Part 4: Safety & Production Readiness (0.5 hour)

**Tasks**:
1. Safety mechanisms (15 min)
   - [ ] Watchdog: Detects frozen nodes
   - [ ] Emergency stop: Hardware cutoff
   - [ ] Rate limiting: Prevent command spam
   - [ ] Graceful degradation: Use fallback if sensor fails

2. Logging & monitoring (15 min)
   - [ ] Log all decisions + outcomes
   - [ ] Monitor performance metrics (FPS, latency, accuracy)
   - [ ] Alert on anomalies

**Success Criteria**:
- [ ] All safety mechanisms tested
- [ ] System handles failures gracefully
- [ ] Monitoring in place

---

#### Part 5: Documentation & Demo (0.5 hour)

**Tasks**:
1. Write documentation (20 min)
   - [ ] Architecture overview (1 page)
   - [ ] Deployment guide (1 page)
   - [ ] Troubleshooting guide (1 page)

2. Record demo video (10 min)
   - [ ] 4-5 minute video showing:
     - Humanoid in scene with target object
     - Object detection in action
     - SLAM mapping
     - Humanoid approaching/grasping target
     - Real-time performance metrics overlay
   - [ ] Save as: `capstone_demo.mp4`

**Success Criteria**:
- [ ] 3-page documentation
- [ ] 4-5 min demo video
- [ ] Ready for capstone submission

---

## Capstone Project Acceptance Criteria

### Perception

- [ ] Object detection accuracy >80% on test set
- [ ] Detection runs real-time (>10 FPS on Jetson)
- [ ] Robust to lighting changes (domain randomization trained)
- [ ] Confident detections (>0.5 confidence threshold)

### Control & Safety

- [ ] Real-time control loop `<33ms`
- [ ] Watchdog triggers within 100ms of failure
- [ ] Emergency stop activates within 50ms
- [ ] Graceful degradation if sensor fails

### Integration

- [ ] All modules integrated (perception → decision → control)
- [ ] Hardware abstraction layer working (sim and real)
- [ ] ROS 2 communication stable
- [ ] No crashes or deadlocks over 5-minute run

### Documentation

- [ ] Architecture diagram
- [ ] Deployment procedures clear
- [ ] Latency metrics documented
- [ ] Demo video shows system in action

---

## Independent Exercises

### Exercise 1: Multi-Model Ensemble (1 hour)

Run 2-3 detection models in parallel. Compare accuracy vs. speed. Which is better?

### Exercise 2: Adaptive Control (1 hour)

Implement control that adapts based on perception confidence. If confident, move fast. If uncertain, move cautiously.

### Exercise 3: Hardware Stress Test (1 hour)

Run system for 30 minutes. Monitor latency, memory, thermal. Any degradation?

---

## Challenge Project: Real-World Transfer (Bonus, 2-3 hours)

If you have real-world images of objects:

1. Fine-tune synthetic-trained model on real images (Chapter 4)
2. Measure improvement: How much better on real data?
3. Deploy fine-tuned model
4. Evaluate capstone metrics on real humanoid (if available)

---

## Week 9-10 Completion Checklist

- [ ] Isaac ROS Docker environment working
- [ ] Object detection deployed and real-time
- [ ] Visual SLAM mapping in real-time
- [ ] Sensor fusion implemented and tested
- [ ] 10,000+ synthetic images generated
- [ ] Model optimized (TensorRT FP16 or INT8)
- [ ] Deployed to Jetson simulator
- [ ] Real-time control loop implemented
- [ ] Safety mechanisms (watchdog, e-stop) working
- [ ] Capstone project complete
- [ ] Architecture documentation written
- [ ] Demo video recorded
- [ ] All exercises completed

**By end of Week 10**: ✅ Module 3 Complete. Ready to deploy or continue to Module 4.

---

## Troubleshooting

### Latency Exceeds Budget
- Check: Is perception the bottleneck? (run profiler)
- Solution: Reduce image resolution, lower detection confidence threshold, use INT8 quantization

### SLAM Drifts / Map Incorrect
- Check: Are camera intrinsics correct?
- Solution: Lower feature matching threshold, increase map update frequency

### Sensor Fusion Unstable
- Check: Are sensor data rates synchronized?
- Solution: Implement message buffer (e.g., ApproximateTimeSynchronizer from ROS 2)

### Capstone Safety Fails
- Check: Is watchdog timeout too short?
- Solution: Increase timeout to 1 second, verify control node publishing heartbeat

---

## Recommended Pace

- **Week 9 Mon-Fri**: 6-8 hours (perception + data generation)
- **Week 10 Mon-Tue**: 4 hours (model optimization)
- **Week 10 Wed**: 2 hours (deployment)
- **Week 10 Thu-Fri**: 4 hours (capstone sprint)
- **Total**: 16-20 hours

Aim to complete capstone by Friday. Use weekend to refine if needed.

---

## Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Jetson Deployment Guide](https://docs.nvidia.com/jetson/)
- [TensorRT Inference](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/)
- [ROS 2 Real-Time](https://docs.ros.org/en/humble/Concepts/About-Real-Time.html)

---

**Congratulations on completing Module 3!** 🤖🚀

Next: Module 4 (Vision-Language-Action Robotics) or capstone refinement.
