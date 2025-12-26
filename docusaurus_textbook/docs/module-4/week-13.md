# Week 13: VLA Capstone Sprint & Integration

**Module**: 4 | **Week**: 13 (Part 2) | **Difficulty**: Advanced | **Estimated Time**: 40–50 hours (2-week sprint)

---

## Week 13 Overview

Week 13 is the capstone sprint where you integrate Chapters 1-4 into a **complete end-to-end VLA system**. Over two weeks, you'll build a humanoid robot that understands voice commands and executes multi-step household tasks.

### Learning Outcomes

By completing Week 13, you will:

1. **Deploy a complete VLA pipeline** in simulation (Isaac Lab)
2. **Respond to natural language** voice commands from users
3. **Perceive and understand** complex household environments
4. **Decompose tasks** into executable robot skills using LLMs
5. **Execute multi-step behaviors** with real-time feedback and replanning
6. **Evaluate your system** against a technical rubric covering perception, planning, control, and safety
7. **(Bonus)** Deploy to real Jetson hardware with performance tuning

### Capstone Project Overview

**Title**: "Embodied AI Household Assistant: Voice-Controlled Humanoid Robot"

**Goal**: Build a humanoid robot that responds to voice commands like:
- "Pick up the red cup from the table and place it on the shelf"
- "Fetch the medicine from the bathroom and bring it to the living room"
- "Clean the table and organize the toys into the bin"

**Success Criteria**: Your robot successfully:
1. **Listens** to voice input (Whisper)
2. **Understands** the scene (perception pipeline)
3. **Plans** a task sequence (LLM decomposition)
4. **Executes** the plan (ROS 2 + MoveIt + Nav2)
5. **Adapts** to failures (replanning)
6. **Completes** the task safely

---

## Daily Capstone Schedule (Week 13)

### **Monday: Perception Sprint**

**Goal**: Implement and test real-time scene perception.

**Morning (2 hours)**:
- Review Chapter 2 (Vision for VLA)
- Load perception models (YOLO, Mask R-CNN, SAM)
- Test on sample scenes

**Afternoon (2 hours)**:
- Integrate with ROS 2 (create perception node)
- Subscribe to simulated camera topics
- Publish scene graph

**Evening (1 hour)**:
- Debug and optimize perception latency
- Target: `<100ms` per frame

**Deliverable**: Working perception node publishing accurate scene graphs.

---

### **Tuesday: Language Planning Sprint**

**Goal**: Implement LLM-driven task decomposition.

**Morning (2 hours)**:
- Review Chapter 3 (Language & LLM)
- Set up Whisper API (or mock)
- Test speech-to-text on sample commands

**Afternoon (2 hours)**:
- Implement LLM prompting (task decomposition)
- Test with 10+ household tasks
- Verify skill parsing from LLM outputs

**Evening (1 hour)**:
- Build fallback mechanisms (error handling, ambiguity resolution)
- Target: `<1` second for task decomposition

**Deliverable**: LLM-based task decomposer accepting voice commands and outputting skill sequences.

---

### **Wednesday: Control Integration Sprint**

**Goal**: Wire up perception → planning → control.

**Morning (2 hours)**:
- Review Chapter 4 (Control Architecture)
- Implement VLA orchestrator node
- Connect sub-components (perception, LLM, control)

**Afternoon (2 hours)**:
- Test orchestrator with simple tasks (single skill)
- Debug message passing and synchronization
- Add logging for visibility

**Evening (1 hour)**:
- Implement safety watchdog
- Test emergency stop functionality

**Deliverable**: Working VLA orchestrator executing simple skills.

---

### **Thursday: Full Pipeline Testing**

**Goal**: Execute multi-step household tasks.

**Morning (2 hours)**:
- Run end-to-end system with sample tasks
- Execute task: "Pick red cup, place on shelf"
- Evaluate success metrics

**Afternoon (2 hours)**:
- Test 5+ different household tasks
- Identify and fix failures
- Optimize control loop latency

**Evening (1 hour)**:
- Stress test (rapid command sequencing)
- Document any observed limitations

**Deliverable**: Stable end-to-end VLA system handling diverse household tasks.

---

### **Friday: Evaluation & Documentation**

**Goal**: Evaluate system against rubric, prepare presentation.

**Morning (2 hours)**:
- Run technical evaluation on all rubric criteria
- Measure: perception accuracy, LLM quality, control robustness, safety
- Document metrics

**Afternoon (2 hours)**:
- Prepare capstone presentation/report
- Create system architecture diagram
- Prepare demo video

**Evening (1 hour)**:
- Finalize documentation and submission

**Deliverable**: Complete capstone evaluation, presentation, and system report.

---

### **Week 2 (Days 6-10): Refinement & Bonus Features**

**Day 6**: Perception Optimization
- Improve accuracy with Mask R-CNN or SAM
- Test on diverse scenes
- Optimize latency

**Day 7**: LLM Prompt Engineering
- Refine prompts for better task decomposition
- Test edge cases (ambiguous commands, novel objects)
- Implement confidence-based fallbacks

**Day 8**: Safety & Robustness
- Implement advanced failure recovery
- Test collision avoidance
- Stress-test with adversarial scenarios

**Day 9**: Hardware Optimization (Bonus)
- Quantize models (FP32 → INT8)
- Deploy to Jetson platform
- Benchmark real-time performance

**Day 10**: Final Polish & Submission
- Finalize documentation
- Record demo video
- Submit for evaluation

---

## Capstone Project Specification

### Project Requirements

**Core Requirements** (All students):

1. **Perception Pipeline** (Chapter 2):
   - Detect objects with >90% accuracy (on test dataset)
   - Extract affordances (graspable, movable, etc.)
   - Build scene graph in JSON format
   - ROS 2 integration with latency `<100ms`

2. **Language Planning** (Chapter 3):
   - Accept voice commands via Whisper
   - Decompose into skill sequences using LLM
   - Parse LLM outputs into JSON action plans
   - Handle ambiguity with user fallback

3. **ROS 2 Control** (Chapter 4):
   - Navigate using Nav2 (avoid obstacles)
   - Manipulate using MoveIt (pick & place)
   - Execute skills with proper error handling
   - Provide feedback (joint states, gripper force)

4. **System Integration**:
   - VLA orchestrator coordinates all components
   - End-to-end execution of household tasks
   - Real-time performance (control loop `<33ms`)
   - Safety mechanisms (watchdogs, emergency stop)

**Bonus Requirements** (For higher grades):

1. **Advanced Perception**:
   - Implement Mask R-CNN for precise masks
   - Use SAM for zero-shot segmentation
   - Multi-view fusion for better accuracy

2. **Robust Planning**:
   - Implement behavior trees (hierarchical control)
   - Advanced replanning on skill failure
   - Confidence-based decision making

3. **Hardware Deployment**:
   - Quantize models for Jetson
   - Deploy to real Jetson platform
   - Demonstrate real-time performance
   - Latency `<50ms` for control loop

---

## Capstone Evaluation Rubric

Your capstone project will be evaluated on four dimensions:

| Category | Weight | Criteria | Excellent | Good | Fair | Poor |
|----------|--------|----------|-----------|------|------|------|
| **Perception** (25%) | 25% | Object detection accuracy, affordance extraction, ROS 2 integration | >95% accuracy, SAM integration, `<50ms` latency | >90% accuracy, Mask R-CNN, `<100ms` latency | >85% accuracy, basic YOLO, `<100ms` latency | `<85%` accuracy or non-functional |
| **LLM Planning** (25%) | 25% | Task decomposition, prompt engineering, error handling | Complex multi-step tasks, robust fallbacks, semantic understanding | Standard household tasks, basic fallbacks | Simple tasks only, limited error handling | Non-functional or unable to parse output |
| **Control & Integration** (35%) | 35% | Nav2/MoveIt execution, orchestration, real-time performance | Full multi-step execution, `<33ms` control loop, advanced recovery | Successful execution of 3+ tasks, `<50ms` loop, basic recovery | 1-2 tasks work, >100ms loop, minimal recovery | Failed execution or non-functional components |
| **Safety Mechanisms** (15%) | 15% | Watchdogs, emergency stops, collision avoidance, graceful degradation | Advanced watchdogs, full safety suite, proven stress tests | Watchdog + emergency stop, basic collision checks | Minimal safety mechanisms | No safety features |

**Grading Scale**:
- **Excellent** (>90% on all dimensions): A (95-100%)
- **Good** (>80% on all dimensions): B (85-94%)
- **Fair** (>70% on all dimensions): C (75-84%)
- **Below Fair** (`<70%`): D/F (`<75%`)

---

## Technical Challenges & Solutions

### Challenge 1: Latency Management

**Problem**: End-to-end latency (perception + planning + control) exceeds real-time budgets.

**Solutions**:
- Run perception in parallel (not serialized)
- Cache LLM outputs for repeated commands
- Quantize heavy models (YOLO → INT8)
- Use edge inference (Jetson + TensorRT)

### Challenge 2: LLM Hallucination

**Problem**: LLM proposes infeasible actions (e.g., "grasp object through wall").

**Solutions**:
- Validate action feasibility before execution
- Provide detailed system prompt with constraints
- Implement confidence thresholds
- Add human-in-the-loop verification for low-confidence plans

### Challenge 3: Sim-to-Real Gap

**Problem**: Simulation physics doesn't match real robots.

**Solutions**:
- Use high-fidelity simulation (Isaac Lab)
- Randomize simulation parameters (domain randomization)
- Calibrate real robot controllers separately
- Implement graceful degradation (fallback to safer parameters)

---

## Capstone Bonus: Real Hardware Deployment

For students with access to physical robots (Jetson + real humanoid):

### Hardware Setup

1. **Jetson Platform** (e.g., Orin Nano):
   - Install ROS 2 Humble
   - Install robotics libraries (MoveIt, Nav2)
   - Set up perception models (TensorRT-optimized)

2. **Sensors**:
   - RGB camera + depth sensor (Intel RealSense D435)
   - Microphone (USB or built-in)
   - IMUs, joint encoders for feedback

3. **Actuators**:
   - Mobile base (wheels or legged locomotion)
   - Robot arm (7-DOF or less)
   - Gripper (parallel jaw or multi-finger)

### Deployment Steps

1. **Quantize Models**:
   ```bash
   # Convert YOLO to TensorRT
   python -c "from ultralytics import YOLO; model = YOLO('yolov8m.pt'); model.export(format='engine')"
   ```

2. **Deploy to Jetson**:
   ```bash
   # Copy ROS 2 packages to Jetson
   scp -r ros_workspace jetson@robot:/home/jetson/

   # SSH into Jetson and launch
   ssh jetson@robot
   cd /home/jetson/ros_workspace
   ros2 launch vla_system full_pipeline.launch.py
   ```

3. **Calibrate**:
   - Calibrate camera intrinsics
   - Calibrate arm kinematics
   - Calibrate gripper force feedback

4. **Test**:
   - Run simple skills (MoveTo, Pick, Place)
   - Test full pipelines
   - Measure real-world latency

### Real Hardware Evaluation

**Latency Benchmarks**:
- Perception (Jetson GPU): 80-120ms
- LLM planning (cloud API): 1-3s
- Control execution: 20-50ms per step
- **Total cycle time**: ~3-4 seconds per high-level command (acceptable)

**Success Metrics**:
- Pick success rate: >80% (grasping without dropping)
- Navigation accuracy: within 5cm of goal
- Task completion rate: >70% for household tasks
- Safety incidents: 0 (no collisions, emergency stops work)

---

## Capstone Submission

### Required Deliverables

1. **Code**:
   - All ROS 2 nodes (perception, planning, control)
   - Configuration files and launch scripts
   - Dependency list (requirements.txt, package.xml)

2. **Documentation**:
   - System architecture diagram
   - Setup and deployment instructions
   - Evaluation results with metrics
   - Known limitations and future work

3. **Presentation**:
   - 5-minute demo video showing full system
   - Slides explaining architecture and results
   - Q&A session discussing challenges faced

4. **Evaluation Report**:
   - Detailed rubric scoring
   - Metric evidence (perception accuracy, LLM quality, control performance)
   - Comparison to baselines or prior work

### Submission Format

- GitHub repository with all code
- README.md with quick-start instructions
- docs/ folder with architecture, deployment guide, results

---

## Summary

The capstone integrates everything you've learned:

- **Chapter 1**: Conceptual framework for VLA systems
- **Chapter 2**: Real-time perception for scene understanding
- **Chapter 3**: Language processing and task reasoning
- **Chapter 4**: Robust control and safe deployment

Your capstone project demonstrates **end-to-end autonomous behavior** in response to natural language, showcasing the power of integrated Vision-Language-Action systems.

---

## References

1. Brohan, A., et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." arXiv:2307.15818
2. Padalkar, A., et al. (2024). "RT-X: Robotics Transformer for Diverse Tasks." DeepMind
3. NVIDIA Isaac Lab Documentation: https://isaac-sim.github.io/
4. ROS 2 Documentation: https://docs.ros.org/en/humble/

---

**Congratulations on reaching the capstone! You're now ready to build state-of-the-art autonomous robotic systems.**
