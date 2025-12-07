# Module 4 Capstone Project Starter Template

Use this template to structure your capstone project. Replace `[YOUR CONTENT]` with your implementation.

---

## Project Title

**[Your Robot Name/Project Title]**

**Team Members**: [List all team members]

**Date Started**: [Date]

**Target Completion**: [Date]

---

## 1. Project Overview

### Problem Statement

[Describe the real-world problem your VLA system solves. Example: "Enable household robots to understand natural language commands and perform multi-step household tasks without pre-programming specific task variants."]

### Solution Approach

[Describe your VLA architecture at high level. Include: vision module, language module, action module. How do they integrate?]

### Success Criteria

- [ ] Perception accuracy: `>90%` object detection on test dataset
- [ ] Speech recognition: `>95%` accuracy with Whisper
- [ ] Task completion: Successfully execute `5+` different household tasks from voice
- [ ] Safety: Zero collisions, working emergency stops, ``<2`s` recovery from failures
- [ ] Real-time performance: ``<3`3ms` control loop (30 FPS), ``<1`00ms` perception latency

---

## 2. System Architecture

### High-Level Block Diagram

```
[Draw your architecture here using Mermaid or ASCII art]

Example:
┌─────────────────────────────────────────────┐
│  User Voice Input                           │
│  "Pick up the red cup"                      │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│  [Whisper Speech-to-Text]                   │
│  "pick up the red cup"                      │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│  [Scene Perception]                         │
│  {objects: [red_cup@(0.5, 0.3)], ...}       │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│  [LLM Task Decomposition]                   │
│  [MoveTo(0.5, 0.3), Pick(), MoveTo(0.8, ...│
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│  [ROS 2 Orchestrator]                       │
│  Execute skills with feedback & recovery    │
└──────────────┬──────────────────────────────┘
               ↓
┌─────────────────────────────────────────────┐
│  Robot Action                               │
│  Navigate → Pick → Place → Verify           │
└─────────────────────────────────────────────┘
```

### Component Details

#### Perception Module
- **Models Used**: [YOLO, Mask R-CNN, SAM, Grounding DINO, etc.]
- **Input**: RGB + Depth images from [camera type]
- **Output**: Scene graph (JSON with objects, affordances, relationships)
- **Latency Target**: `<1`00ms
- **ROS 2 Node**: [Node name and topic mappings]

#### Language Module
- **Speech-to-Text**: Whisper API / [alternative]
- **LLM**: GPT-4 / Claude / [mock/local model]
- **Prompting Strategy**: [Describe system prompt, chain-of-thought approach]
- **Output**: JSON action sequence
- **Latency Target**: `<1` second

#### Control Module
- **Navigation Stack**: Nav2 with [costmap type, planner type]
- **Manipulation**: MoveIt with [arm model, gripper type]
- **Orchestrator**: [Description of central control loop]
- **Latency Target**: `<5`0ms per skill execution

---

## 3. Implementation Details

### 3.1 Perception Pipeline

**Selected Models**:
```
- Detection: [Model name and rationale]
- Segmentation: [Model name and rationale]
- Affordance: [Model name and rationale]
```

**ROS 2 Nodes**:
```yaml
perception_node:
  subscribers:
    - /camera/rgb/image_raw (sensor_msgs/Image)
    - /camera/depth/image_rect_raw (sensor_msgs/Image)
  publishers:
    - /perception/scene_graph (std_msgs/String with JSON)
    - /perception/detections_viz (sensor_msgs/Image)

performance:
  latency_ms: [actual measured latency]
  accuracy_percent: [measured on test set]
```

**Code Reference**: [Link to your perception node implementation]

### 3.2 Language Planning Pipeline

**Whisper Configuration**:
```python
# Model size: [tiny/base/small/medium/large]
# Language: en
# Confidence threshold: [0.0-1.0]
```

**LLM System Prompt**:
```
[Paste your full system prompt here]
```

**Skill Library**:
```json
{
  "skills": [
    {"name": "MoveTo", "params": ["x", "y", "z"]},
    {"name": "Pick", "params": ["object", "gripper_force"]},
    {"name": "Place", "params": ["location", "release_speed"]},
    [... other skills ...]
  ]
}
```

**Example Decomposition**:
```
User Command: "Pick up the red cup and place it on the shelf"

LLM Output:
{
  "steps": [
    {"skill": "search", "params": {"object": "red cup"}},
    {"skill": "moveto", "params": {"position": [0.5, 0.3, 0.8]}},
    {"skill": "pick", "params": {"gripper_force": "moderate"}},
    {"skill": "moveto", "params": {"position": [0.8, 1.0, 1.2]}},
    {"skill": "place", "params": {"release_speed": "slow"}}
  ]
}
```

**Code Reference**: [Link to LLM integration and task decomposer]

### 3.3 Control Architecture

**ROS 2 Action Servers Used**:
- [ ] Nav2: NavigateToPose
- [ ] MoveIt: MoveGroup
- [ ] Custom: [Other action servers]

**VLA Orchestrator**:
```python
# High-level pseudocode for orchestrator
while robot_is_running:
    1. Wait for voice command
    2. Run perception pipeline → scene_graph
    3. Call LLM with scene context → task_plan
    4. For each step in task_plan:
       a. Execute skill via ROS 2 action
       b. Monitor feedback
       c. If failure: trigger replanning
    5. Report task completion or failure
```

**Safety Mechanisms**:
- [ ] Watchdog timer (timeout: [X seconds])
- [ ] Emergency stop button
- [ ] Collision detection and avoidance
- [ ] Joint limit enforcement
- [ ] Gripper force limiting

**Code Reference**: [Link to orchestrator and safety implementations]

---

## 4. Evaluation Results

### 4.1 Perception Evaluation

| Metric | Target | Achieved | Evidence |
|--------|--------|----------|----------|
| Object Detection Accuracy | >90% | [X]% | [Command to run test, screenshot] |
| Affordance Detection Accuracy | >80% | [X]% | [Test results] |
| Segmentation Precision | >85% | [X]% | [Test results] |
| Latency (per frame) | `<1`00ms | [X]ms | [Benchmark output] |

**Test Dataset**: [Description: synthetic, real, mixed]

**Failure Analysis**: [What types of objects/scenes cause failures?]

### 4.2 Language Planning Evaluation

| Metric | Target | Achieved | Evidence |
|--------|--------|----------|----------|
| Speech Recognition (Whisper) | >95% WER | [X]% | [Test audio samples] |
| Task Decomposition Quality | >80% | [X]% | [5+ task examples with LLM output] |
| Parsing Success Rate | >95% | [X]% | [Test run logs] |
| Planning Latency | `<1`s | [X]s | [Benchmark] |

**Test Cases**:
- [ ] Simple single-step tasks (e.g., "pick up cup")
- [ ] Multi-step tasks (e.g., "pick, move, place")
- [ ] Ambiguous commands (e.g., "grab something red")
- [ ] Novel object names (not in training data)

**Failure Examples**: [Describe commands that failed and why]

### 4.3 Control & Integration Evaluation

| Metric | Target | Achieved | Evidence |
|--------|--------|----------|----------|
| Navigation Accuracy | ±5cm | ±[X]cm | [Test run video] |
| Pick Success Rate | >80% | [X]% | [Count of successful picks in 10 trials] |
| Multi-Step Execution | 3+ tasks | [X] tasks | [Demo video] |
| Control Loop Frequency | 30 Hz | [X] Hz | [ROS 2 timing logs] |
| Replanning Latency | `<2`s | [X]s | [Failure recovery logs] |

**Test Scenarios**:
- [ ] Simple navigation
- [ ] Pick and place
- [ ] Multi-step household task
- [ ] Failure recovery (dropped object, blocked path)
- [ ] Real-time performance under load

### 4.4 Safety Evaluation

| Feature | Status | Evidence |
|---------|--------|----------|
| Watchdog Timer | [Working/Not Working] | [Test logs] |
| Emergency Stop | [Working/Not Working] | [Demo video] |
| Collision Avoidance | [Working/Not Working] | [Test with obstacle] |
| Joint Limits | [Enforced/Not Enforced] | [Test configuration] |
| Graceful Degradation | [Yes/No] | [Behavior on component failure] |

**Stress Test Results**: [Max commands/sec, peak latency under load, recovery behavior]

---

## 5. Technical Challenges & Solutions

### Challenge 1: [Challenge Name]

**Problem**: [What went wrong?]

**Impact**: [How did it affect the system?]

**Solution Attempted**: [What did you try?]

**Final Resolution**: [What worked?]

**Lessons Learned**: [What would you do differently?]

---

### Challenge 2: [Challenge Name]

[Same structure as above]

---

## 6. Code & Documentation

### Repository Structure

```
capstone-vla-system/
├── README.md                      # Quick start guide
├── requirements.txt               # Python dependencies
├── Dockerfile                     # Reproducible environment
├── ros2_ws/                       # ROS 2 workspace
│   ├── src/
│   │   ├── perception_node/       # YOLO + depth fusion
│   │   ├── planning_node/         # Whisper + LLM
│   │   ├── control_node/          # VLA orchestrator
│   │   └── safety_watchdog/       # Safety mechanisms
│   └── launch/
│       └── full_pipeline.launch.py
├── models/                        # Pretrained models
│   ├── yolov8m.pt                # YOLO weights
│   └── affordance_classifier.pth  # Custom affordance model
├── config/                        # Configuration files
│   ├── robot_params.yaml          # Robot specifications
│   ├── camera_calib.yaml          # Camera intrinsics
│   └── llm_config.json            # LLM prompts and settings
├── docs/                          # Documentation
│   ├── ARCHITECTURE.md            # System design
│   ├── DEPLOYMENT.md              # Setup instructions
│   └── TROUBLESHOOTING.md         # Common issues
└── tests/                         # Evaluation tests
    ├── test_perception.py         # Perception accuracy
    ├── test_planning.py           # LLM quality
    └── test_integration.py        # Full pipeline
```

### Key Files & Line References

[Provide links or line numbers to key implementations:]

- Perception node: `ros2_ws/src/perception_node/perception_node.py:1-150`
- LLM integration: `ros2_ws/src/planning_node/llm_decomposer.py:1-100`
- VLA orchestrator: `ros2_ws/src/control_node/vla_orchestrator.py:1-200`
- Safety watchdog: `ros2_ws/src/safety_watchdog/watchdog.py:1-80`

### Running the System

```bash
# Build ROS 2 workspace
cd ros2_ws
colcon build

# Launch full pipeline
ros2 launch vla_system full_pipeline.launch.py

# Test with sample commands
ros2 topic pub /user/voice_command std_msgs/String "data: 'pick up the red cup'"

# View perception output
ros2 topic echo /perception/scene_graph

# Monitor control execution
ros2 topic echo /vla/status
```

---

## 7. Bonus: Hardware Deployment (Optional)

### Jetson Deployment

**Hardware Used**: [Jetson Orin Nano / Orin NX / Orin]

**Model Optimization**:
- [ ] Quantization: FP32 → INT8 (estimated [X]x speedup)
- [ ] Pruning: Reduced [X]% of weights
- [ ] TensorRT optimization: [Achieved latency]

**Real Hardware Results**:

| Component | Simulation | Real Hardware | Speedup |
|-----------|-----------|---------------|---------|
| Perception | [X]ms | [X]ms | [X]x |
| LLM Planning | [X]s | [X]s | N/A (cloud) |
| Control Loop | [X]ms | [X]ms | [X]x |
| **Total Cycle** | [X]s | [X]s | [X]x |

**Deployment Steps**:
```bash
# Quantize models
python quantize_models.py --model yolov8m --target int8

# Deploy to Jetson
scp -r ros2_ws jetson@robot:/home/jetson/
ssh jetson@robot
cd /home/jetson/ros2_ws
ros2 launch vla_system full_pipeline.launch.py
```

**Real Hardware Testing**:
- [ ] Navigation in real environment: [Success rate, max speed]
- [ ] Manipulation on real robot: [Pick success %, force feedback accuracy]
- [ ] Multi-step task on hardware: [Video link, success/failure analysis]

---

## 8. Future Work & Recommendations

### Known Limitations

- [Limitation]: [Why it exists, impact]
- [Limitation]: [Why it exists, impact]
- [Limitation]: [Why it exists, impact]

### Recommended Improvements

**Short-term** (1-2 weeks):
- [Improvement 1]: Why important, estimated effort
- [Improvement 2]: Why important, estimated effort

**Medium-term** (1-2 months):
- [Improvement 1]
- [Improvement 2]

**Long-term** (Research directions):
- [Direction 1]: Multi-agent coordination
- [Direction 2]: Continual learning
- [Direction 3]: Real sim-to-real transfer

---

## 9. References & Acknowledgments

### Academic References

- [Brohan et al. 2023] "RT-2: Vision-Language-Action Models" - arXiv:2307.15818
- [Padalkar et al. 2024] "RT-X: Robotics Transformer" - DeepMind Blog
- [Your other references...]

### Code & Library Credits

- YOLO: Ultralytics (https://github.com/ultralytics/yolov8)
- Whisper: OpenAI (https://github.com/openai/whisper)
- ROS 2: Open Robotics (https://www.ros.org/)
- MoveIt: PickNik Robotics (https://moveit.ros.org/)
- Nav2: Open Robotics (https://nav2.org/)

### Acknowledgments

[Thank advisors, team members, resources that helped]

---

## 10. Appendices

### A. Dataset Statistics

[If using a custom dataset for perception or affordance training]

- Total images: [X]
- Train/test split: [X%] / [X%]
- Object classes: [List]
- Affordance labels: [List]

### B. Hyperparameters

[YOLO training, LLM prompting, MoveIt planning params]

### C. Failure Case Analysis

[Document 5-10 failure cases with root cause analysis]

### D. Performance Profiling

[Latency breakdown, CPU/GPU usage, memory consumption]

---

## Submission Checklist

Before submitting, verify:

- [ ] All code pushed to GitHub with README.md
- [ ] Docusaurus builds with zero MDX errors
- [ ] All 6 chapters referenced and cross-linked
- [ ] Evaluation rubric completed (perception, planning, control, safety)
- [ ] Demo video recorded (5 minutes max) showing full system
- [ ] Technical report with all sections (1-10) completed
- [ ] Architecture diagram included
- [ ] At least 3 test scenarios documented with results
- [ ] Bonus section attempted (hardware or advanced features)
- [ ] All code examples have proper error handling
- [ ] Dockerfile or requirements.txt for reproducibility

---

## Contact & Support

**Project Lead**: [Your name, email]

**Team Members**: [Names and roles]

**Repository**: [GitHub link]

**Demo Video**: [YouTube/link]

**Technical Report**: [Link to PDF or Google Doc]

---

**Good luck with your capstone! Remember: start simple (single skill), build incrementally (add skills/tasks), and validate thoroughly (test each component separately).**

**You've got this! 🚀**
