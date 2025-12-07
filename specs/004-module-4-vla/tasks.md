# Implementation Tasks: Module 4 - Vision–Language–Action (VLA) Robotics

**Feature Branch**: `004-module-4-vla` | **Spec**: `/specs/004-module-4-vla/spec.md` | **Plan**: `/specs/004-module-4-vla/plan.md`

**Total Tasks**: 158 (original 145 + 13 new edge case/optimization/metadata subtasks) | **Organized by**: 5 User Stories + Setup + Polish | **Deliverables**: 6 markdown files, 5,875+ lines, 25,000+ words

---

## Phase 1: Setup & Infrastructure

**Goal**: Initialize module structure, configure documentation framework, establish RAG metadata patterns

**Independent Test**: Docusaurus can build module-4 directory with valid structure (no errors, all files created)

### Content Structure & Configuration

- [ ] T001 Create `/docs/module-4/` directory with proper structure
- [ ] T002 Create `/docs/module-4/index.md` with module landing page structure (TOC, learning outcomes, capstone preview)
- [ ] T003 Update `sidebars.ts` to include Module 4 navigation under "Learning Modules" section
- [ ] T004 Create RAG metadata template for Module 4 chapters (module, week, difficulty, topics, prerequisites)
- [ ] T005 [P] Configure Docusaurus MDX settings to validate Markdown syntax (no unescaped `<NUMBER` patterns)

### Documentation Standards & Examples

- [ ] T006 Create `.specify/examples/module-4-chapter-template.md` showing pedagogical structure (learning objectives, key concepts, theory, hands-on labs, code examples, exercises)
- [ ] T007 [P] Create code example comment standards file documenting: File paths, inline comments, error handling requirements, type hints for Python 3.10+
- [ ] T008 [P] Create Mermaid diagram standards file with examples for VLA architecture, perception flow, LLM decision trees, action graphs

### RAG Metadata & Semantic Chunking (Phase 1)

**Critical for Constitution IV (RAG-First Documentation)**: Establish metadata patterns upfront to ensure all content is written RAG-compatible

- [ ] T008a [P] Create RAG metadata schema for Module 4 (module, week, difficulty, topics, prerequisites, estimated time, learning outcomes)
- [ ] T008b [P] Create semantic chunking guidelines: sections as discrete retrieval units (learning objectives, key concepts, tutorials, code examples, exercises = separate chunks)
- [ ] T008c [P] Add metadata annotation examples to chapter template (how to mark code example IDs, exercise section boundaries, concept definitions)

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Goal**: Create foundational chapters that all user stories depend on

**Independent Test**: Chapter 1 builds successfully, students understand VLA architecture fundamentals

### Chapter 1: Introduction to Vision–Language–Action Robotics (820 lines, ~10 labs/examples)

**Difficulty**: Beginner | **Estimated Time**: 8-10 hours | **Prerequisite**: Modules 1-3 knowledge

#### Part 1: VLA Fundamentals & Architecture (180 lines)

- [ ] T009 Write learning objectives section for Chapter 1 (5 measurable outcomes on VLA concepts)
- [ ] T010 Write key concepts section identifying: VLA definition, three pillars (V-L-A), why important for humanoids, cognitive robotics vs classical
- [ ] T011 [P] Write "What is VLA?" subsection (15 lines): definition, motivation, emerging field status
- [ ] T012 [P] Write "Why VLA is Essential" subsection (20 lines): humanoid autonomy, real-world task complexity, industry adoption
- [ ] T013 Write VLA system architecture overview section explaining: Language Model (LLM), Vision Perception, Action Controller (ROS 2) with diagram
- [ ] T014 Create Mermaid diagram: VLA system architecture with 3 main components and data flow arrows
- [ ] T015 Write "Cognitive Robotics vs. Classical Robotics" comparison section (25 lines): classical (pre-programmed) vs. VLA (adaptive)

#### Part 2: Example VLA Systems & Industry Use Cases (200 lines)

- [ ] T016 Write "Example VLA Systems" section covering: OpenAI Robotics, DeepMind RT-X, NVIDIA VLM pipelines (20 lines each)
- [ ] T017 Create Mermaid timeline diagram showing evolution of VLA (2020→2025, key papers/systems)
- [ ] T018 Write "Industry Applications" section: warehouse humanoids, household humanoids, assistive robotics (25 lines each)
- [ ] T019 [P] Write case study: "Amazon Digit + GPT Integration" (30 lines, real-world example)
- [ ] T020 [P] Write case study: "Home Assistant Robot Task Planning" (30 lines, real-world example)

#### Part 3: Task Representation Patterns (220 lines)

- [ ] T021 Write "Task Representation" section introducing: Skills, Behaviors, Action Graphs
- [ ] T022 Write "Skills" subsection: definition, examples (MoveTo, Pick, Place, Search), composition
- [ ] T023 Write "Behaviors" subsection: definition, state machines, behavior trees, examples
- [ ] T024 Create Mermaid diagram: Behavior tree example for "Pick object from table"
- [ ] T025 Write "Action Graphs" subsection: definition, ROS 2 action pattern, feedback loops, error handling
- [ ] T026 Create Mermaid diagram: Action graph for simple manipulation task (locate → navigate → grasp → place)
- [ ] T027 Write pseudocode example: Simple behavior tree for object search (15 lines)

#### Part 4: Labs & Exercises (220 lines)

- [ ] T028 Write Lab 1: "Analyze a Real VLA System" (analyze OpenAI Robotics or RT-X architecture, 45 minutes)
  - [ ] T028a Create acceptance criteria for Lab 1 (3+ items: identify components, explain data flow, etc.)
  - [ ] T028b Write step-by-step instructions for Lab 1
  - [ ] T028c Create Lab 1 evaluation rubric

- [ ] T029 Write Lab 2: "Design a VLA Architecture for Household Cleanup" (design task plan, 1 hour)
  - [ ] T029a Create acceptance criteria for Lab 2
  - [ ] T029b Write step-by-step instructions
  - [ ] T029c Create diagrams for expected Lab 2 outputs

- [ ] T030 Write Lab 3: "Evaluate VLA Trade-offs" (compare approaches, 45 minutes)
  - [ ] T030a Create acceptance criteria for Lab 3
  - [ ] T030b Write step-by-step instructions

- [ ] T031 [P] Write 3+ end-of-chapter exercises with progressive difficulty (simple→complex)
  - [ ] T031a Exercise 1: Identify VLA components in a given system description
  - [ ] T031b Exercise 2: Design task decomposition for a novel scenario
  - [ ] T031c Exercise 3: Propose VLA system for real-world robotics company

#### Part 5: Capstone Connection (50 lines)

- [ ] T032 Write "Capstone Integration" subsection explaining: how Chapter 1 concepts apply to final humanoid project, what students will build

#### Validation & Polish

- [ ] T033 Validate Chapter 1: all code examples tested, all links working, RAG metadata correct
- [ ] T034 [P] Copy Chapter 1 content to `/docs/module-4/chapter-1-vla-intro.md`
- [ ] T035 Verify Docusaurus builds Chapter 1 without errors (MDX, links, syntax)

---

## Phase 3: User Story 1 - Build Speech-Controlled Robot Perception Pipeline (P1)

**Goal**: Create Chapter 2 teaching perception pipelines and integration with speech/LLM

**Acceptance Criteria**: Student can implement object detection, depth fusion, and convert scene to LLM-readable format; end-to-end pipeline works in simulation

**Independent Test**: Chapter 2 labs complete successfully with >90% detection accuracy on test images

### Chapter 2: Vision Pipelines for VLA (880 lines, ~10 labs/examples)

**Difficulty**: Intermediate | **Prerequisite**: Chapter 1 | **Estimated Time**: 10-12 hours

#### Part 1: Perception Stack Overview (200 lines)

- [ ] T036 Write learning objectives for Chapter 2 (5 measurable outcomes on perception)
- [ ] T037 Write key concepts section: RGB+depth+segmentation, YOLO vs Mask R-CNN vs SAM tradeoffs
- [ ] T038 Write "Perception Stack for VLA" section explaining: cameras, sensors, models, data flow
- [ ] T039 Create Mermaid diagram: Complete perception pipeline (image → YOLO → depth → segmentation → scene graph)
- [ ] T040 [P] Write "Camera & Depth Sensors" subsection (30 lines): RGB cameras, depth sensors (stereo, ToF, LiDAR), ROS message types
- [ ] T041 [P] Write "Object Detection Models" subsection (50 lines): YOLO architecture & tradeoffs, Mask R-CNN, SAM, Grounding DINO overview
- [ ] T042 Write "Segmentation & Affordances" subsection (40 lines): instance segmentation, affordance detection (graspable, movable, etc.)
- [ ] T043 Write "Scene Graphs" subsection (30 lines): extracting spatial relationships, object-object interactions, converting to LLM input format

#### Part 2: Multi-Modal Fusion & Analysis (250 lines)

- [ ] T044 Write "RGB + Depth Fusion" section explaining: coordinate transforms, reprojection, depth inference
- [ ] T045 Create Mermaid diagram: RGB-D fusion pipeline (image + depth → 3D point cloud)
- [ ] T046 Write code example: ROS 2 node that subscribes to RGB + depth topics and outputs fused image (30 lines)
- [ ] T047 [P] Write "Multi-View Camera Fusion" section (40 lines): multiple camera setup, timestamp synchronization, 3D reconstruction
- [ ] T048 [P] Write "Object Affordance Detection" section (40 lines): identifying graspable vs non-interactive objects
- [ ] T049 Write pseudocode: Convert scene to structured JSON for LLM (20 lines) showing:
  ```
  {
    "objects": [
      {"id": 1, "class": "cup", "bbox": [...], "affordance": "graspable", "location": {...}},
      ...
    ],
    "relationships": [{"obj1": 1, "obj2": 2, "relation": "on_top_of"}]
  }
  ```

#### Part 3: ROS 2 Integration Patterns (150 lines)

- [ ] T050 Write "ROS 2 Integration" section covering: sensor_msgs/Image, sensor_msgs/PointCloud2, tf frames, subscription patterns
- [ ] T051 Write code example: ROS 2 perception node subscribing to camera and publishing detections (40 lines, with error handling)
- [ ] T052 Create Mermaid diagram: ROS 2 node graph for perception pipeline (camera → YOLO → segmentation → scene graph node)
- [ ] T053 Write code example: YOLO detection ROS 2 node using Ultralytics (50 lines, Python 3.10+)
- [ ] T054 [P] Write code example: Depth processing ROS 2 node (40 lines)
- [ ] T055 [P] Write code example: Scene graph extraction node (45 lines)

#### Part 4: Labs & Exercises (230 lines)

- [ ] T056 Write Lab 1: "Detect Objects in a Scene Image" (1 hour)
  - [ ] T056a Create acceptance criteria (>90% accuracy, detects all object classes)
  - [ ] T056b Write step-by-step instructions using YOLO on Isaac Sim or sample images
  - [ ] T056c Provide test images with ground truth annotations

- [ ] T057 Write Lab 2: "Identify Pickable Objects" (1.5 hours)
  - [ ] T057a Create acceptance criteria (correctly identifies graspable vs non-graspable)
  - [ ] T057b Write step-by-step instructions
  - [ ] T057c Create affordance labeling tool or reference dataset

- [ ] T058 Write Lab 3: "Convert Scene to LLM-Readable Format" (1 hour)
  - [ ] T058a Create acceptance criteria (JSON valid, all objects represented, relationships correct)
  - [ ] T058b Write step-by-step instructions
  - [ ] T058c Provide reference Python code and test scenes

- [ ] T059 [P] Write 3+ end-of-chapter exercises:
  - [ ] T059a Exercise 1: Compare YOLO vs Mask R-CNN tradeoffs for different scenarios
  - [ ] T059b Exercise 2: Debug perception failure (missing object detection) with provided scene
  - [ ] T059c Exercise 3: Design perception pipeline for novel robot gripper

#### Part 5: Capstone Connection (50 lines)

- [ ] T060 Write "Capstone Integration" subsection: how perception feeds into LLM planning in final project

#### Validation & Polish

- [ ] T061 Test all code examples in ROS 2 Humble + Isaac Sim environment (detection >90% accuracy)
- [ ] T062 Verify all Mermaid diagrams render correctly in Docusaurus
- [ ] T063 [P] Copy Chapter 2 content to `/docs/module-4/chapter-2-vision-for-vla.md`
- [ ] T064 Verify Docusaurus builds Chapter 2 without errors

---

## Phase 4: User Story 2 - Master Vision Perception for Robotics (P1, overlaps with Phase 3)

**Note**: Chapter 2 labs address this story. Additional tasks for depth understanding:

- [ ] T065 Write advanced lab: "Multi-View 3D Reconstruction" (1.5 hours, optional/bonus)
  - [ ] T065a Create acceptance criteria (reconstruct 3D point cloud from 2+ views)
  - [ ] T065b Write step-by-step instructions
  - [ ] T065c Provide sample multi-view image sets

- [ ] T066 Write advanced exercise: "Optimize Perception Pipeline for Latency" (bonus)
  - [ ] T066a Create acceptance criteria: Achieve inference <100ms per frame AND measure <5% accuracy loss after optimization
  - [ ] T066b Write INT8 quantization lab: Convert FP32 YOLO model to INT8 using TensorRT, measure speedup (3-5x expected)
  - [ ] T066b-i Step-by-step instructions for YOLO quantization with TensorRT on Jetson
  - [ ] T066b-ii Provide baseline model + quantized model for comparison
  - [ ] T066b-iii Accuracy validation: measure mAP before/after quantization
  - [ ] T066c Write optimization guidance covering: model selection (YOLOv8n vs v8m tradeoffs), batching strategies, GPU utilization profiling
  - [ ] T066d Create comparison table: original latency vs quantized latency, accuracy metrics (baseline <100ms target)

---

## Phase 5: User Story 3 - Design and Execute LLM-Driven Task Decomposition (P1)

**Goal**: Create Chapter 3 teaching Whisper, LLMs, and task decomposition with mock+real APIs

**Acceptance Criteria**: Student can convert natural language to executable ROS 2 action sequences; uses mock OR real LLM APIs

**Independent Test**: Chapter 3 labs produce valid ROS 2 action plans from natural language inputs

### Chapter 3: Language & Planning - Whisper, LLMs, and Task Decomposition (920 lines, ~10 labs/examples)

**Difficulty**: Intermediate-Advanced | **Prerequisite**: Chapters 1-2 | **Estimated Time**: 12-14 hours

#### Part 1: Whisper Speech-to-Text (180 lines)

- [ ] T067 Write learning objectives for Chapter 3 (5 outcomes on Whisper, LLMs, task decomposition)
- [ ] T068 Write "Speech Recognition with Whisper" section (60 lines): architecture, accuracy, model sizes, ROS 2 integration
- [ ] T069 Create Mermaid diagram: Whisper pipeline (audio → tokens → text)
- [ ] T070 Write code example: Whisper ROS 2 node (50 lines, Python 3.10+, with error handling)
- [ ] T071 Write code example: Whisper Python script (standalone, 30 lines, for testing)
- [ ] T072 [P] Write "Handling Noisy Input" section (30 lines): error recovery, confidence thresholds, fallback mechanisms

#### Part 2: LLM Fundamentals & Prompting (250 lines)

- [ ] T073 Write "Large Language Models for Robotics" section (60 lines): GPT-4, Claude, open-source options
- [ ] T074 Write "Prompting Strategies" section covering:
  - [ ] T074a Zero-shot action planning (explain, example, 30 lines)
  - [ ] T074b Tool use / function calling (explain, example, 30 lines)
  - [ ] T074c Chain-of-thought reasoning (explain, example, 30 lines)

- [ ] T075 Create Mermaid diagram: LLM prompting loop (prompt → inference → action parsing → execution → feedback)
- [ ] T076 Write pseudocode: Example prompts for task decomposition (50 lines showing:
  - Input: "Clean the table"
  - Output: Structured action plan with steps, parameters, conditional logic

- [ ] T077 [P] Write "Mock LLM Implementation" section (40 lines): explanation of mock responses for core labs
- [ ] T078 [P] Write code example: Mock LLM module (60 lines, Python, with standard responses for common scenarios)
  - [ ] T078a Acceptance criteria: Mock LLM module runs without API key setup, returns valid JSON action plans for standard scenarios
  - [ ] T078b Include 5+ hardcoded response scenarios (pick cup, clean table, fetch object, navigate room, place item)
  - [ ] T078c Clearly document: "No API keys required for this module"
- [ ] T079 Write code example: Real OpenAI API integration (50 lines, with feature gate for bonus content)

#### Part 3: Task Decomposition & Planning (250 lines)

- [ ] T080 Write "Task Decomposition" section explaining: hierarchical planning, sequential actions, conditional logic, loops
- [ ] T081 Create Mermaid diagram: Task decomposition example ("pick red cup") breaking down to skills
- [ ] T082 Write "Skill Representation" section (40 lines): MoveTo, Pick, Place, Search, FollowHuman primitives with ROS 2 action mappings
- [ ] T083 Write "Handling Failures & Replanning" section (50 lines): error detection, replanning logic, feedback loops
- [ ] T084 Write pseudocode: Replanning algorithm (25 lines, pseudocode showing failure detection and alternative generation)
- [ ] T085 Write code example: ROS 2 action executor with replanning (60 lines, Python, implements basic state machine)
- [ ] T086 [P] Write "Behavior Trees for Task Planning" section (50 lines): structure, selectors/sequences, action nodes
- [ ] T087 Create Mermaid diagram: Behavior tree example for complex task

#### Part 4: Labs & Exercises (200 lines)

- [ ] T088 Write Lab 1: "Test Whisper Speech Recognition" (45 minutes)
  - [ ] T088a Create acceptance criteria (recognize >95% accuracy on clear speech)
  - [ ] T088b Write step-by-step instructions
  - [ ] T088c Provide test audio samples (clear + noisy variants)

- [ ] T089 Write Lab 2: "Decompose Tasks with Mock LLM" (1 hour)
  - [ ] T089a Create acceptance criteria (convert 5 natural language tasks to action plans)
  - [ ] T089b Write step-by-step instructions
  - [ ] T089c Provide test scenarios with expected outputs

- [ ] T090 Write Lab 3: "Implement Replanning Logic" (1.5 hours)
  - [ ] T090a Create acceptance criteria (system recovers from 3+ failure types)
  - [ ] T090b Write step-by-step instructions
  - [ ] T090c Provide failure scenarios to test

- [ ] T091 Write Lab 4: "Real LLM Integration (Bonus)" (1.5 hours, optional)
  - [ ] T091a Create acceptance criteria (OpenAI API integration working, same task success as mock)
  - [ ] T091b Write step-by-step instructions
  - [ ] T091c Require API key setup documentation

- [ ] T092 [P] Write 3+ end-of-chapter exercises:
  - [ ] T092a Exercise 1: Design prompt for "Navigate to kitchen"
  - [ ] T092b Exercise 2: Debug LLM plan failure (infeasible action)
  - [ ] T092c Exercise 3: Extend skill library with new primitive

#### Part 5: Capstone Connection (40 lines)

- [ ] T093 Write "Capstone Integration" subsection: Whisper + LLM planning in final project pipeline

#### Validation & Polish

- [ ] T094 Test Whisper integration with sample audio (>95% accuracy)
- [ ] T094a Test mock LLM module with 10+ scenarios
- [ ] T094b Test real API examples with OpenAI/Claude (if available)
- [ ] T095 [P] Copy Chapter 3 content to `/docs/module-4/chapter-3-language-planning-whisper-llm.md`
- [ ] T096 Verify Docusaurus builds Chapter 3 without errors

---

## Phase 6: User Story 4 - Integrate Perception, Planning, and Control into Unified VLA Architecture (P2)

**Goal**: Create Chapter 4 teaching ROS 2 control, integration, safety, and deployment design

**Acceptance Criteria**: Student can wire up perception → LLM → control pipeline; system executes tasks with error handling

**Independent Test**: Chapter 4 capstone architecture works end-to-end in simulation with all three VLA components

### Chapter 4: VLA Control Architecture, Integration & Deployment Design (850 lines, ~10 labs/examples)

**Difficulty**: Advanced | **Prerequisite**: Chapters 1-3 | **Estimated Time**: 12-14 hours

#### Part 1: ROS 2 Control & Navigation (200 lines)

- [ ] T097 Write learning objectives for Chapter 4 (5 outcomes on control, integration, deployment)
- [ ] T098 Write "ROS 2 Control Fundamentals" section (50 lines): action servers, feedback, goal handling
- [ ] T099 Write "Navigation with Nav2" section (50 lines): costmaps, planners, action interfaces
- [ ] T100 Write "Manipulation Primitives" section (50 lines): MoveIt for arm planning, gripper control
- [ ] T101 [P] Create Mermaid diagram: ROS 2 action graph for humanoid (nav, manipulation, perception)
- [ ] T102 [P] Write code example: Nav2 action client for navigation (40 lines, Python)
- [ ] T103 Write code example: Gripper control ROS 2 node (30 lines)

#### Part 2: VLA Pipeline Integration (200 lines)

- [ ] T104 Write "VLA Pipeline Overview" section (60 lines): perception → LLM → control flow, component orchestration
- [ ] T105 Create Mermaid diagram: Complete VLA pipeline (Whisper → LLM → perception check → nav → manipulation)
- [ ] T106 Write "Data Flow & Synchronization" section (40 lines): timestamp handling, message buffering, state management
- [ ] T107 Write "Feedback Loops" section (50 lines): closed-loop control, perception updates, status monitoring
- [ ] T108 Write code example: VLA orchestrator node (70 lines, Python, combines all components)

#### Part 3: Safety, Failure Handling & Deployment Design (250 lines)

- [ ] T109 Write "Safety Mechanisms" section (60 lines): watchdogs, emergency stops, collision avoidance, joint limits
- [ ] T110 Write "Failure Recovery" section (50 lines): grasping failures, navigation deadlocks, task abortion
- [ ] T111 Create Mermaid diagram: Error recovery state machine
- [ ] T112 Write code example: Watchdog implementation (40 lines, Python, monitors control loop timing)
- [ ] T113 Write "Deployment Design Considerations" section (80 lines):
  - [ ] T113a Model optimization (quantization, pruning, 20 lines)
  - [ ] T113b Latency budgeting (30ms/33ms per cycle, 20 lines)
  - [ ] T113c Real-time constraints (30 lines)
  - [ ] T113d Safety mechanisms for hardware (20 lines)

- [ ] T114 [P] Write "Jetson Deployment Overview" section (40 lines): hardware specs, TensorRT, power management (conceptual, not executable)

#### Part 4: Labs & Exercises (200 lines)

- [ ] T115 Write Lab 1: "Build Perception-Aware Navigation" (1.5 hours)
  - [ ] T115a Create acceptance criteria (navigate around perceived obstacles)
  - [ ] T115b Write step-by-step instructions
  - [ ] T115c Provide test scenarios

- [ ] T116 Write Lab 2: "Integrate LLM Planning with Control" (1.5 hours)
  - [ ] T116a Create acceptance criteria (LLM plan executes successfully)
  - [ ] T116b Write step-by-step instructions
  - [ ] T116c Provide test task scenarios

- [ ] T117 Write Lab 3: "Implement Safety Watchdogs" (1 hour)
  - [ ] T117a Create acceptance criteria (watchdog detects timeout, stops execution)
  - [ ] T117b Write step-by-step instructions
  - [ ] T117c Provide test scenarios

- [ ] T118 Write Lab 4: "Design Deployment Optimization Strategy" (1.5 hours, design exercise)
  - [ ] T118a Create acceptance criteria (document optimization plan, measure latency)
  - [ ] T118b Write step-by-step instructions
  - [ ] T118c Provide baseline models for optimization

- [ ] T119 [P] Write 3+ end-of-chapter exercises:
  - [ ] T119a Exercise 1: Optimize perception model for 100ms latency budget
  - [ ] T119b Exercise 2: Design replanning strategy for gripper failure
  - [ ] T119c Exercise 3: Propose safety mechanisms for household deployment

#### Part 5: Capstone Connection (50 lines)

- [ ] T120 Write "Capstone Integration" subsection: full VLA system architecture for final project

#### Validation & Polish

- [ ] T121 Test VLA orchestrator with mock components in simulation
- [ ] T122 Verify latency budgets (<33ms per cycle in simulation)
- [ ] T123 [P] Copy Chapter 4 content to `/docs/module-4/chapter-4-vla-control-architecture.md`
- [ ] T124 Verify Docusaurus builds Chapter 4 without errors

---

## Phase 7: User Story 5 - Deploy VLA System to Real Hardware (P3, Optional/Bonus)

**Goal**: Create bonus hardware deployment content and capstone sprint guide

**Acceptance Criteria**: Students understand deployment process; capstone can run on Jetson (if available); design exercise done

### Week 13 Practice Guide (1,100 lines, daily activities + capstone sprint)

**Difficulty**: Advanced | **Prerequisite**: Chapters 1-4 | **Estimated Time**: 30-40 hours (week 13 compressed course)

#### Part 1: Weekly Overview & Progression (150 lines)

- [ ] T125 Write Week 13 introduction explaining: daily structure, learning path through all chapters, capstone sprint
- [ ] T126 Write Week 13 learning objectives (7 measurable outcomes covering all VLA pillars)
- [ ] T127 Create Mermaid diagram: Week 13 progression (daily topics and what students will complete)

#### Part 2: Day-by-Day Activities (600 lines, Monday-Friday × 2 weeks)

**Week 1 Days 1-5 (Chapters 1-2)**:

- [ ] T128 Write Day 1 activities (8-10 hours): Chapter 1 concepts + Lab 1 completion
  - [ ] T128a Review: VLA fundamentals
  - [ ] T128b Activity 1: Analyze RT-X or OpenAI Robotics system
  - [ ] T128c Activity 2: Design VLA for household task
  - [ ] T128d Success criteria (3 items)

- [ ] T129 Write Day 2 activities (8-10 hours): Chapter 1 labs 2-3 completion
  - [ ] T129a Lab 2: Design humanoid cleanup strategy
  - [ ] T129b Lab 3: Evaluate VLA tradeoffs
  - [ ] T129c Success criteria

- [ ] T130 Write Day 3 activities (8-10 hours): Chapter 2 perception introduction
  - [ ] T130a Review: Perception stack, YOLO vs Mask R-CNN
  - [ ] T130b Lab 1: Object detection on test images
  - [ ] T130c Success criteria (>90% accuracy)

- [ ] T131 Write Day 4 activities (8-10 hours): Chapter 2 labs 2-3
  - [ ] T131a Lab 2: Identify pickable objects
  - [ ] T131b Lab 3: Convert scene to LLM format
  - [ ] T131c Success criteria

- [ ] T132 Write Day 5 activities (8-10 hours): Chapter 2 exercises + reflection
  - [ ] T132a Exercise 1-3 completion
  - [ ] T132b Reflection: What perception challenges were hardest?
  - [ ] T132c Success criteria

**Week 2 Days 1-5 (Chapters 3-4 + Capstone)**:

- [ ] T133 Write Day 6 activities (8-10 hours): Chapter 3 Whisper + LLM
  - [ ] T133a Review: Whisper, LLM prompting
  - [ ] T133b Lab 1: Whisper speech recognition
  - [ ] T133c Lab 2: Mock LLM task decomposition
  - [ ] T133d Success criteria

- [ ] T134 Write Day 7 activities (8-10 hours): Chapter 3 labs 3-4
  - [ ] T134a Lab 3: Replanning logic
  - [ ] T134b Lab 4: Real LLM integration (bonus)
  - [ ] T134c Success criteria

- [ ] T135 Write Day 8 activities (8-10 hours): Chapter 4 control + integration
  - [ ] T135a Review: ROS 2 control, Nav2, safety
  - [ ] T135b Lab 1: Perception-aware navigation
  - [ ] T135c Lab 2: LLM + control integration
  - [ ] T135d Success criteria

- [ ] T136 Write Day 9 activities (8-10 hours): Chapter 4 labs 3-4 + deployment
  - [ ] T136a Lab 3: Safety watchdogs
  - [ ] T136b Lab 4: Deployment design exercise
  - [ ] T136c Success criteria

- [ ] T137 Write Day 10 activities (8-10 hours): Capstone sprint day 1-2
  - [ ] T137a Capstone checkpoint 1: Perception pipeline working
  - [ ] T137b Capstone checkpoint 2: LLM planning integrated
  - [ ] T137c Success criteria
  - [ ] T137d Bonus: Hardware deployment (if available)

#### Part 3: Capstone Project Definition & Sprint (250 lines)

- [ ] T138 Write Capstone Project Overview explaining:
  - [ ] T138a Project goal: "Build Autonomous Humanoid Assistant"
  - [ ] T138b Required components: Whisper + LLM + Perception + Control
  - [ ] T138c Deliverables: Working system + documentation + video demo
  - [ ] T138d Evaluation rubric: Perception 25%, LLM 25%, Control 35%, Safety 15%

- [ ] T139 Write Capstone Project Specification (150 lines):
  - [ ] T139a Voice input: "Pick up the red cup from the table and place it on the shelf"
  - [ ] T139b Perception requirements: Detect cup, estimate pose, check grasping feasibility
  - [ ] T139c Planning requirements: Generate valid action sequence with error handling
  - [ ] T139d Control requirements: Execute navigation + manipulation with safety monitoring
  - [ ] T139e Success criteria (3-5 measurable items)

- [ ] T140 [P] Write Capstone Sprint Plan (60 lines):
  - [ ] T140a Checkpoint 1 (Day 8): Perception pipeline complete
  - [ ] T140b Checkpoint 2 (Day 9): LLM planning integrated
  - [ ] T140c Checkpoint 3 (Day 10): Full system with safety
  - [ ] T140d Bonus: Hardware deployment

- [ ] T141 Write Capstone Evaluation Rubric (50 lines) with:
  - [ ] T141a Perception component: accuracy, affordance detection, robustness (25%)
  - [ ] T141b LLM component: plan quality, error recovery, flexibility (25%)
  - [ ] T141c Control component: execution reliability, timing, safety (35%)
  - [ ] T141d Safety component: watchdogs, e-stops, graceful degradation (15%)

#### Part 3b: Edge Case Handling Labs (Bonus)

**Critical gap from spec.md edge cases (lines 103-110)**: Explicit labs for edge case recovery

- [ ] T141a Write Lab 5: "Detect and Recover from LLM Hallucination" (1.5 hours, edge case)
  - [ ] T141a-i Create acceptance criteria: System detects infeasible action (grasp through wall), proposes alternative
  - [ ] T141a-ii Write step-by-step instructions with test scenarios
  - [ ] T141a-iii Provide reference LLM outputs that hallucinate

- [ ] T141b Write Lab 6: "Handle Perception Failures" (1 hour, edge case)
  - [ ] T141b-i Create acceptance criteria: Missing object detection handled gracefully, user notified, task aborted safely
  - [ ] T141b-ii Write step-by-step instructions
  - [ ] T141b-iii Provide test images with missing/occluded objects

- [ ] T141c Write Capstone Troubleshooting Guide (100 lines)
  - Covers: Perception <85% accuracy recovery, LLM plan infeasibility detection, control loop exceeding 33ms budget, gripper failures
  - Expected output: Decision tree for students on what to do when capstone system fails

#### Part 4: Bonus Hardware Deployment Content (100 lines, optional)

- [ ] T142 Write "Deploying to Jetson Hardware" section (100 lines):
  - [ ] T142a Hardware setup checklist (Jetson Orin Nano specs, ROS 2 installation)
  - [ ] T142b Model optimization for Jetson (TensorRT, quantization)
  - [ ] T142c Real-time control loop implementation (33ms budget)
  - [ ] T142d Safety mechanisms for physical robot (watchdogs, emergency stops)
  - [ ] T142e Troubleshooting common deployment issues

#### Validation & Polish

- [ ] T143 [P] Copy Week 13 guide to `/docs/module-4/week-13.md`
- [ ] T144 Verify Docusaurus builds Week 13 without errors
- [ ] T145 Final module-wide validation: all 6 files exist, all cross-references work

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Final validation, module integration, RAG metadata, build verification

### Module Index & Navigation

- [ ] T146 Write complete `/docs/module-4/index.md` (250 lines) including:
  - Module landing page with learning outcomes
  - Week-by-week progression
  - Capstone project preview
  - Prerequisites (Modules 1-3)
  - How to use this module (suggested path)

- [ ] T147 Verify sidebar configuration includes Module 4 navigation

### RAG Metadata & Semantic Chunking

- [ ] T148 Add RAG metadata to all 6 markdown files:
  - [ ] T148a Learning objectives metadata (difficulty, topics, time estimate)
  - [ ] T148b Key concepts section markers for semantic chunking
  - [ ] T148c Code example IDs (e.g., Example 4.2.1, Example 4.3.2)
  - [ ] T148d Exercise section markers for discovery

- [ ] T149 [P] Verify semantic heading hierarchy (H1→H2→H3→H4) across all files
- [ ] T150 [P] Validate all internal cross-references work (chapters, sections, code examples)

### Final Build & Validation

- [ ] T151 Run final Docusaurus build (< 60 seconds)
  - [ ] T151a Zero MDX errors (no unescaped `<NUMBER` patterns)
  - [ ] T151b Zero broken links
  - [ ] T151c All 6 Module 4 files render correctly

- [ ] T152 [P] Verify RAG indexing readiness:
  - [ ] T152a All sections chunked semantically
  - [ ] T152b Metadata present (difficulty, topics, prerequisites)
  - [ ] T152c All code examples discoverable

- [ ] T153 [P] Final technical accuracy check:
  - [ ] T153a All code examples tested in ROS 2 Humble (if applicable)
  - [ ] T153b All Mermaid diagrams render without errors
  - [ ] T153c All claims about APIs, libraries, versions verified (2025 versions)

### Documentation & Handoff

- [ ] T154 Create module completion report documenting:
  - [ ] Total content: 5,875+ lines, 25,000+ words
  - [ ] File breakdown: 6 files with line counts
  - [ ] Code examples: 12+ per chapter
  - [ ] Hands-on labs: 10+ total, 3+ per chapter
  - [ ] Exercises: 50+ progressive difficulty
  - [ ] Diagrams: 40+ Mermaid diagrams

- [ ] T155 Create Module 4 README for instructors with:
  - [ ] Learning objectives and progression
  - [ ] Lab setup instructions (Isaac Sim or Gazebo)
  - [ ] Capstone project rubric
  - [ ] Common student challenges & solutions
  - [ ] Time estimates per chapter and week

---

## Dependency Graph & Parallel Execution

### Critical Path (Must complete in order):

```
Phase 1: Setup
  ↓
Phase 2: Chapter 1 (Foundational)
  ↓
Phase 3: Chapter 2 (US1 Perception)
  ↓
Phase 5: Chapter 3 (US3 Language)
  ↓
Phase 6: Chapter 4 (US4 Integration)
  ↓
Phase 7: Week 13 (Capstone)
  ↓
Phase 8: Polish & Validation
```

### Parallelizable Tasks (after dependencies met):

**Within Chapter 2 (perception)**:
- [ ] T040 + T041 + T042 + T043: Different perception topics (run in parallel after T039)
- [ ] T046 + T047 + T048 + T049: Different fusion & affordance topics (parallel)
- [ ] T051 + T052 + T053 + T054: Different ROS 2 integration code examples (parallel)
- [ ] T056 + T057 + T058: Different labs (parallel after content complete)

**Within Chapter 3 (language)**:
- [ ] T074a + T074b + T074c: Different prompting strategies (parallel)
- [ ] T077 + T078 + T079: Mock + Real LLM implementations (parallel)
- [ ] T088 + T089 + T090 + T091: Different labs (parallel after content complete)

**Within Chapter 4 (control)**:
- [ ] T102 + T103 + T108: Different ROS 2 control examples (parallel)
- [ ] T112 + T113 + T114: Safety & deployment design sections (parallel)
- [ ] T115 + T116 + T117 + T118: Different labs (parallel after content complete)

**Cross-phase parallel** (after respective chapters complete):
- [ ] T148 (RAG metadata) can run in parallel with T145 (Week 13 validation)
- [ ] T149 + T150 (semantic chunking & references) can run in parallel

---

## Implementation Strategy

### MVP Scope (Minimum Viable Textbook):

Complete tasks **T001-T063** (Phases 1-3) to deliver:
- Setup & infrastructure
- Chapter 1: VLA fundamentals (complete)
- Chapter 2: Perception pipelines (complete)

**Delivery**: 3,000+ lines covering V + core P (vision + partial perception)

**Test**: Chapter 2 labs work; object detection >90% accurate

**Time estimate**: 60-80 hours

### Incremental Delivery (v2):

Add **T064-T096** (Phase 4-5):
- Chapter 3: Language & planning (complete)

**Delivery**: 5,000+ lines covering V + P + L (full perception + language)

**Test**: End-to-end Whisper → LLM → text plan working

**Time estimate**: 40-50 additional hours

### Full Delivery (v3):

Add **T097-T145** (Phase 6-8):
- Chapter 4: Control & integration (complete)
- Week 13: Capstone sprint guide (complete)
- Polish & validation (complete)

**Delivery**: 5,875+ lines covering V + L + A (complete VLA)

**Test**: End-to-end system works in simulation with all three pillars

**Time estimate**: 60-80 additional hours

### Total Effort:
- **MVP (Ch 1-2)**: 60-80 hours
- **v2 (Ch 1-3)**: 100-130 hours total
- **Full (Ch 1-4 + Week 13)**: 160-210 hours total

---

## Success Metrics & Acceptance Criteria

### Per User Story:

| Story | MVP? | Deliverable | Success Criteria |
|-------|------|-------------|-----------------|
| US1 (Speech-Controlled Pipeline) | No | Ch 2-3-4 + capstone | End-to-end Whisper→LLM→execute works |
| US2 (Vision Perception) | Yes | Ch 2 labs | >90% detection, affordances identified |
| US3 (LLM Decomposition) | No | Ch 3 labs | Convert 5+ tasks → valid ROS 2 plans |
| US4 (Integration) | No | Ch 4 labs | Full VLA pipeline executes successfully |
| US5 (Hardware Deploy) | No | Ch 4 bonus + Week 13 | Deployment design exercise complete |

### Per Chapter:

| Chapter | Lines | Labs | Examples | Exercises | Success |
|---------|-------|------|----------|-----------|---------|
| Ch 1 | 820 | 3+ | 3+ | 3+ | Build, all links work |
| Ch 2 | 880 | 3+ | 3+ | 3+ | Perception labs >90% accurate |
| Ch 3 | 920 | 4+ | 3+ | 3+ | LLM plans valid & executable |
| Ch 4 | 850 | 4+ | 3+ | 3+ | Full VLA integration works |
| Week 13 | 1,100 | 10+ | embedded | 50+ | Capstone sprint completes |

### Module-Wide:

- **Content**: 5,875+ lines, 25,000+ words ✓
- **Docusaurus build**: Zero errors, zero broken links ✓
- **RAG-ready**: Semantic chunking, metadata, discoverable ✓
- **Constitution compliance**: All 8 principles met ✓
- **Capstone ready**: Full VLA system architecture defined ✓

---

**Status**: ✅ Ready for implementation via `/sp.implement`
