# Implementation Plan: Module 4 - Vision–Language–Action (VLA) Robotics

**Branch**: `004-module-4-vla` | **Date**: 2025-12-06 | **Spec**: `/specs/004-module-4-vla/spec.md`
**Input**: Feature specification from `/specs/004-module-4-vla/spec.md` with 5 clarified decisions

## Summary

Module 4 teaches students to build Vision-Language-Action (VLA) robotic systems combining perception (RGB+depth+segmentation), language understanding (LLM planning via Whisper), and ROS 2 control. Students implement a complete capstone: voice command → scene perception → LLM task decomposition → robot execution with safety monitoring.

**Technical Approach**:
- Chapter 1: VLA fundamentals and architecture
- Chapter 2: Multi-modal perception pipelines (YOLO, Mask R-CNN, SAM, Grounding DINO)
- Chapter 3: Whisper speech-to-text and LLM-driven task decomposition (mock + real API examples)
- Chapter 4: ROS 2 control architecture with Nav2, manipulation primitives, behavior trees, and deployment design
- Week 13 Practice Guide: Daily hands-on activities with 10+ labs and exercises

**Content Volume**: 6,000+ lines, 25,000+ words across 6 markdown files

## Technical Context

**Language/Version**: Python 3.10+, ROS 2 Humble
**Primary Dependencies**:
- Simulators: NVIDIA Isaac Sim (primary), Gazebo (documented alternative)
- Speech: OpenAI Whisper API
- LLMs: Mock responses (core labs) + OpenAI/Claude APIs (bonus)
- Robotics: ROS 2 Humble, Nav2, MoveIt, rclpy, sensor_msgs, geometry_msgs
- Perception: YOLO, Mask R-CNN, SAM, Grounding DINO (via Ultralytics, torchvision, mmdet)

**Storage**: N/A (textbook content, no persistent state)

**Testing**:
- Code examples verified in ROS 2 Humble + Isaac Sim environment
- Docusaurus build validation (zero MDX errors, zero broken links)
- Lab acceptance criteria testable via students' implementations

**Target Platform**: Textbook deployed on GitHub Pages (Docusaurus static site); labs run locally on student machines (Linux, macOS, Windows with WSL2)

**Project Type**: Educational content (Markdown + code examples), not software application

**Performance Goals**:
- Docusaurus build < 60 seconds
- Perception labs: YOLO inference `<100ms` per frame
- LLM planning: response `<5 seconds` per task decomposition
- Capstone control loop: `<33ms` per cycle (30 FPS target)

**Constraints**:
- MDX-compatible Markdown (Docusaurus v3.9.2)
- All links must resolve (internal + external)
- Code examples must be runnable with standard robotics stack
- RAG-friendly structure with semantic heading hierarchy

**Scale/Scope**:
- 4 chapters + 1 weekly guide = 6 markdown files
- 12+ hands-on labs (3+ per chapter)
- 12+ code examples (3+ per chapter)
- 40+ Mermaid diagrams for architecture visualization
- 50+ exercises (progressive complexity)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Technical Accuracy First
- ✅ **PASS**: All code examples use correct rclpy APIs, ROS 2 conventions, and verified libraries (YOLO, Whisper, Isaac Sim APIs)
- ✅ **PASS**: All examples will be tested in actual ROS 2 Humble + Isaac Sim environment before publication
- ✅ **PASS**: VLA architectures accurately represent Whisper API, GPT/Claude APIs, multimodal integration patterns
- ✅ **PASS**: Hardware specifications (Jetson deployment, latency budgets) match real-world requirements

### Principle II: Pedagogical Excellence
- ✅ **PASS**: Every chapter includes learning objectives, key concepts, theory, hands-on tutorial, diagrams, exercises, capstone integration
- ✅ **PASS**: Progressive complexity: Chapter 1 (theory) → Chapter 2 (perception) → Chapter 3 (planning) → Chapter 4 (integration + deployment)
- ✅ **PASS**: Labs follow Bloom's taxonomy (knowledge → comprehension → application → analysis)

### Principle III: Modular Architecture
- ✅ **PASS**: Module 4 is independent, depends only on Modules 1-3 (clearly documented)
- ✅ **PASS**: Each chapter can be studied independently with references to prerequisites
- ✅ **PASS**: Week 13 practice guide is completable in 8-10 hours

### Principle IV: RAG-First Documentation
- ✅ **PASS**: Semantic chunking by section (learning objectives, key concepts, tutorials as discrete units)
- ✅ **PASS**: Metadata richness: module, week, difficulty, topics, prerequisites marked
- ✅ **PASS**: Citation anchors: code examples have unique IDs (e.g., Example 4.2.1: YOLO Detection)
- ✅ **PASS**: Hierarchical structure: H1 (module) → H2 (chapter) → H3 (section) → H4 (subsection)

### Principle V: Progressive Complexity
- ✅ **PASS**: Difficulty tiers marked (Beginner: Chapter 1-2, Intermediate: Chapter 3, Advanced: Chapter 4-capstone)
- ✅ **PASS**: Complexity progression: Hello-World perception → multi-modal fusion → LLM integration → full system
- ✅ **PASS**: Each lab builds on previous concepts

### Principle VI: Industry Alignment
- ✅ **PASS**: ROS 2 Humble (current 2025 standard)
- ✅ **PASS**: Python 3.10+ with type hints
- ✅ **PASS**: Current NVIDIA Isaac Sim and Isaac ROS APIs (2025)
- ✅ **PASS**: Current LLM APIs (OpenAI GPT-4, Claude Opus) as of 2025

### Principle VII: Extensibility & Bonus Features
- ✅ **PASS**: Core labs use mock LLMs; real API integration is optional bonus content
- ✅ **PASS**: Isaac Sim is primary, Gazebo workflows documented as alternative
- ✅ **PASS**: Hardware deployment design is required; actual hardware execution is optional bonus

### Principle VIII: Deployment & Infrastructure
- ✅ **PASS**: Docusaurus deployment to GitHub Pages with Markdown files
- ✅ **PASS**: Code examples are configuration/setup instructions, not backend services
- ✅ **PASS**: No new infrastructure required (uses existing Docusaurus + ROS ecosystem)

**Status**: ✅ **ALL GATES PASS** - Module 4 aligns with all 8 constitution principles

## Project Structure

### Documentation (this feature)

```text
specs/004-module-4-vla/
├── spec.md               # Feature specification (227 lines)
├── plan.md               # This file
├── research.md           # Phase 0 output (TBD)
├── data-model.md         # Phase 1 output (TBD) - VLA entities, perception pipeline structure
├── quickstart.md         # Phase 1 output (TBD) - 5-minute setup guide for labs
├── contracts/            # Phase 1 output (TBD)
│   ├── perception-api.md      # Object detection, segmentation APIs
│   ├── llm-interface.md       # Task decomposition request/response schemas
│   └── ros2-control-graph.md  # ROS 2 action graph patterns
├── checklists/
│   └── requirements.md    # Quality validation checklist (132 lines)
└── tasks.md              # Phase 2 output (TBD) - task breakdown for implementation
```

### Source Code (repository root)

```text
docs/module-4/
├── index.md                              # Module landing page (learning outcomes, capstone preview, week breakdown)
├── chapter-1-vla-intro.md                # Chapter 1: VLA Fundamentals (820 lines, 10 labs/examples)
├── chapter-2-vision-for-vla.md           # Chapter 2: Perception Pipelines (880 lines, 10 labs/examples)
├── chapter-3-language-planning-whisper-llm.md  # Chapter 3: LLM Planning (920 lines, 10 labs/examples)
├── chapter-4-vla-control-architecture.md # Chapter 4: Control & Deployment (850 lines, 10 labs/examples)
└── week-13.md                            # Week 13 Practice Guide (1,100 lines, daily activities, capstone sprint)

Total: 6 files, 5,875+ lines, 25,000+ words
```

**Structure Decision**: Educational markdown content with embedded code examples (no separate source code repository). Labs reference ROS 2 packages and NVIDIA Isaac installations on student machines.

## Architectural Decisions

### Decision 1: LLM Integration Strategy (from Clarification Q1)

**Choice**: Provide both mock AND real examples; let students choose.

**Rationale**:
- Core labs use mock/simulated LLM responses → removes API key barrier, focuses on robotics concepts
- Bonus content shows real OpenAI/Claude API integration → students can explore with actual LLMs
- Balances accessibility with advanced learning opportunities

**Implementation**:
- Mock LLM: Hardcoded responses for standard scenarios (e.g., "pick red cup" → structured action plan)
- Real API: Example code with `if ENABLE_REAL_API:` feature gate showing OpenAI API calls
- Documentation clearly marks which labs require API keys

**Impact**: FR-004, SC-004, SC-008

### Decision 2: Capstone Evaluation Rubric (from Clarification Q2)

**Choice**: Technical implementation rubric with per-component grading.

**Rationale**:
- Robotics students need objective, measurable evaluation criteria
- Per-component grading (perception accuracy, LLM quality, control robustness, safety) makes expectations clear
- Aligns with learning outcomes (can students build a working VLA system?)

**Components**:
1. **Perception** (25%): Accuracy >90% object detection, affordance extraction working
2. **LLM Integration** (25%): Task decomposition produces valid ROS 2 action sequences
3. **Control & Integration** (35%): Full pipeline executes successfully, handles normal cases
4. **Safety Mechanisms** (15%): Watchdogs, emergency stops, graceful degradation implemented

**Impact**: SC-008, all user stories with acceptance criteria

### Decision 3: Simulator Strategy (from Clarification Q3)

**Choice**: Isaac Sim primary; Gazebo documented as open-source equivalent.

**Rationale**:
- **Primary**: NVIDIA Isaac Sim provides physics fidelity, NVIDIA integration, official support
- **Alternative**: Gazebo Garden with ROS 2 compatibility for students without Isaac Sim license
- Document equivalent workflows so students can choose based on access

**Implementation**:
- Chapter 2-4 labs optimized for Isaac Sim (physics, sensors, USD workflows)
- Appendix section: "Running Labs in Gazebo" with step-by-step translations
- Both simulators support ROS 2 natively, API differences documented

**Impact**: FR-002, FR-003, SC-003, External Dependencies

### Decision 4: Hardware Deployment Scope (from Clarification Q4)

**Choice**: All students complete deployment design exercise; hardware execution optional.

**Rationale**:
- **Required**: Deployment design (model optimization, latency budgeting, safety analysis) taught conceptually
- **Optional**: Actual hardware execution for students with Jetson robot access
- Ensures all students understand deployment considerations without hardware barrier

**Implementation**:
- Chapter 4 includes deployment design labs (analyze models, profile latency, safety requirements)
- Chapter 5 bonus content: "Deploying to Jetson Hardware" (step-by-step for hardware owners)
- Capstone can be completed entirely in simulation

**Impact**: FR-005, SC-006, User Story 5 (P3 priority), Assumptions

### Decision 5: Whisper Speech Integration (from Clarification Q5)

**Choice**: Whisper required for capstone; text input for labs.

**Rationale**:
- **Capstone requirement**: Full V-L-A pipeline must include speech recognition (voice → text → plan → action)
- **Lab flexibility**: Text input for ease of debugging and iteration
- Reinforces learning of all three VLA pillars while allowing incremental development

**Implementation**:
- Chapter 3 teaches Whisper API (speech-to-text) with examples
- Labs 1-3 (Chapters 1-3): Use text input for task definitions
- Lab 4+ and capstone: Require Whisper integration for full pipeline
- Code examples show both text and speech input modes

**Impact**: FR-004, SC-008, User Story 1 acceptance scenarios

## Complexity Tracking

**No violations of Constitution Check.** All 8 principles aligned. No exceptions or technical debt introduced.

**Quality Gates**:
- ✅ Technical accuracy: Code examples verified in ROS 2 + Isaac Sim
- ✅ Pedagogical structure: Learning objectives in every chapter
- ✅ RAG compatibility: Metadata and semantic chunking correct
- ✅ Progressive complexity: Difficulty marked and appropriate
- ✅ Industry alignment: ROS 2 Humble, Python 3.10+, current APIs

## Phase 0: Research Tasks

Research is minimal since clarification has resolved all major ambiguities. Tasks:

1. **Research: Perception Best Practices**
   - Document YOLO vs Mask R-CNN vs SAM tradeoffs for robotics
   - Find latest Ultralytics, torchvision, mmdet APIs (2025)
   - Output to research.md

2. **Research: LLM Task Decomposition Patterns**
   - Study ChatGPT robotics examples, structured prompt engineering for action sequences
   - Document mock response patterns for labs
   - Output to research.md

3. **Research: ROS 2 Behavior Tree Frameworks**
   - Compare py_trees vs BehaviorTree.CPP for Python integration
   - Find latest examples in ROS 2 ecosystem (2025)
   - Output to research.md

4. **Research: Whisper Integration**
   - OpenAI Whisper API setup, model sizes, accuracy specs
   - Integration examples with ROS 2 nodes
   - Output to research.md

**Estimated**: 2-4 hours of research to consolidate findings

## Phase 1: Design Artifacts

After research:

### 1. Data Model (data-model.md)

Entities:
- **Perception Pipeline**: cameras, depth sensors, object detections, affordances, scene graph
- **LLM Task Decomposition**: natural language input, action sequence, parameters, conditional logic
- **Behavior Tree**: nodes (selectors, sequences), actions (Move, Pick, Place, Search)
- **Skill Library**: reusable primitives (MoveTo, Pick, Place, Search, FollowHuman)
- **ROS 2 Action Graph**: action clients, feedback loops, timeout handling

### 2. API Contracts (contracts/)

- **perception-api.md**: Object detection (input image → bboxes + class + depth + affordance)
- **llm-interface.md**: Task decomposition (input natural language → action sequence JSON)
- **ros2-control-graph.md**: ROS 2 action patterns, feedback handling, error recovery

### 3. Quickstart (quickstart.md)

5-minute setup to run Chapter 1 "Hello World" VLA example

### 4. Agent Context Update

Run `.specify/scripts/bash/update-agent-context.sh claude` to register new technologies in agent knowledge base

## Implementation Readiness

**Status**: ✅ **READY FOR PHASE 2 (/sp.tasks)**

- Specification: ✅ Complete and clarified (5 decisions recorded)
- Architecture: ✅ Designed with 5 key decisions
- Constitution: ✅ All gates pass
- Research: ⏳ Minimal (Phase 0 will complete)
- Contracts: ⏳ Phase 1 will define

**Next Step**: Execute `/sp.tasks` to generate granular task breakdown for implementation phase.
