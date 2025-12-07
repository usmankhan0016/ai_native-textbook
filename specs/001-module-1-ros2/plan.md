# Implementation Plan: Module 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-module-1-ros2` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-module-1-ros2/spec.md`

## Summary

Create comprehensive educational content for Module 1 of the Physical AI & Humanoid Robotics textbook, covering ROS 2 fundamentals (Weeks 3-5). This module introduces students to ROS 2 architecture, communication patterns (nodes, topics, services, actions), package management, launch files, and URDF robot descriptions for humanoid systems. Content will be delivered as 8 Docusaurus-compatible markdown files with embedded code examples, architecture diagrams, learning objectives, and hands-on exercises. All content structured for RAG chatbot indexing with semantic chunking and metadata.

**Primary Requirements**:
- 4 core chapters: Introduction to ROS 2, Architecture, Packages/Launch, URDF for Humanoids
- 3 weekly breakdown files: Week 3 (concepts), Week 4 (packages), Week 5 (URDF)
- 1 module index file with navigation and overview
- All content follows Constitution Principle II (Pedagogical Excellence) structure
- Technical accuracy per Constitution Principle I (verified against ROS 2 Humble docs)
- RAG-first documentation per Constitution Principle IV (semantic chunking, metadata)

**Technical Approach**:
- Docusaurus v3+ markdown format with frontmatter metadata
- Python 3.10+ code examples using rclpy (ROS 2 Humble)
- ASCII art and mermaid diagrams for architecture visualization
- Admonitions (:::tip, :::warning, :::note) for pedagogical callouts
- Code blocks with syntax highlighting and file path comments
- Cross-linking between chapters and weekly guides

## Technical Context

**Language/Version**: Markdown (Docusaurus v3+), Python 3.10+ (code examples only - not implementation)
**Primary Dependencies**: Docusaurus v3+ (target platform), ROS 2 Humble Hawksbill (content subject)
**Storage**: Git repository with markdown files in `docs/module-1/` directory
**Testing**: Content validation (markdown syntax, link checking), Code example verification (manual execution on Ubuntu 22.04 + ROS 2 Humble)
**Target Platform**: Docusaurus static site generator, deployed to GitHub Pages or Vercel
**Project Type**: Documentation content (markdown files for static site)
**Performance Goals**: RAG chatbot retrieval accuracy 95%+, Page load time <2 seconds, Search indexing complete coverage
**Constraints**: 2500-4000 words per chapter (excluding code), Code-to-explanation ratio 1:3, Beginner difficulty level, 8-10 hours study time per week
**Scale/Scope**: 8 markdown files, ~20,000-30,000 total words, 15-20 code examples, 8-12 architecture diagrams, 20-30 exercises

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Technical Accuracy First**
- ✅ **Pass**: All ROS 2 code examples will be verified against official ROS 2 Humble documentation
- ✅ **Pass**: URDF syntax will be validated using `check_urdf` tool (per SC-015)
- ✅ **Pass**: QoS configurations, message types, and launch file patterns will reference official docs
- ✅ **Pass**: No placeholder code - all examples must be runnable (per FR-033)

**Principle II: Pedagogical Excellence**
- ✅ **Pass**: All chapters include required structure: Learning Objectives, Key Concepts, Conceptual Explanation, Hands-On Tutorial, Architecture Diagrams, End-of-Chapter Exercises, Capstone Integration (per FR-032)
- ✅ **Pass**: Each chapter has 3-5 learning objectives, 4-6 key concepts, 2-3 diagrams, 4-6 exercises (per SC-012)
- ✅ **Pass**: Progressive complexity: Chapter 1 (conceptual) → Chapter 2 (hands-on) → Chapter 3 (organization) → Chapter 4 (modeling)

**Principle III: Modular Architecture**
- ✅ **Pass**: Module 1 is self-contained with explicit prerequisites (course intro, Ubuntu, Python basics per Assumptions #1-2)
- ✅ **Pass**: Each chapter independently readable with cross-references
- ✅ **Pass**: No circular dependencies between chapters (linear progression)
- ✅ **Pass**: Clear connection to Module 2 (Digital Twin Simulation) via URDF and Gazebo integration

**Principle IV: RAG-First Documentation**
- ✅ **Pass**: Frontmatter metadata on all files: difficulty (Beginner), topics, prerequisites (per FR-034)
- ✅ **Pass**: Semantic chunking via consistent H1→H2→H3 hierarchy
- ✅ **Pass**: Each section contextually complete (can be retrieved independently)
- ✅ **Pass**: Citation anchors via heading IDs for precise source references

**Principle V: Progressive Complexity**
- ✅ **Pass**: All content marked Beginner difficulty (Weeks 3-5 of course)
- ✅ **Pass**: Gradual concept introduction: ROS 2 basics → communication → organization → robot modeling
- ✅ **Pass**: Code examples start simple (minimal node) and build complexity (services, actions, URDF)

**Principle VI: Industry Alignment**
- ✅ **Pass**: ROS 2 Humble Hawksbill (latest LTS as of 2025)
- ✅ **Pass**: Python 3.10+ with type hints (per constitution)
- ✅ **Pass**: Modern tooling: colcon (build), ros2 CLI, rviz2, rqt
- ✅ **Pass**: Integration with Gazebo Garden/Harmonic, Unity 2022 LTS, Isaac Sim

**Principle VII: Extensibility & Bonus Features**
- ✅ **Pass**: Frontmatter structure supports personalization (difficulty, topics, user background filtering)
- ✅ **Pass**: Markdown content compatible with translation (Urdu translation button feature)
- ✅ **Pass**: Modular structure allows RAG chatbot integration without content changes

**Principle VIII: Deployment & Infrastructure**
- ✅ **Pass**: Docusaurus v3+ compatible markdown
- ✅ **Pass**: GitHub Pages / Vercel deployment ready
- ✅ **Pass**: Static site generation (no server-side dependencies for content)

**Constitution Check Result**: ✅ **ALL GATES PASS** - No violations, ready for Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/001-module-1-ros2/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Docusaurus best practices, ROS 2 Humble APIs
├── data-model.md        # Phase 1 output: Content entity models (Chapter, CodeExample, Exercise, Diagram)
├── quickstart.md        # Phase 1 output: Content creation guide for implementers
├── contracts/           # Phase 1 output: File specifications and content contracts
│   ├── index-md.md      # Module index file specification
│   ├── chapter-1.md     # Chapter 1 content contract
│   ├── chapter-2.md     # Chapter 2 content contract
│   ├── chapter-3.md     # Chapter 3 content contract
│   ├── chapter-4.md     # Chapter 4 content contract
│   ├── week-3.md        # Week 3 guide specification
│   ├── week-4.md        # Week 4 guide specification
│   └── week-5.md        # Week 5 guide specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/module-1/
├── index.md                          # Module 1 introduction and navigation
├── chapter-1-introduction-to-ros2.md # Chapter 1: What is ROS 2? Why for humanoids?
├── chapter-2-ros2-architecture.md    # Chapter 2: Nodes, topics, services, actions, QoS
├── chapter-3-ros2-packages-launch.md # Chapter 3: Packages, colcon, launch files, parameters
├── chapter-4-urdf-for-humanoids.md   # Chapter 4: URDF, xacro, RViz, Gazebo integration
├── week-3.md                         # Week 3: ROS 2 concepts, architecture, CLI tools
├── week-4.md                         # Week 4: Packages, launch, parameters, multi-node
└── week-5.md                         # Week 5: URDF, xacro, joints, Gazebo loading

# No code subdirectories - all examples inline in markdown
# Diagrams embedded as ASCII art or mermaid code blocks
```

**Structure Decision**: Documentation-only project with markdown content files. All code examples embedded inline within markdown files (no separate .py files). Architecture diagrams rendered as ASCII art or mermaid diagrams within markdown. This aligns with Docusaurus best practices for educational content and simplifies RAG indexing (single file = single semantic unit).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations** - Constitution Check passed all gates. No complexity justification needed.

---

## Phase 0: Research & Technology Selection

**Goal**: Resolve all technical unknowns and establish authoritative sources for content creation.

### Research Areas

1. **Docusaurus v3+ Best Practices**
   - **Decision**: Use frontmatter metadata with `sidebar_position`, `sidebar_label`, `tags`, custom fields for RAG
   - **Rationale**: Docusaurus v3+ supports autogenerated sidebars with frontmatter control; custom fields enable RAG metadata without breaking Docusaurus rendering
   - **Alternatives Considered**: Manual sidebar.js configuration (rejected - less maintainable for 8 files)
   - **Sources**: `/websites/docusaurus_io` documentation (Context7 MCP), official Docusaurus docs

2. **ROS 2 Humble API References**
   - **Decision**: Use official ROS 2 Humble documentation as authoritative source for all code examples
   - **Rationale**: ROS 2 Humble is LTS (Long Term Support) release, widely adopted, stable APIs through 2027
   - **Alternatives Considered**: Iron (newer but not LTS), Rolling (too unstable for educational content)
   - **Sources**: docs.ros.org/en/humble/, rclpy API docs, geometry_msgs, sensor_msgs package docs

3. **Code Example Patterns for rclpy**
   - **Decision**: Follow official ROS 2 tutorials pattern: minimal working example → explanation → extension
   - **Rationale**: Students learn best with runnable code they can modify incrementally
   - **Alternatives Considered**: Conceptual-only (rejected - not hands-on), Full application examples (rejected - too complex for Module 1)
   - **Sources**: ROS 2 Humble tutorials (docs.ros.org), ros2/examples repository

4. **URDF Best Practices for Humanoids**
   - **Decision**: Start with minimal biped (torso, 2 legs, 2 feet) then extend with arms, head
   - **Rationale**: Humanoid structure is complex; minimal example teaches fundamentals without overwhelming
   - **Alternatives Considered**: Full humanoid from start (rejected - too much to explain), Abstract robot (rejected - not humanoid-specific)
   - **Sources**: URDF specification (wiki.ros.org/urdf), Unitree G1 URDF (reference only), PAL Robotics examples

5. **Pedagogical Structure for Technical Content**
   - **Decision**: Use Bloom's Taxonomy progression: Remember (concepts) → Understand (architecture) → Apply (hands-on) → Analyze (exercises)
   - **Rationale**: Adult learning theory supports progressive skill building with immediate application
   - **Alternatives Considered**: Theory-first (rejected - low engagement), Project-first (rejected - lacks foundation)
   - **Sources**: Constitution Principle II, educational best practices for engineering students

6. **Diagram Format Selection**
   - **Decision**: ASCII art for simple node graphs, mermaid for complex architecture, external images for robot structures
   - **Rationale**: ASCII art renders in all markdown viewers, mermaid supports by Docusaurus, images for high-detail visuals
   - **Alternatives Considered**: PlantUML (rejected - requires external rendering), Draw.io (rejected - not version-controllable)
   - **Sources**: Docusaurus markdown features docs, ROS 2 documentation diagram patterns

7. **Admonition Usage Patterns**
   - **Decision**: `:::tip` for best practices, `:::warning` for common pitfalls, `:::note` for supplementary info, `:::danger` for critical errors
   - **Rationale**: Docusaurus admonitions provide visual hierarchy and improve scannability
   - **Alternatives Considered**: Plain markdown (rejected - less visual), Custom components (rejected - over-engineering)
   - **Sources**: Docusaurus admonitions documentation (Context7 MCP examples)

8. **Cross-Linking Strategy**
   - **Decision**: Use relative markdown links `[text](./chapter-2.md#section)` with explicit section anchors
   - **Rationale**: Portable across environments, works in GitHub and Docusaurus, enables deep linking
   - **Alternatives Considered**: Docusaurus-specific links (rejected - less portable), No deep links (rejected - poor UX)
   - **Sources**: Docusaurus linking documentation, markdown best practices

### Research Outputs

See [research.md](./research.md) for detailed findings, API references, code patterns, and authoritative source URLs.

---

## Phase 1: Content Design & Specifications

**Prerequisites**: research.md complete

### Data Model

See [data-model.md](./data-model.md) for complete entity definitions. Summary:

**Entities**:
1. **Chapter**: Self-contained learning unit (title, number, difficulty, topics, prerequisites, word_count, learning_objectives[], key_concepts[], sections[], exercises[], code_examples[], diagrams[])
2. **CodeExample**: Runnable code snippet (language, file_path_comment, purpose, code, expected_output, explanation)
3. **Exercise**: End-of-chapter challenge (title, difficulty, estimated_time_minutes, description, acceptance_criteria[], solution_hints[])
4. **Diagram**: Visual architecture representation (type: ascii|mermaid|image, caption, content, referenced_section)
5. **WeeklyGuide**: Supplementary practice document (week_number, chapter_references[], cli_tools[], mini_labs[], assignments[])

**Relationships**:
- Chapter has_many CodeExamples, Exercises, Diagrams
- WeeklyGuide references_many Chapters
- Exercise references CodeExample (for solution hints)

**Validation Rules**:
- Chapter word count: 2500-4000 (excluding code blocks)
- Learning objectives: 3-5 per chapter
- Key concepts: 4-6 per chapter
- Diagrams: 2-3 per chapter
- Exercises: 4-6 per chapter
- Code-to-explanation ratio: 1:3 (one code block per three paragraphs)

### Content Specifications (Contracts)

See `contracts/` directory for detailed file-by-file specifications. Each contract defines:
- Frontmatter metadata (title, sidebar_position, tags, custom RAG fields)
- Required sections with heading structure
- Code example requirements (quantity, complexity, topics)
- Diagram specifications (type, purpose, content guidelines)
- Exercise specifications (difficulty, estimated time, acceptance criteria)
- Cross-links to other chapters/guides
- Word count targets

**Contract Files**:
- `contracts/index-md.md`: Module 1 index file specification
- `contracts/chapter-1.md`: Chapter 1 (Introduction to ROS 2) specification
- `contracts/chapter-2.md`: Chapter 2 (ROS 2 Architecture) specification
- `contracts/chapter-3.md`: Chapter 3 (Packages, Launch, Parameters) specification
- `contracts/chapter-4.md`: Chapter 4 (URDF for Humanoids) specification
- `contracts/week-3.md`: Week 3 guide specification
- `contracts/week-4.md`: Week 4 guide specification
- `contracts/week-5.md`: Week 5 guide specification

### Quickstart Guide

See [quickstart.md](./quickstart.md) for implementer guide covering:
- How to create a new chapter following the template
- Frontmatter metadata requirements
- Code example formatting standards
- Diagram creation workflows
- Exercise design guidelines
- Link checking and validation
- RAG metadata best practices

---

## Phase 2: Task Generation

**Output**: tasks.md (created by `/sp.tasks` command, NOT by `/sp.plan`)

The tasks file will break down content creation into:
- Phase 1: Setup (create directory structure, initialize frontmatter templates)
- Phase 2: Foundational (Module index, Chapter 1 conceptual content)
- Phase 3: Chapter 2 (Hands-on code examples - nodes, topics, services, actions)
- Phase 4: Chapter 3 (Package management, launch files, parameters)
- Phase 5: Chapter 4 (URDF modeling, xacro, simulator integration)
- Phase 6: Weekly guides (Week 3, 4, 5 reinforcement content)
- Phase 7: Polish (link checking, diagram refinement, exercise validation, RAG metadata verification)

**Parallelization Opportunities**:
- All chapters can be written in parallel after foundational phase
- Weekly guides can be written in parallel after corresponding chapters complete
- Diagrams can be created in parallel with text content
- Code examples can be written and tested in parallel

---

## Implementation Notes from Context7 MCP

**Docusaurus Frontmatter Best Practices** (from `/websites/docusaurus_io`):

1. **Sidebar Control**:
   ```yaml
   ---
   sidebar_position: 1
   sidebar_label: "Chapter 1: Introduction"
   sidebar_class_name: "chapter-intro"
   ---
   ```
   - Use `sidebar_position` for ordering (1-8 for Module 1 files)
   - Use `sidebar_label` for custom nav text (shorter than title)
   - Use `sidebar_class_name` for custom CSS styling

2. **RAG Metadata** (custom fields):
   ```yaml
   ---
   title: "Chapter 1: Introduction to ROS 2"
   tags: [ROS2, Fundamentals, Beginner, Week3]
   difficulty: Beginner
   module: 1
   week: 3
   prerequisites: ["Ubuntu 22.04", "Python basics", "Command line"]
   estimated_time: "2-3 hours"
   topics: ["ROS 2 architecture", "DDS middleware", "ROS 1 vs ROS 2"]
   ---
   ```
   - Custom fields pass through Docusaurus unchanged
   - RAG indexer can extract these for metadata filtering
   - Use arrays for multi-value fields (tags, prerequisites, topics)

3. **Admonition Syntax** (from Docusaurus examples):
   ```markdown
   :::tip[Best Practice]

   Always use type hints in your ROS 2 Python nodes for better code clarity.

   :::

   :::warning[Common Pitfall]

   QoS settings must match between publisher and subscriber, or messages may be dropped.

   :::

   :::note[For Advanced Students]

   ROS 2 Humble uses DDS (Data Distribution Service) as the middleware layer.

   :::
   ```
   - Empty lines around admonition directives prevent Prettier formatting issues
   - Use optional title in brackets for context

4. **Code Block Formatting**:
   ```markdown
   ```python showLineNumbers title="minimal_publisher.py"
   # File: src/minimal_publisher.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
   ```
   ```
   - Use `showLineNumbers` for reference in explanations
   - Use `title` for file path context
   - Always include `# File:` comment as first line

5. **Cross-Linking**:
   ```markdown
   See [Chapter 2: ROS 2 Architecture](./chapter-2-ros2-architecture.md#qos-profiles) for details on QoS.

   Prerequisites: [Course Introduction](../intro/index.md)
   ```
   - Use relative paths for portability
   - Include section anchors for deep linking
   - Link to prerequisites at chapter start

6. **Diagram Integration**:
   ```markdown
   ```mermaid
   graph LR
       A[Publisher Node] -->|Topic| B[Subscriber Node]
       C[Service Client] -->|Request| D[Service Server]
       D -->|Response| C
   ```
   ```
   - Mermaid renders automatically in Docusaurus
   - Keep diagrams simple (max 10 nodes)
   - Add caption below diagram as plain text

---

## Cross-Linking Map

**Chapter Dependencies**:
- `index.md` → links to all 4 chapters + 3 weekly guides
- `chapter-1` → no dependencies (prerequisite: course intro only)
- `chapter-2` → references `chapter-1` concepts (ROS 2 graph, nodes)
- `chapter-3` → references `chapter-2` code examples (nodes to package)
- `chapter-4` → references `chapter-2` (URDF loaded by nodes), `chapter-3` (URDF in packages)
- `week-3.md` → references `chapter-1`, `chapter-2`
- `week-4.md` → references `chapter-3`
- `week-5.md` → references `chapter-4`

**External Links** (to be included in content):
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- rclpy API: https://docs.ros.org/en/humble/p/rclpy/
- URDF Specification: http://wiki.ros.org/urdf/XML
- Gazebo Documentation: https://gazebosim.org/docs
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub

**Module 2 Forward References** (for capstone integration sections):
- Gazebo simulation (Chapter 4 URDF loading)
- Digital Twin concepts (Chapter 1 overview)
- Unity rendering (Chapter 4 URDF export)

---

## Prerequisites for Students

**Before Starting Module 1, students must have**:
1. **Development Environment**: Ubuntu 22.04 (native or WSL2) with ROS 2 Humble installed
2. **Programming Knowledge**: Python basics (functions, classes, imports)
3. **Command Line**: Linux terminal navigation, running commands
4. **Tools Installed**:
   - ROS 2 Humble Hawksbill
   - Python 3.10+
   - Text editor (VS Code recommended)
   - ros2 CLI tools
   - rviz2
   - colcon build tool
5. **Course Introduction Complete**: Understanding of Physical AI goals, humanoid robotics overview

**Installation Resources** (referenced in Module 1 index):
- ROS 2 Humble installation guide (link to official docs)
- Ubuntu 22.04 setup (link to course intro)
- Python environment configuration

---

## Success Metrics (from Specification)

**Content Quality**:
- ✅ Each chapter 2500-4000 words (SC-011)
- ✅ Each chapter has 3-5 objectives, 4-6 concepts, 2-3 diagrams, 4-6 exercises (SC-012)
- ✅ Code-to-explanation ratio 1:3 (SC-013)
- ✅ Technical accuracy verified against ROS 2 Humble docs (SC-014)
- ✅ URDF examples validate with `check_urdf` tool (SC-015)

**Student Outcomes**:
- ✅ Chapter 1 readable in 2-3 hours with comprehension (SC-001)
- ✅ Students write talker/listener in 1 hour post-Chapter 2 (SC-002)
- ✅ Students create package + launch file in 1.5 hours post-Chapter 3 (SC-003)
- ✅ Students write URDF and visualize in 2 hours post-Chapter 4 (SC-004)
- ✅ 90% exercise completion rate on first attempt (SC-005)
- ✅ Students troubleshoot ROS 2 issues with CLI tools after Week 3 (SC-006)

**Technical Validation**:
- ✅ RAG chatbot 95%+ retrieval accuracy (SC-007)
- ✅ All code executes on Ubuntu 22.04 + ROS 2 Humble (SC-008)
- ✅ Students build multi-node system by module end (SC-009)
- ✅ Clear prerequisite mapping to Module 2 (SC-010)

---

## Next Steps

1. ✅ **Complete Phase 0**: Generate `research.md` with detailed ROS 2 API references and Docusaurus patterns
2. ✅ **Complete Phase 1**: Generate `data-model.md`, `quickstart.md`, and all contract files in `contracts/`
3. ⏳ **Run `/sp.tasks`**: Generate detailed task breakdown for content creation
4. ⏳ **Run `/sp.implement`**: Execute tasks to create all 8 markdown files
5. ⏳ **Validation**: Test code examples, check links, verify RAG metadata, validate URDF

**Current Status**: Phase 0 and Phase 1 planning complete. Ready for task generation.

**Branch**: `001-module-1-ros2`
**Plan File**: `/mnt/f/ai_native-textbook/specs/001-module-1-ros2/plan.md`
**Generated Artifacts**: This file (plan.md), next: research.md, data-model.md, contracts/, quickstart.md
