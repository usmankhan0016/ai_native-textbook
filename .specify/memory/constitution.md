<!--
SYNC IMPACT REPORT
==================
Version Change: Template → 1.0.0
Rationale: Initial constitution ratification for AI-native Physical AI & Humanoid Robotics Textbook project

Modified Principles:
- Created comprehensive principles framework from template

Added Sections:
1. Core Principles (8 principles)
   - I. Technical Accuracy First
   - II. Pedagogical Excellence
   - III. Modular Architecture
   - IV. RAG-First Documentation
   - V. Progressive Complexity
   - VI. Industry Alignment
   - VII. Extensibility & Bonus Features
   - VIII. Deployment & Infrastructure
2. Technical Standards
3. Content Development Workflow
4. Governance

Removed Sections:
- All template placeholder sections replaced with concrete requirements

Templates Requiring Updates:
- ✅ plan-template.md: Constitution Check section can reference new principles
- ✅ spec-template.md: Aligned with modular user story approach
- ✅ tasks-template.md: Supports multi-module textbook structure
- ⚠ No command files found in repository yet (expected for new project)

Follow-up TODOs:
- None - all placeholders filled with concrete values
==================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy First

**Non-negotiable**: All textbook content MUST be technically accurate, verified, and industry-aligned.

- ROS 2 code examples MUST use correct rclpy APIs, URDF syntax, and launch file conventions
- Simulation tutorials MUST reflect actual Gazebo physics engines, SDF specifications, and Unity rendering pipelines
- NVIDIA Isaac content MUST align with official Isaac Sim and Isaac ROS documentation
- VLA system architectures MUST accurately represent Whisper, GPT, and multimodal integration patterns
- Hardware specifications MUST match real-world requirements (GPU capabilities, memory, sensors)
- No placeholder code or "this might work" explanations—verify before publishing

**Rationale**: Students will use this textbook to build real robotic systems. Inaccuracies lead to failed implementations, wasted time, and lost trust in the educational material.

### II. Pedagogical Excellence

**Non-negotiable**: Every chapter MUST follow proven pedagogical principles.

Required structure for all chapters:
- **Learning Objectives**: Clear, measurable outcomes (e.g., "After this chapter, you will be able to create a ROS 2 publisher node")
- **Key Concepts**: 3-5 core ideas introduced in the chapter
- **Conceptual Explanation**: Theory and background before implementation
- **Hands-On Tutorial**: Step-by-step guided practice with code
- **Architecture Diagrams**: Visual representations (ASCII art or markdown diagrams)
- **End-of-Chapter Exercises**: 3-5 progressive challenges
- **Capstone Integration**: How this chapter connects to the final humanoid project

**Rationale**: Adult learners in engineering require structured, progressive learning with clear objectives and practical application. Random explanations or code dumps fail to build competence.

### III. Modular Architecture

**Non-negotiable**: The textbook MUST be organized into independent, composable modules.

Module structure:
```
docs/
├── intro/                    # Course overview, prerequisites, setup
├── module-1/                 # ROS 2 — The Robotic Nervous System
│   ├── week-1-ros2-basics/
│   ├── week-2-nodes-topics/
│   └── week-3-services-actions/
├── module-2/                 # Digital Twin Simulation
│   ├── week-4-gazebo-intro/
│   ├── week-5-physics-sensors/
│   └── week-6-unity-rendering/
├── module-3/                 # NVIDIA Isaac Platform
│   ├── week-7-isaac-sim/
│   ├── week-8-synthetic-data/
│   └── week-9-isaac-ros/
├── module-4/                 # Vision-Language-Action Robotics
│   ├── week-10-vla-intro/
│   ├── week-11-whisper-gpt/
│   └── week-12-task-planning/
├── hardware/                 # Hardware tier specifications
├── capstone/                 # Final humanoid project documentation
├── glossary/                 # Technical terms and definitions
└── references/               # Citations and external resources
```

- Each module MUST be independently readable (with references to prerequisites)
- Each week MUST be completable in 8-10 study hours
- Modules MUST NOT have circular dependencies

**Rationale**: Modular structure enables students to navigate flexibly, instructors to customize course sequences, and contributors to update specific sections without breaking the entire textbook.

### IV. RAG-First Documentation

**Non-negotiable**: All textbook content MUST be structured for optimal RAG chatbot retrieval.

Requirements:
- **Semantic Chunking**: Each section (Learning Objectives, Key Concepts, Tutorials) MUST be a discrete semantic unit for embedding
- **Metadata Richness**: Every chapter MUST include structured metadata (module, week, difficulty, topics, prerequisites)
- **Citation Anchors**: Code examples and explanations MUST have unique IDs for precise source citation
- **Contextual Completeness**: Each section MUST be understandable with minimal surrounding context (for retrieval scenarios)
- **Hierarchical Structure**: Headers MUST follow consistent H1→H2→H3 hierarchy for context extraction

Indexing pipeline:
1. Markdown → Semantic chunks (by section)
2. Chunks → OpenAI embeddings → Qdrant Cloud vector store
3. Metadata → Neon Serverless Postgres (chapter info, topics, difficulty)
4. Chatbot queries → Hybrid search (vector similarity + metadata filters) → Context-aware answers

**Rationale**: The RAG chatbot is a core deliverable, not an afterthought. Structuring content for retrieval from the start prevents costly refactoring and ensures high-quality student Q&A experiences.

### V. Progressive Complexity

**Non-negotiable**: Content difficulty MUST increase gradually within and across modules.

Complexity tiers (clearly marked in metadata):
- **Beginner**: Basic concepts, guided tutorials, minimal prerequisites (Weeks 1-3)
- **Intermediate**: Multi-component integration, problem-solving exercises (Weeks 4-9)
- **Advanced**: System-level architecture, sim-to-real transfer, humanoid dynamics (Weeks 10-13)

Within each chapter:
1. Start with simplest possible example (e.g., "Hello World" ROS 2 node)
2. Introduce one new concept at a time
3. Build complexity through composition (combine previous concepts)
4. End with integration challenge (apply all chapter concepts)

**Rationale**: Cognitive load theory shows that learners can only handle limited new information at once. Jumping complexity levels causes frustration and dropouts.

### VI. Industry Alignment

**Non-negotiable**: All tools, frameworks, and patterns MUST reflect current industry practices (as of 2025).

Technology requirements:
- **ROS 2**: Use Humble Hawksbill or later (not ROS 1)
- **Python**: 3.10+ (for modern type hints and match statements)
- **NVIDIA Isaac**: Latest stable release APIs
- **Simulation**: Gazebo Garden or Harmonic, Unity 2022 LTS or later
- **LLMs**: OpenAI GPT-4 or equivalent (document model assumptions)
- **Deployment**: Docker for reproducible environments, GitHub Actions for CI/CD

Code standards:
- Follow ROS 2 best practices (package structure, naming conventions)
- Use type hints for all Python code
- Include linting configuration (ruff, black, mypy)
- Provide requirements.txt and Docker configurations

**Rationale**: Students enter the workforce expecting to use modern tools. Teaching outdated frameworks harms employability and wastes learning time.

### VII. Extensibility & Bonus Features

**Non-negotiable**: The textbook architecture MUST support optional bonus features without disrupting core content.

Bonus feature architecture:
- **Better-Auth Integration**: User signup/signin stored in Neon Postgres, seamlessly integrated with RAG chatbot user context
- **Personalization Engine**: Collect user background (software/hardware experience) → Render personalized chapter variants (e.g., "For experienced ROS 1 users: key differences in ROS 2")
- **Urdu Translation**: Button-triggered translation using Google Translate API or similar, cached in Postgres
- **Claude Code Subagents**: Reusable agents for textbook Q&A, code debugging, exercise grading
- **Agent Skills**: Skill modules for specific tasks (e.g., "ROS 2 troubleshooting", "Gazebo setup assistance")

Implementation requirements:
- Core textbook MUST work without any bonus features enabled
- Each bonus feature MUST be toggleable via feature flags
- Bonus features MUST NOT alter core content structure (only presentation layer)

**Rationale**: Bonus features add competition value (50-200 extra points) without compromising the primary educational mission. Clean separation enables incremental delivery.

### VIII. Deployment & Infrastructure

**Non-negotiable**: The textbook MUST be deployable to production with clear, documented infrastructure.

Deployment targets:
- **Primary**: GitHub Pages (free, versioned, automatic deployment from main branch)
- **Alternative**: Vercel (faster builds, preview deployments for PRs)

Infrastructure components:
- **Frontend**: Docusaurus static site (Node.js 18+, React 18)
- **RAG Backend**: FastAPI (Python 3.10+), deployed on Render Free Tier or Railway
- **Vector Store**: Qdrant Cloud Free Tier (1GB storage, sufficient for textbook embeddings)
- **Metadata Store**: Neon Serverless Postgres Free Tier (0.5GB, ~10 hours compute/month)
- **CI/CD**: GitHub Actions for automated testing, building, and deployment

Required documentation:
- `README.md`: Quick start, local development setup, deployment instructions
- `docs/infrastructure.md`: Architecture diagrams, data flow, API endpoints
- `docs/rag-setup.md`: Indexing pipeline, embedding strategy, query handling
- `Dockerfile`: Reproducible backend environment
- `.env.example`: Required environment variables

**Rationale**: A textbook that can't be deployed is a failed deliverable. Clear infrastructure documentation enables reproducibility, collaboration, and maintenance.

## Technical Standards

### Docusaurus Configuration

Required `docusaurus.config.js` settings:
- **Theme**: Classic theme with dark mode support
- **Search**: Algolia DocSearch or local search plugin
- **Plugins**:
  - `@docusaurus/plugin-content-docs` (core documentation)
  - Custom plugin for RAG chatbot integration (embedded iframe or React component)
  - Optional: `docusaurus-plugin-image-zoom` for diagram expansion
- **Navbar**: Module navigation, search, GitHub link, language toggle (if Urdu enabled)
- **Footer**: License (CC BY-NC-SA 4.0 recommended for educational content), contributors, feedback link

### RAG Chatbot API Contract

**Endpoints** (FastAPI backend):
```
POST /api/chat/query
  Body: { "question": str, "context": str | null, "user_id": str | null }
  Response: { "answer": str, "sources": [{ "chapter": str, "url": str, "snippet": str }], "confidence": float }

POST /api/chat/query-selected
  Body: { "question": str, "selected_text": str, "chapter_url": str }
  Response: { "answer": str, "sources": [{ "url": str, "snippet": str }] }

GET /api/health
  Response: { "status": "ok", "vector_db": "connected", "metadata_db": "connected" }
```

**Error Handling**:
- 400 Bad Request: Invalid query format
- 429 Too Many Requests: Rate limiting (10 queries/minute/user)
- 500 Internal Server Error: Database connection issues
- 503 Service Unavailable: OpenAI API down

### Code Block Standards

All code examples MUST follow these conventions:
- **Language Tag**: Specify language for syntax highlighting (```python, ```bash, ```xml)
- **File Path Comment**: First line indicates where code belongs (# File: src/my_node.py)
- **Inline Comments**: Explain non-obvious logic (not obvious syntax)
- **Runnable**: Code MUST be executable as-is (no ellipsis or "// rest of code")
- **Error Handling**: Production examples MUST include try/except or equivalent
- **Diff Markers**: When showing changes, use +/- prefixes or side-by-side blocks

Example:
```python
# File: src/simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create publisher on 'topic' with queue size 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Hardware Documentation Standards

For each hardware tier (Digital Twin Workstation, Edge AI Kit, Robot Lab A/B/C):
- **Component List**: Part names, model numbers, purchase links
- **Specifications**: Technical specs (GPU VRAM, CPU cores, RAM, storage)
- **Setup Instructions**: Step-by-step assembly and configuration
- **Software Requirements**: OS version, driver versions, ROS 2 installation
- **Validation Tests**: Scripts to verify hardware is correctly configured
- **Cost Breakdown**: Itemized pricing with totals (in USD, updated 2025)
- **Cloud Alternative**: Equivalent AWS instance type (e.g., g5.xlarge for GPU workloads)

## Content Development Workflow

### Specification → Plan → Tasks → Implement Cycle

For each module or chapter:

1. **Specification Phase** (`/sp.specify`):
   - Define learning objectives (measurable outcomes)
   - List key concepts to teach
   - Outline user stories (student journeys through the content)
   - Specify exercises and assessments
   - Mark prerequisites and co-requisites
   - Output: `specs/<module-name>/spec.md`

2. **Planning Phase** (`/sp.plan`):
   - Research existing documentation (ROS 2 docs, Isaac docs, Unity docs)
   - Design content architecture (section breakdown, diagram types)
   - Identify code examples to include
   - Plan RAG metadata structure for the module
   - Output: `specs/<module-name>/plan.md`, `specs/<module-name>/research.md`

3. **Task Breakdown** (`/sp.tasks`):
   - Generate checklist of writing tasks (intro section, tutorial section, exercises)
   - Assign priorities (P1: core content, P2: advanced topics, P3: bonus deep dives)
   - Mark parallelizable tasks (different chapters in same module)
   - Output: `specs/<module-name>/tasks.md`

4. **Implementation** (`/sp.implement`):
   - Write markdown content following pedagogical structure
   - Create code examples and test them in actual environments
   - Generate or source diagrams (ASCII art, mermaid, exported images)
   - Add RAG metadata annotations
   - Output: `docs/<module-name>/<chapter>.md`

5. **Review & Validation**:
   - Technical accuracy check (run all code examples)
   - Pedagogical review (learning objectives met?)
   - RAG indexing test (query chatbot for chapter topics)
   - Link verification (internal cross-references work)

### Prompt History Records (PHRs)

**Mandatory for all content development work**:
- After every `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` cycle
- Stored in `history/prompts/<module-name>/`
- Captures: user input (verbatim), agent response (summary), files created, tests run
- Enables: traceability, debugging, learning from past iterations

### Architecture Decision Records (ADRs)

**Required for significant decisions**:
- Module structure changes
- RAG architecture choices (embedding model, chunking strategy)
- Bonus feature implementation approaches (e.g., translation service selection)
- Infrastructure selections (Qdrant vs alternatives, Neon vs alternatives)

**Trigger**: When a decision meets all three criteria:
1. **Impact**: Long-term consequences on textbook structure or infrastructure
2. **Alternatives**: Multiple viable options considered with tradeoffs
3. **Scope**: Cross-cutting effect on multiple modules or systems

**Process**:
- Agent suggests: "📋 Architectural decision detected: [brief]. Document reasoning and tradeoffs? Run `/sp.adr [title]`"
- Wait for user consent
- Document in `history/adr/<sequential-number>-<decision-title>.md`

## Governance

### Amendment Process

Constitution changes require:
1. **Proposal**: Documented rationale for change (what, why, impact)
2. **Impact Analysis**: Which templates, modules, or infrastructure affected
3. **Version Bump**: Semantic versioning (MAJOR.MINOR.PATCH)
   - MAJOR: Backward-incompatible principle removals or redefinitions
   - MINOR: New principle/section added or materially expanded guidance
   - PATCH: Clarifications, wording, typo fixes
4. **Template Propagation**: Update plan-template.md, spec-template.md, tasks-template.md to align
5. **Sync Impact Report**: Prepended to constitution as HTML comment
6. **Commit Message**: `docs: amend constitution to vX.Y.Z (<brief summary>)`

### Compliance

**All development work MUST verify**:
- Technical accuracy (Principle I): Code examples tested in actual ROS 2/Gazebo/Isaac environments
- Pedagogical structure (Principle II): Learning Objectives and Key Concepts present in every chapter
- RAG compatibility (Principle IV): Metadata and chunking boundaries correct
- Progressive complexity (Principle V): Difficulty tier marked and appropriate
- Industry alignment (Principle VI): Tools and versions match 2025 standards

**Quality Gates**:
- Chapter cannot be merged without passing all code example tests
- Module cannot be published without RAG indexing verification
- Bonus features cannot be enabled without feature flag isolation verification

### Constitution Supremacy

In conflicts between:
- Constitution vs. individual chapter style → Constitution wins
- Constitution vs. external documentation → Constitution defines how to integrate external docs
- Constitution vs. time pressure → Constitution principles cannot be skipped (adjust scope instead)

**Use** `.specify/memory/constitution.md` (this file) as the authoritative source for all governance and development practices.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
