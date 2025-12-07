---
id: 0001
title: Create AI-native textbook constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "initialization", "hackathon", "physical-ai", "robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

You are creating a complete AI-native textbook project for the Hackathon I: Physical AI & Humanoid Robotics challenge.
The goal is to generate:
1. A full Docusaurus-based textbook.
2. An integrated RAG chatbot.
3. Optional bonus features: agent skills, subagents, personalization, Urdu translation, and Better-Auth integration.

============================================================
PROJECT PURPOSE
============================================================
Create a highly technical, accurate, and pedagogically strong textbook that teaches the full Physical AI & Humanoid Robotics course.
The textbook must prepare students to:
- Understand embodied intelligence.
- Use ROS 2 for robotic control.
- Build Digital Twin simulations in Gazebo and Unity.
- Use NVIDIA Isaac Sim and Isaac ROS.
- Develop humanoid robot capabilities: perception, locomotion, manipulation.
- Build Vision-Language-Action (VLA) systems using LLMs, Whisper, GPT models.
- Complete a capstone humanoid robotics project.

============================================================
BOOK STRUCTURE (High-Level)
============================================================
The book will contain:
- Introduction & overview.
- Four Major Modules:
  Module 1: ROS 2 — The Robotic Nervous System
  Module 2: Digital Twin Simulation (Gazebo + Unity)
  Module 3: NVIDIA Isaac Platform
  Module 4: Vision-Language-Action Robotics
- Weekly breakdown chapters (aligned to the 13-week course).
- Hardware requirements section.
- Lab architectures, cloud vs. on-prem infrastructure.
- Capstone project documentation.
- Appendices and code examples.

============================================================
DOCUSAURUS PROJECT ARCHITECTURE
============================================================
Use the following folder structure:
docs/
  intro/
  module-1/
  module-2/
  module-3/
  module-4/
  hardware/
  capstone/
  glossary/
  references/

The constitution defines structure, style, and system rules.
Actual writing will occur through iterative /sp.specify → /sp.plan → /sp.task → /sp.implement cycles.

============================================================
RAG CHATBOT REQUIREMENTS
============================================================
A RAG chatbot must be embedded into the published textbook.
Requirements:
- Use OpenAI Agents / ChatKit SDK.
- Backend built with FastAPI.
- Vector store: Qdrant Cloud Free Tier.
- Metadata store: Neon Serverless Postgres.
- Chatbot must:
  1. Answer questions about any chapter.
  2. Support "answer only using the selected text" mode.
  3. Cite sources when appropriate.
  4. Provide contextual comprehension and study assistance.
  5. Support multimodal expansion (if images added later).

Include documentation files explaining:
- RAG pipeline architecture.
- Indexing strategy.
- Embeddings strategy.
- API endpoints.
- Frontend integration.

============================================================
BONUS FEATURE SUPPORT (50–200 Extra Points)
============================================================
The project may optionally include:
- Better-Auth signup + signin.
- User background collection (software and hardware).
- Personalized chapter rendering (button-triggered).
- Urdu translation button (auto-translate chapters).
- Reusable intelligence:
  - Claude Code Subagents
  - Agent Skills
Document and structure the book to support these extensions.

============================================================
WRITING & PEDAGOGY RULES
============================================================
All chapters must follow these principles:
- Technical accuracy first.
- Clear explanations + diagrams (markdown-based).
- Use code blocks for ROS 2, Gazebo, Unity, Isaac, Python, etc.
- Include "Learning Objectives" and "Key Concepts" sections.
- Include end-of-chapter exercises and project tasks.
- Use a mix of tutorials, conceptual explanations, and hands-on labs.
- Provide architecture diagrams in ASCII or markdown.

============================================================
TECHNICAL DEPTH REQUIREMENTS
============================================================
Ensure the textbook content is high-level and industry-aligned:
- ROS 2: Nodes, topics, services, actions, rclpy, URDF.
- Simulation: Gazebo physics, SDF, sensor simulation, Unity rendering.
- NVIDIA Isaac: synthetic data, photorealistic simulation, Nav2, SLAM.
- VLA Robotics: Whisper → intent → task planner → ROS 2 actions.
- Sim-to-Real transfer (structured and emphasized).
- Humanoid robot dynamics, bipedal locomotion, manipulation.

============================================================
HARDWARE SECTION REQUIREMENTS
============================================================
The textbook must explain all hardware tiers from the hackathon document:
- Digital Twin Workstation:
  - GPU: RTX 4070 Ti+ (ideal: 3090/4090)
  - RAM: 64GB recommended
  - Ubuntu 22.04
- Edge AI Kit:
  - Jetson Orin Nano / NX
  - RealSense D435i/D455
  - IMU
  - Microphone (ReSpeaker)
- Robot Lab Options:
  A: Unitree Go2
  B: Miniature humanoids (Unitree G1, Robotis OP3)
  C: Premium humanoid (Unitree G1 for sim-to-real)
- Cloud-native option using AWS g5/g6e instances.

============================================================
STYLE GUIDE
============================================================
- Tone: professional, technical, clear.
- Audience: university-level engineering, AI, robotics students.
- Use consistent formatting for headings, callouts, warnings, commands.
- Include architecture diagrams and flowcharts.
- Explain code with comments.
- Use consistent folder names and filenames.
- Avoid redundant explanations unless needed pedagogically.
- Ensure all chapters are self-contained and logically linked.

============================================================
REPO EXPECTATIONS
============================================================
The project must generate:
- Docusaurus site
- Markdown chapters
- RAG backend code
- API documentation
- Dockerfile (optional)
- Scripts for indexing book content
- Integration notes for frontend ↔ backend
- Instructions for deploying to GitHub Pages or Vercel

============================================================
FLEXIBILITY
============================================================
This constitution defines global rules but does not instruct creation of any specific module.
Actual module creation will occur through subsequent /sp.specify commands.

End of constitution.

## Response snapshot

Created comprehensive Physical AI & Humanoid Robotics Textbook Constitution (v1.0.0) with 8 core principles:

1. **Technical Accuracy First**: All code examples and technical content must be verified and industry-aligned
2. **Pedagogical Excellence**: Structured learning with objectives, concepts, tutorials, exercises in every chapter
3. **Modular Architecture**: 4 modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA Robotics) organized by 13-week course structure
4. **RAG-First Documentation**: Content structured for optimal chatbot retrieval with semantic chunking and metadata
5. **Progressive Complexity**: Beginner→Intermediate→Advanced tiers with gradual concept introduction
6. **Industry Alignment**: ROS 2 Humble+, Python 3.10+, latest Isaac/Gazebo/Unity versions
7. **Extensibility & Bonus Features**: Feature-flagged bonus capabilities (Better-Auth, personalization, Urdu translation, agents)
8. **Deployment & Infrastructure**: Production-ready deployment to GitHub Pages/Vercel with documented FastAPI backend

Defined complete technical standards including:
- Docusaurus configuration requirements
- RAG chatbot API contract (3 endpoints with error handling)
- Code block standards with runnable examples
- Hardware documentation format for all tiers

Established Spec-Driven Development workflow:
- /sp.specify → /sp.plan → /sp.tasks → /sp.implement cycle
- Mandatory PHRs for all work (stored in history/prompts/)
- ADR suggestions for architectural decisions

Created Sync Impact Report documenting template alignment and follow-up requirements.

## Outcome

- ✅ Impact: Established foundational governance for entire AI-native textbook project. All future work must comply with 8 core principles and technical standards.
- 🧪 Tests: N/A (constitutional document, no code tests required)
- 📁 Files: Created `.specify/memory/constitution.md` (v1.0.0, 398 lines)
- 🔁 Next prompts: Run `/sp.specify` for first module (Module 1: ROS 2 or intro section). Consider creating ADR for RAG architecture decisions (Qdrant vs alternatives, chunking strategy).
- 🧠 Reflection: Constitution successfully captures all hackathon requirements while maintaining flexibility for iterative development. RAG-first principle ensures chatbot integration from day 1 rather than retrofit.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution creation completed successfully
- Graders run and results (PASS/FAIL): Manual validation PASS - all placeholders filled, no unresolved tokens, comprehensive principles coverage
- Prompt variant (if applicable): Standard /sp.constitution workflow with comprehensive user requirements
- Next experiment (smallest change to try): Test constitution compliance by creating first module specification and verifying alignment with Principle II (Pedagogical Excellence) structure requirements
