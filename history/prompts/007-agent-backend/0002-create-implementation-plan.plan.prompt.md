---
id: 0002
title: Create RAG Agent Backend implementation plan
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: 007-agent-backend
branch: 007-agent-backend
user: usmankhan
command: /sp.plan 007-rag-agent-backend
labels: ["rag", "agent", "backend", "planning", "architecture"]
links:
  spec: /home/usmankhan/projects/ai_native-textbook/specs/007-agent-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-agent-backend/plan.md
  - specs/007-agent-backend/research.md
  - specs/007-agent-backend/data-model.md
  - specs/007-agent-backend/contracts/api-endpoints.md
  - specs/007-agent-backend/quickstart.md
tests:
  - Constitution Check (all gates passing)
  - Phase 0 research (5 technical questions resolved)
  - Phase 1 design (3 architectural decisions resolved)
---

## Prompt

Create implementation plan for feature 007-rag-agent-backend:

User provided explicit implementation notes:
- Set up FastAPI backend
- Initialize OpenAI Agents SDK using **Gemini API key** as LLM provider (NOT OpenAI)
- Implement agent with dual-mode routing:
  - Whole-book mode: Query Qdrant for relevant chapters, generate answer grounded in retrieval
  - Selected-text mode: Answer using ONLY user-selected text, zero external knowledge leakage
- Integrate feature 006 retrieval pipeline (`search_chunks()` function)
- Design Neon Serverless Postgres schema for:
  - User sessions (UUID-based, persistent)
  - Chat messages (questions + agent responses)
  - Selected-text metadata (audit trail)
- Expose REST API endpoints:
  - POST /api/sessions (create session)
  - GET /api/sessions/{id}/history (retrieve chat history)
  - POST /api/sessions/{id}/chat (whole-book query)
  - POST /api/sessions/{id}/selected-text-chat (selected-text query)
- Target deployment: Render Free Tier or Railway
- Performance goals: <10s whole-book, <5s selected-text (p95 latency)

## Response snapshot

✅ Implementation Plan Complete

Generated comprehensive plan with:
- **Summary**: RAG agent backend with Gemini API, FastAPI, dual-mode querying
- **Technical Context**: Python 3.11+, FastAPI 0.109+, google-generativeai 0.3+, asyncpg 0.29+, Neon Postgres, Qdrant/Cohere reuse
- **Constitution Check**: All gates PASSING (Principles IV, VI, VIII, Code Standards)
- **Project Structure**: Backend extension with new directories (agent/, api/, db/), reuse retrieval/ from feature 006
- **Phase 0 Research**: 5 technical questions resolved:
  1. OpenAI Agents SDK + Gemini → Use direct `google-generativeai` SDK (simpler, fewer deps)
  2. Neon Postgres → `asyncpg` with connection pooling
  3. FastAPI + Agent → Initialize in lifespan context, share via `app.state`
  4. Session isolation → Database filtering by UUID session_id
  5. Zero knowledge leakage → Strict system prompts + validation tests
- **Phase 1 Design**: 3 architectural decisions resolved:
  1. Agent SDK: Direct Gemini SDK (Option B) ✅
  2. Database schema: 3 tables (sessions, messages, selected_text_metadata), no RetrievalResult persistence ✅
  3. API error handling: 7 HTTP status codes (200/201/400/404/429/500/503) ✅
- **Artifacts created**:
  - research.md (Phase 0 findings, technical decisions, dependency list)
  - data-model.md (SQL DDL, Pydantic models, entity relationships)
  - contracts/api-endpoints.md (4 endpoints, request/response schemas, error handling)
  - quickstart.md (developer setup, curl examples, deployment guide)

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All technical decisions resolved, no ambiguities.
- 🧪 Tests: Constitution Check PASS (4/4 gates). Research validated 5 integration patterns. Design resolved 3 architectural decisions.
- 📁 Files: plan.md (259 lines), research.md (350+ lines), data-model.md (250+ lines), contracts/api-endpoints.md (400+ lines), quickstart.md (300+ lines)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks from plan, then `/sp.implement` to execute
- 🧠 Reflection: Key decision was rejecting OpenAI Agents SDK in favor of direct Gemini SDK integration. This simplifies implementation, reduces dependencies, and gives full control over dual-mode prompting (critical for zero external knowledge leakage). Database schema design is straightforward (3 tables, UUID primary keys, JSONB metadata). API design follows REST best practices with comprehensive error handling.

## Evaluation notes (flywheel)

- Failure modes observed: None. Initial assumption was to use OpenAI Agents SDK with `litellm` wrapper, but research revealed simpler direct integration path.
- Graders run and results (PASS/FAIL): Constitution Check PASS (all 4 gates). No violations.
- Prompt variant (if applicable): Single-shot planning with user-provided implementation notes (Gemini API requirement was explicitly stated)
- Next experiment (smallest change to try): Validate Gemini API prompt engineering for selected-text mode during implementation (edge case: test with off-topic text to verify zero external knowledge leakage)
