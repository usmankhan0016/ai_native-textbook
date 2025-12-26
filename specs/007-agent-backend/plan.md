# Implementation Plan: RAG Agent Backend

**Branch**: `007-agent-backend` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-agent-backend/spec.md`

## Summary

Build a production-ready RAG agent backend using OpenAI Agents SDK (with Gemini API as LLM provider) that supports two query modes: (1) whole-book retrieval via Qdrant semantic search, and (2) selected-text-only Q&A with zero external knowledge leakage. The system exposes FastAPI REST endpoints for session-based chat with persistent state stored in Neon Serverless Postgres. The agent integrates the validated retrieval pipeline from feature 006 for grounding responses in curated textbook content.

## Technical Context

**Language/Version**: Python 3.11+ (aligns with feature 005/006 backend)
**Primary Dependencies**:
- FastAPI 0.109+ (REST API framework)
- OpenAI Agents SDK (agent orchestration, using Gemini API as LLM backend)
- google-generativeai 0.3+ (Gemini API client)
- psycopg2 2.9+ or asyncpg 0.29+ (Neon Postgres connector)
- Reuse from feature 006: qdrant-client, cohere (for retrieval pipeline)

**Storage**:
- Neon Serverless Postgres (sessions, messages, selected-text metadata)
- Qdrant Cloud (reuse existing collection from feature 005/006)

**Testing**: pytest 7.4+, pytest-asyncio 0.21+, httpx 0.25+ (for FastAPI test client)

**Target Platform**: Linux server (Render Free Tier, Railway, or local development)

**Project Type**: Web backend (backend/ directory, extends existing feature 005/006 structure)

**Performance Goals**:
- <10s p95 latency for whole-book queries (Qdrant + Gemini inference)
- <5s p95 latency for selected-text queries (Gemini inference only)
- Support ≥10 concurrent users without degradation

**Constraints**:
- Zero external knowledge leakage in selected-text mode (strict prompt engineering)
- Session isolation (no cross-contamination between users)
- Synchronous request/response (no async job queues for MVP)

**Scale/Scope**:
- MVP: 10-100 queries/day, single-node deployment
- Database: ~1000 sessions, ~10,000 messages expected in first month
- Code: ~800-1200 lines (agent logic, API endpoints, DB models, tests)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle IV: RAG-First Documentation ✅ PASS
- **Requirement**: API endpoints must support RAG chatbot integration
- **Verification**: Endpoints designed for chat-based Q&A with source attribution
- **Status**: Aligned - system IS the RAG backend infrastructure

### Principle VI: Industry Alignment ✅ PASS
- **Requirement**: Use 2025-current tools and frameworks
- **Verification**:
  - Python 3.11+ ✅
  - FastAPI (modern async Python web framework) ✅
  - OpenAI Agents SDK (latest agent orchestration) ✅
  - Gemini API (2024-2025 LLM provider) ✅
  - Neon Serverless Postgres (modern cloud database) ✅
- **Status**: All dependencies align with current industry standards

### Principle VIII: Deployment & Infrastructure ✅ PASS
- **Requirement**: Clear deployment targets and infrastructure documentation
- **Verification**:
  - Target platform: Render Free Tier / Railway (documented)
  - Infrastructure components: FastAPI backend, Neon Postgres, Qdrant Cloud
  - Required documentation: README, API contracts, quickstart guide (planned in Phase 1)
- **Status**: Infrastructure plan complete and documented

### Code Standards ✅ PASS
- **Requirement**: Python type hints, error handling, linting
- **Verification**:
  - Type hints: Will use throughout (FastAPI requires Pydantic models with types)
  - Error handling: Explicit HTTP status codes (400/429/500/503) in spec
  - Linting: Reuse ruff/black/mypy from feature 005/006 setup
- **Status**: Standards will be enforced via existing backend config

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent/                     # NEW: Agent orchestration and session management
│   ├── __init__.py
│   ├── core.py               # OpenAI Agents SDK setup with Gemini API
│   ├── modes.py              # Whole-book vs selected-text routing logic
│   └── prompts.py            # System prompts for dual-mode agent
├── api/                       # NEW: FastAPI REST endpoints
│   ├── __init__.py
│   ├── main.py               # FastAPI app initialization
│   ├── routes/
│   │   ├── __init__.py
│   │   ├── sessions.py       # POST /api/sessions, GET /api/sessions/{id}/history
│   │   └── chat.py           # POST /api/sessions/{id}/chat, /selected-text-chat
│   └── models.py             # Pydantic request/response models
├── db/                        # NEW: Neon Postgres database layer
│   ├── __init__.py
│   ├── connection.py         # Database connection pool
│   ├── models.py             # SQLAlchemy/asyncpg models (Session, Message, SelectedTextMetadata)
│   └── repositories.py       # CRUD operations for sessions and messages
├── retrieval/                 # EXISTING: Reuse from feature 006
│   ├── __init__.py
│   ├── search.py             # search_chunks() function (validated)
│   ├── validators.py         # Metadata validation (validated)
│   └── test_queries.py       # Test queries (validated)
├── tests/                     # EXTENDED: Add agent and API tests
│   ├── agent/
│   │   ├── test_whole_book_mode.py
│   │   └── test_selected_text_mode.py
│   ├── api/
│   │   ├── test_sessions.py
│   │   └── test_chat.py
│   ├── db/
│   │   └── test_repositories.py
│   └── retrieval/            # EXISTING: From feature 006
│       ├── test_retriever.py
│       └── test_validators.py
├── .env.example              # EXTENDED: Add Gemini API key, Neon Postgres URL
├── requirements.txt          # EXTENDED: Add fastapi, openai-agents-sdk, google-generativeai, psycopg2/asyncpg
└── README.md                 # EXTENDED: Add API quickstart and deployment guide
```

**Structure Decision**: **Web backend extension (Option 2 variant)**

This feature extends the existing `backend/` directory structure from features 005/006. Rationale:
- **Reuse validated components**: The `retrieval/` module from feature 006 is proven (100% accuracy) and reused without modification
- **Separation of concerns**: New directories (`agent/`, `api/`, `db/`) isolate agent logic, REST endpoints, and database operations
- **Testing isolation**: Each new module has corresponding test directory for unit and integration testing
- **Deployment simplicity**: Single backend service (no separate frontend) simplifies deployment to Render/Railway
- **Future extensibility**: Clear boundaries allow future features (e.g., frontend integration, analytics) without restructuring

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status**: No constitution violations detected. All checks passing.

---

## Phase 0: Research

**Status**: ✅ COMPLETE

**Objective**: Validate technical feasibility and resolve any NEEDS CLARIFICATION items from Technical Context section.

### Research Findings

**1. OpenAI Agents SDK with Gemini API Integration**
- **Finding**: OpenAI Agents SDK does NOT natively support Gemini API as an LLM provider
- **Resolution**: Use `litellm` library as a unified interface to route OpenAI SDK calls to Gemini
- **Implementation approach**:
  ```python
  from litellm import completion

  # Configure for Gemini
  response = completion(
      model="gemini/gemini-1.5-pro",
      messages=[{"role": "user", "content": "Query text"}],
      api_key=GEMINI_API_KEY
  )
  ```
- **Alternative**: Use Google's `google-generativeai` SDK directly instead of OpenAI Agents SDK
- **Decision**: Research required in Phase 1 to determine best integration path

**2. Neon Serverless Postgres Connection Pattern**
- **Finding**: Neon requires connection pooling for serverless workloads
- **Recommended approach**: `asyncpg` for async FastAPI + connection pooling via `asyncpg.create_pool()`
- **Schema design**: Sessions table with UUID primary keys, Messages table with foreign key to sessions

**3. FastAPI + OpenAI Agents Integration Pattern**
- **Finding**: Standard pattern is to initialize agent in FastAPI lifespan context and share via dependency injection
- **Example**:
  ```python
  @asynccontextmanager
  async def lifespan(app: FastAPI):
      # Initialize agent on startup
      app.state.agent = initialize_agent()
      yield
      # Cleanup on shutdown
  ```

**4. Session Isolation Strategy**
- **Finding**: Session isolation requires separate conversation threads per session ID
- **Implementation**: Store session_id in Messages table, filter by session_id when loading chat history

**5. Zero External Knowledge Leakage for Selected-Text Mode**
- **Finding**: Requires strict system prompt engineering:
  - "You are a helpful assistant. Answer ONLY using the text provided below. Do not use any external knowledge."
  - "If the answer is not in the provided text, say 'The provided text does not contain enough information to answer this question.'"
- **Validation approach**: Test with off-topic selected text (e.g., recipe) and verify agent refuses to answer unrelated questions

---

## Phase 1: Design

**Status**: ✅ COMPLETE

### Artifacts Created

1. ✅ **data-model.md**: Database schema (sessions, messages, selected_text_metadata), entity relationships, Pydantic API models
2. ✅ **contracts/api-endpoints.md**: Full API specifications with request/response schemas, error handling, and HTTP status codes
3. ✅ **quickstart.md**: Developer setup guide with curl examples, deployment instructions, and troubleshooting

### Design Decisions Resolved

**Decision 1: OpenAI Agents SDK vs Direct Gemini SDK** → **RESOLVED**
- **Selected**: Option B (Direct `google-generativeai` SDK)
- **Rationale**: Simpler integration path, fewer dependencies, clearer control flow
- **Implementation**: Use `google.generativeai.GenerativeModel` with custom prompt engineering for dual-mode agent
- **Documented in**: data-model.md (research findings), quickstart.md (setup)

**Decision 2: Database Schema Design** → **RESOLVED**
- **Tables**: `sessions`, `messages`, `selected_text_metadata`
- **Key decisions**:
  - ✅ RetrievalResult: NOT persisted (computed on-demand, store metadata in messages.metadata)
  - ✅ Gemini responses: NOT stored raw (store only assistant message content)
  - ✅ Indexes: Composite index on `(session_id, created_at)` for chat history retrieval
- **Documented in**: data-model.md (full DDL with indexes)

**Decision 3: API Error Handling Strategy** → **RESOLVED**
- **HTTP Status Codes**:
  - 200: Success (GET /history, POST /chat)
  - 201: Created (POST /sessions)
  - 400: Bad request (malformed input, validation errors)
  - 404: Session not found
  - 429: Rate limit exceeded (Gemini API)
  - 500: Internal error (database, agent failure)
  - 503: Service unavailable (Qdrant/Gemini down)
- **Documented in**: contracts/api-endpoints.md (full error response schemas)

---

## Next Steps

1. **Create data-model.md**: Define database schema, entity relationships, Pydantic models
2. **Create contracts/**: API endpoint specifications with OpenAPI-style schemas
3. **Create quickstart.md**: Developer setup guide with example curl commands
4. **Update agent context**: Ensure `.specify/memory/agent.md` reflects new feature structure
5. **Create PHR**: Document this planning phase in `history/prompts/007-agent-backend/`
6. **Run `/sp.tasks`**: Generate implementation tasks from this plan
