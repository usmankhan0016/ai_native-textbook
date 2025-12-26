# Tasks: RAG Agent Backend (Feature 007)

**Input**: Design documents from `/specs/007-agent-backend/`
- `spec.md` — 5 user stories (P1: US1-3, P2: US4-5)
- `plan.md` — Technical stack, project structure, Phase 0/1 research
- `data-model.md` — Database schema, Pydantic models
- `contracts/api-endpoints.md` — 4 REST endpoints with full specs
- `research.md` — Technical decisions (Gemini SDK, asyncpg, FastAPI patterns)
- `quickstart.md` — Setup and deployment guide

**Tests**: Not explicitly requested in feature spec. Tests are OPTIONAL and can be added by running implementation tasks with TDD approach per user story.

**Organization**: Tasks grouped by user story (US1-5) for independent implementation and testing. Setup/Foundational phases completed first (blocking prerequisites).

---

## Implementation Strategy

**MVP Scope** (Phase 1-3 recommended):
- ✅ Phase 1: Setup (project structure, dependencies)
- ✅ Phase 2: Foundational (database, Gemini agent, API framework)
- ✅ Phase 3: User Story 1 (whole-book Q&A)
- ✅ Phase 4: User Story 2 (selected-text Q&A)
- ✅ Phase 5: User Story 3 (persistent chat sessions)
- ⏸️ Phase 6: User Story 4 (concurrent users) — Can defer, design supports this
- ⏸️ Phase 7: User Story 5 (source attribution) — Can defer, contracts define this

**Parallel Opportunities**:
- Setup phase: All tasks [P] can run in parallel
- Foundational phase: Database schema + Gemini agent init can run in parallel
- User Story 1 & 2: Can run in parallel (independent endpoints)
- User Story 4 & 5: Can run in parallel if needed

**Dependency Graph**:
```
Phase 1: Setup
    ↓
Phase 2: Foundational (database, agent, API)
    ├→ Phase 3: US1 (whole-book queries)
    ├→ Phase 4: US2 (selected-text queries)
    └→ Phase 5: US3 (persistent sessions)
        ├→ Phase 6: US4 (concurrency)
        └→ Phase 7: US5 (source attribution)
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and basic structure

**Acceptance**: Dependencies installed, FastAPI app runs, database connection configured

- [ ] T001 Create backend directory structure per implementation plan (`backend/agent/`, `backend/api/`, `backend/db/`)
- [ ] T002 Update `backend/requirements.txt` with new dependencies:
  - fastapi==0.109.0
  - uvicorn[standard]==0.27.0
  - pydantic==2.5.0
  - google-generativeai==0.3.2
  - asyncpg==0.29.0
  - pytest-asyncio==0.21.1
  - httpx==0.25.2
- [ ] T003 [P] Create `backend/.env.example` with required API keys (GEMINI_API_KEY, DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, QDRANT_COLLECTION_NAME)
- [ ] T004 [P] Initialize FastAPI application in `backend/api/main.py` with health check endpoint (`GET /health`)
- [ ] T005 [P] Create empty module files for organization: `backend/agent/__init__.py`, `backend/api/__init__.py`, `backend/api/routes/__init__.py`, `backend/db/__init__.py`
- [ ] T006 Update `backend/README.md` with quickstart instructions (database setup, environment variables, running server)

**Checkpoint**: Project structure ready, dependencies installed, FastAPI server starts

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST complete before any user story implementation

**⚠️ CRITICAL**: No user story work begins until this phase completes

**Acceptance**: Database connected, Gemini agent initialized, API routing established, all databases accessible

### Database Layer

- [ ] T007 Create Neon Postgres schema in `backend/db/migration.sql`:
  - `sessions` table (id UUID, created_at, updated_at, metadata JSONB)
  - `messages` table (id, session_id, role, content, created_at, mode, metadata)
  - `selected_text_metadata` table (id, message_id, selected_text, chapter_origin, created_at)
  - Indexes: `(session_id, created_at)` on messages, `message_id` on selected_text_metadata
- [ ] T008 Implement `backend/db/connection.py` with asyncpg connection pool:
  - `create_pool()` function (min_size=2, max_size=10, timeout=30)
  - `get_db_pool()` dependency injection for FastAPI
  - Database initialization in FastAPI lifespan context
- [ ] T009 [P] Create SQLAlchemy/raw SQL models in `backend/db/models.py`:
  - `Session` model (id, created_at, updated_at, metadata)
  - `Message` model (id, session_id, role, content, created_at, mode, metadata)
  - `SelectedTextMetadata` model (id, message_id, selected_text, chapter_origin, created_at)
- [ ] T010 [P] Implement `backend/db/repositories.py` with CRUD operations:
  - `SessionRepository.create()`, `get_by_id()`, `list_messages()`, `delete()`
  - `MessageRepository.create()`, `list_by_session()`, `get_by_id()`
  - `SelectedTextMetadataRepository.create()`, `get_by_message_id()`

### Agent Layer

- [ ] T011 Implement `backend/agent/core.py` with Gemini API integration:
  - Initialize `google.generativeai.GenerativeModel("gemini-1.5-pro")` in FastAPI lifespan
  - Store model in `app.state.gemini_model` for access in request handlers
  - Configure API key from environment variable
- [ ] T012 [P] Create `backend/agent/prompts.py` with system prompts:
  - `WHOLE_BOOK_SYSTEM_PROMPT`: "You are a helpful assistant. Answer questions about AI-native robotics using the provided context from a curated textbook. Always cite sources."
  - `SELECTED_TEXT_SYSTEM_PROMPT`: "You are a helpful assistant. Answer the user's question using ONLY the text provided below. Do NOT use any external knowledge. If the answer is not in the provided text, respond: 'The provided text does not contain enough information to answer this question.'"
- [ ] T013 [P] Create `backend/agent/modes.py` with routing logic:
  - `route_whole_book_query(query: str, db_pool) → (gemini_response: str, sources: List[Source])`
  - `route_selected_text_query(query: str, selected_text: str, db_pool) → (gemini_response: str)`
  - Integration with feature 006 `retrieval.search_chunks()` for whole-book mode

### API Layer

- [ ] T014 Create Pydantic models in `backend/api/models.py`:
  - `SessionResponse`: id, created_at, updated_at, metadata
  - `MessageResponse`: id, session_id, role, content, created_at, mode, metadata
  - `ChatRequest`: query, mode="whole_book"
  - `SelectedTextChatRequest`: query, selected_text, chapter_origin, mode="selected_text"
  - `RetrievalSource`: chapter, relevance_score, text_preview
  - `ChatResponse`: message, sources (optional)
  - `ChatHistoryResponse`: session_id, messages
- [ ] T015 [P] Initialize API routing in `backend/api/routes/sessions.py`:
  - Import `FastAPI`, `HTTPException`, dependency injection setup
  - Placeholder route definitions (will be filled in user story phases)
- [ ] T016 [P] Initialize API routing in `backend/api/routes/chat.py`:
  - Import `FastAPI`, dependency injection setup
  - Placeholder route definitions (will be filled in user story phases)
- [ ] T017 Configure error handling in `backend/api/main.py`:
  - Global exception handler for `HTTPException` → return status code + detail
  - Logging setup (import logging, configure for FastAPI)
  - Include routes from `api/routes/sessions.py` and `api/routes/chat.py`
- [ ] T018 Add FastAPI lifespan context in `backend/api/main.py`:
  - Initialize database connection pool on startup
  - Initialize Gemini agent on startup
  - Cleanup on shutdown

**Checkpoint**: Database accessible, Gemini agent initialized, API framework ready, all dependencies configured

---

## Phase 3: User Story 1 - Ask Questions Using Whole-Book Context (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the entire textbook. The agent retrieves relevant chapters via Qdrant semantic search and generates answers grounded in those results.

**Acceptance**: REST endpoint works, agent retrieves chapters from Qdrant, returns coherent answers with source attribution

**Independent Test**: `curl -X POST http://localhost:8000/api/sessions/{id}/chat -d '{"query": "What is Isaac Sim?"}' → agent returns answer + sources from Isaac Sim chapter`

### Implementation for User Story 1

- [ ] T019 [US1] Implement `POST /api/sessions/{session_id}/chat` in `backend/api/routes/chat.py`:
  - Parse `ChatRequest` (query, mode="whole_book")
  - Validate session exists (query database)
  - Call `route_whole_book_query(query)` from `backend/agent/modes.py`
  - Store user question in messages table (role="user", mode="whole_book")
  - Store agent response in messages table (role="assistant")
  - Return `ChatResponse` with message + sources
- [ ] T020 [US1] Enhance `backend/agent/modes.py` whole-book routing:
  - Retrieve chat history for session (for context in multi-turn conversations)
  - Call `retrieval.search_chunks(query, qdrant_client, cohere_client, top_k=5)` to get relevant chapters
  - Build prompt: `WHOLE_BOOK_SYSTEM_PROMPT + retrieved_chapters + query`
  - Call Gemini API: `gemini_model.generate_content(messages=...)`
  - Parse response and extract chapter names from retrieval results as sources
  - Return formatted response with relevance scores
- [ ] T021 [US1] Add input validation to whole-book endpoint:
  - Query length: 1-50,000 characters (raise HTTPException 400 if invalid)
  - Session exists validation (raise HTTPException 404 if not found)
  - Gemini API error handling (rate limits → 429, other errors → 500)
- [ ] T022 [US1] Add logging to `backend/agent/modes.py`:
  - Log query, retrieved chapters, latency_ms, model version to `messages.metadata`
  - Log errors with full stack trace
- [ ] T023 [US1] Implement `GET /api/sessions/{session_id}/history` in `backend/api/routes/sessions.py`:
  - Parse session_id from path
  - Query messages table filtered by session_id, ordered by created_at ASC
  - Return `ChatHistoryResponse` with all messages

**Checkpoint**: Whole-book queries work end-to-end, chat history retrievable, responses grounded in Qdrant results

---

## Phase 4: User Story 2 - Ask Questions Using Only Selected Text (Priority: P1)

**Goal**: Enable users to ask questions about text they have selected, with zero external knowledge leakage. The agent answers using ONLY the provided text.

**Acceptance**: REST endpoint works, agent uses ONLY selected text, refuses to answer with external knowledge

**Independent Test**: `curl -X POST http://localhost:8000/api/sessions/{id}/selected-text-chat -d '{"query": "What is URDF?", "selected_text": "URDF is..."}' → agent answers using only selected text`

### Implementation for User Story 2

- [ ] T024 [US2] Implement `POST /api/sessions/{session_id}/selected-text-chat` in `backend/api/routes/chat.py`:
  - Parse `SelectedTextChatRequest` (query, selected_text, chapter_origin, mode="selected_text")
  - Validate session exists
  - Call `route_selected_text_query(query, selected_text)` from `backend/agent/modes.py`
  - Store user question in messages table (role="user", mode="selected_text")
  - Store selected_text_metadata in selected_text_metadata table
  - Store agent response in messages table (role="assistant")
  - Return `ChatResponse` with message + sources=null
- [ ] T025 [US2] Enhance `backend/agent/modes.py` selected-text routing:
  - Retrieve chat history for session
  - Skip Qdrant retrieval (no external context)
  - Build prompt: `SELECTED_TEXT_SYSTEM_PROMPT + selected_text + query`
  - Call Gemini API with strict system prompt
  - Return response (no sources, since no retrieval)
  - Store metadata: latency_ms, selected_text_length, gemini_model version
- [ ] T026 [US2] Add input validation to selected-text endpoint:
  - Query length: 1-50,000 characters (raise HTTPException 400)
  - Selected text length: 1-100,000 characters (raise HTTPException 400)
  - Selected text non-empty/non-whitespace (raise HTTPException 400)
  - Session exists (raise HTTPException 404)
  - Gemini API errors (429, 500, 503)
- [ ] T027 [US2] Add validation test for zero external knowledge:
  - Verify agent response includes "provided text does not contain" for off-topic questions
  - Log and monitor for external knowledge leakage in response analysis
- [ ] T028 [US2] Store selected_text metadata audit trail:
  - Create entry in `selected_text_metadata` with: selected_text, chapter_origin, message_id
  - Enable post-hoc analysis of which sections users query most frequently

**Checkpoint**: Selected-text queries work, agent respects local-context-only constraint, metadata captured for audit

---

## Phase 5: User Story 3 - Maintain Persistent Chat Sessions (Priority: P1)

**Goal**: Enable multi-turn conversations where users can ask follow-ups that depend on prior context. Chat history persists across browser sessions.

**Acceptance**: Sessions persist, chat history retrieved correctly, agent can access prior context for follow-ups

**Independent Test**: Create session → ask question → ask follow-up → verify agent context includes prior exchange

### Implementation for User Story 3

- [ ] T029 [US3] Implement `POST /api/sessions` in `backend/api/routes/sessions.py`:
  - Parse `CreateSessionRequest` (optional metadata)
  - Create new session in database (auto-generate UUID)
  - Return `SessionResponse` with id, created_at, updated_at, metadata
- [ ] T030 [US3] Enhance chat endpoints to load conversation history:
  - In both `route_whole_book_query()` and `route_selected_text_query()`:
    - Load prior messages from session (ordered by created_at ASC)
    - Format as conversation array: [{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]
    - Pass full conversation to Gemini API for context-aware responses
  - Gemini conversation format: Pass all messages in single request (not turn-by-turn)
- [ ] T031 [US3] Implement session update on new message:
  - When message is stored, update `sessions.updated_at` to NOW()
  - Enables tracking active sessions and time-based cleanup
- [ ] T032 [US3] Add validation for session lifecycle:
  - Verify session exists before accepting queries (404 if not found)
  - Handle session not found errors gracefully in all endpoints
- [ ] T033 [US3] Add tests for multi-turn conversations:
  - Create session → ask "What is Isaac Sim?" → ask "Can you give me an example?" → verify agent references prior answer
  - Verify chat history returned in correct order
  - Verify session metadata updated on each message

**Checkpoint**: Sessions created and persisted, multi-turn conversations work, full chat history accessible

---

## Phase 6: User Story 4 - Support Multiple Concurrent Users (Priority: P2)

**Goal**: Backend handles multiple users simultaneously without interference or data leakage. Each user's sessions isolated.

**Acceptance**: Load test passes, no cross-contamination, response times stable under load

**Independent Test**: Concurrent requests from 10+ users → each receives correct isolated response

### Implementation for User Story 4

- [ ] T034 [US4] Validate session isolation in database:
  - Session IDs are UUIDs (globally unique, impossible to guess)
  - Messages filtered by session_id (no way to access other sessions)
  - No shared state between sessions in agent (each call loads its own history)
  - Database transaction isolation (ACID guarantees)
- [ ] T035 [US4] Implement connection pool monitoring:
  - Log connection pool stats: active_connections, idle_connections, waiting_requests
  - Monitor in `backend/db/connection.py` via asyncpg pool introspection
  - Alert if pool exhaustion (max_size=10 limit)
- [ ] T036 [US4] Load test concurrent users:
  - Create test script: simultaneous requests from 10 users to different sessions
  - Each user: create session → ask 3 questions in sequence
  - Verify: no data leakage, all responses correct, <5s response times
  - Test file: `backend/tests/test_concurrency.py` (optional, recommended for confidence)
- [ ] T037 [US4] Add monitoring for concurrent request safety:
  - Ensure database connection pool doesn't exhaust
  - Monitor API response times under load
  - Set up alerting for response time degradation (>10s for whole-book, >5s for selected-text)

**Checkpoint**: Concurrent users handled safely, isolation verified, performance stable

---

## Phase 7: User Story 5 - Retrieve and Display Agent Reasoning (Priority: P2)

**Goal**: Users and developers see sources (chapters, relevance scores) used by agent. Transparency for trust and debugging.

**Acceptance**: Responses include source attribution, relevance scores visible, selected-text mode clearly labeled

**Independent Test**: Query agent → response includes chapter names and scores for whole-book, or "Based on selected text" label for selected-text mode

### Implementation for User Story 5

- [ ] T038 [US5] Enhance whole-book response formatting in `backend/agent/modes.py`:
  - Extract sources from retrieval results: [{"chapter": "Chapter 2", "relevance_score": 0.89, "text_preview": "..."}]
  - Include sources in `ChatResponse.sources` field
  - Format response to include "Sources:" section with chapter names
- [ ] T039 [US5] Add source attribution to messages metadata:
  - Store in `messages.metadata`: retrieval_count, top_chapter, chapters_used (array)
  - Enable analytics: which chapters queried most frequently
- [ ] T040 [US5] Label selected-text responses clearly:
  - Include in response: "This answer is based on your selected text only."
  - Store indicator in `messages.metadata`: is_selected_text=true
- [ ] T041 [US5] Implement source preview generation:
  - Extract first 200 characters of relevant chunks from Qdrant results
  - Include in `RetrievalSource.text_preview` in API response
  - Helps users verify relevance of sources
- [ ] T042 [US5] Add response formatting for multi-source answers:
  - When multiple sources used, format response with section headers: "Based on Chapter X:" followed by synthesized content
  - Make it clear which facts come from which sources

**Checkpoint**: Source attribution visible, responses formatted for clarity, debugging information available

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final refinements, testing, documentation, deployment preparation

**Acceptance**: All endpoints tested, documentation complete, deployment ready, error messages user-friendly

- [ ] T043 Add comprehensive error messages in `backend/api/main.py`:
  - 400: "Query must be between 1 and 50,000 characters"
  - 404: "Session not found. Create a new session with POST /api/sessions"
  - 429: "Rate limit exceeded. Please try again in 1 minute."
  - 500: "Failed to process query. Please try again later."
  - 503: "Retrieval service temporarily unavailable. Please try again."
- [ ] T044 [P] Implement comprehensive logging in all modules:
  - `backend/agent/modes.py`: Log query, latency, retrieved chapters, errors
  - `backend/api/main.py`: Log HTTP requests, response codes, errors
  - `backend/db/repositories.py`: Log database operations, errors
  - Use Python logging with structured JSON output (for production)
- [ ] T045 [P] Add type hints and docstrings throughout:
  - All function signatures with type hints (FastAPI requires Pydantic)
  - All public functions with docstrings explaining purpose, arguments, return value
- [ ] T046 Create integration test suite in `backend/tests/integration/`:
  - `test_whole_book_flow.py`: Create session → whole-book query → verify response
  - `test_selected_text_flow.py`: Create session → selected-text query → verify response
  - `test_multi_turn_conversation.py`: Multi-turn conversation flow
  - `test_session_isolation.py`: Concurrent user isolation
- [ ] T047 Update `backend/README.md` with complete documentation:
  - Environment setup (Python, venv, dependencies)
  - Database initialization (Neon Postgres schema)
  - Configuration (API keys, .env)
  - Running server locally
  - Running tests
  - API reference (link to contracts/api-endpoints.md)
  - Troubleshooting (common errors)
  - Deployment guide (Render/Railway)
- [ ] T048 Create deployment checklist:
  - Neon Postgres database created and schema migrated
  - Gemini, Cohere, Qdrant API keys obtained
  - Environment variables configured in deployment platform
  - FastAPI server starts without errors
  - Health check endpoint responds (GET /health → 200)
  - Sample query works end-to-end
- [ ] T049 Add performance monitoring:
  - Instrument whole-book and selected-text endpoints with latency tracking
  - Store in `messages.metadata`: latency_ms, retrieval_count (whole-book), selected_text_length (selected-text)
  - Verify: whole-book <10s, selected-text <5s (p95)
- [ ] T050 [P] Security review:
  - SQL injection: Verify all queries use parameterized statements (asyncpg safe)
  - XSS: Verify no user input embedded in responses without sanitization
  - Session hijacking: Verify session IDs are UUIDs (cryptographically random)
  - Rate limiting: Document Gemini API rate limits and recovery strategy

**Checkpoint**: Feature complete, tested, documented, deployment-ready

---

## Summary

**Total Tasks**: 50 tasks across 8 phases

**Tasks per User Story**:
- Setup: 6 tasks
- Foundational: 12 tasks (blocking prerequisites)
- US1 (whole-book): 5 tasks
- US2 (selected-text): 5 tasks
- US3 (sessions): 5 tasks
- US4 (concurrency): 4 tasks
- US5 (sources): 5 tasks
- Polish: 8 tasks

**Parallel Opportunities**:
- Phase 1: All 6 tasks can run in parallel (different files)
- Phase 2: T007-T010 (database) and T011-T013 (agent) can run in parallel
- Phase 3 & 4: Can run in parallel (independent endpoints)
- Phase 6 & 7: Can run in parallel if Phase 5 complete

**MVP Scope Recommendation**:
- **Essential (Phases 1-5)**: 38 tasks for MVP with whole-book Q&A, selected-text Q&A, and persistent sessions
- **Nice-to-have (Phases 6-8)**: 12 additional tasks for concurrency, source attribution, and final polish

**Dependency Graph**:
```
Phase 1 (Setup: 6)
  ↓
Phase 2 (Foundational: 12) ← All subsequent phases blocked until complete
  ├→ Phase 3 (US1: 5)
  ├→ Phase 4 (US2: 5) ← Can run parallel with US1
  └→ Phase 5 (US3: 5) ← Depends on US1 & US2 functionality
      ├→ Phase 6 (US4: 4) ← Optional, nice-to-have
      └→ Phase 7 (US5: 5) ← Optional, nice-to-have
  ↓
Phase 8 (Polish: 8) ← Final polish and deployment prep
```

---

**Next Step**: Run `/sp.implement` to execute tasks or start with Phase 1 Setup tasks manually.

**Validation**: Each task has acceptance criteria and clear file paths. Implementation can proceed task-by-task, with full end-to-end validation after each phase.
