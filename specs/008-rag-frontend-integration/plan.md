# Implementation Plan: RAG Frontend Integration

**Branch**: `008-rag-frontend-integration` | **Date**: 2025-12-22 | **Spec**: [Feature 008](spec.md)
**Input**: Feature specification from `/specs/008-rag-frontend-integration/spec.md`

## Summary

Integrate the existing Docusaurus chatbot component (`docusaurus_textbook/src/components/ChatbotWidget`) with the RAG backend (Feature 007) to enable end-to-end question answering in two modes:
1. **Whole-Book Mode**: Retrieve relevant sections from Qdrant + generate answers with source attribution
2. **Selected-Text Mode**: Generate answers using only user-selected text (zero external knowledge)

Technical approach: Modify ChatbotWidget to call FastAPI `/api/sessions/{id}/chat` and `/api/sessions/{id}/selected-text-chat` endpoints, implement right-click context menu for selected-text mode, persist session IDs via localStorage/URL params, and display chat history with proper loading/error states.

## Technical Context

**Language/Version**: TypeScript 5.x, React 18.x (Docusaurus stack)
**Primary Dependencies**:
- Frontend: React, Docusaurus 3.x, axios/fetch API, localStorage API
- Backend: FastAPI (Python 3.10+), asyncpg, Gemini API, Cohere API, Qdrant
- Storage: Neon Serverless Postgres (sessions, messages, metadata)
**Storage**: Neon Serverless Postgres for session persistence, in-memory for runtime state
**Testing**: Jest/Vitest for frontend unit tests, Playwright for E2E tests
**Target Platform**: Modern web browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web (frontend + existing backend)
**Performance Goals**:
- Whole-book queries: <15 seconds end-to-end
- Selected-text queries: <5 seconds end-to-end
- Chat history load: <1 second
**Constraints**:
- No authentication required (public chat)
- Non-breaking changes only (additive to ChatbotWidget)
- CORS configured for Docusaurus ↔ FastAPI
- Responsive UI (no blocking during API calls)
**Scale/Scope**: Single chatbot integration, 2 query modes, session persistence, error handling for 6 error codes

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Technical Accuracy First**
- ✅ **PASS**: Using official Docusaurus 3.x APIs, React 18.x best practices, FastAPI patterns validated in Feature 007. All API endpoints verified with unit tests in backend.

**Principle II: Pedagogical Excellence**
- ✅ **PASS**: This is an integration feature (not content). Chatbot UI enhancement serves pedagogy by providing student Q&A access. No pedagogical violations.

**Principle III: Modular Architecture**
- ✅ **PASS**: ChatbotWidget changes are encapsulated. Non-breaking changes only. Backend integration via clean API contracts. No modification to Docusaurus module structure.

**Principle IV: RAG-First Documentation**
- ✅ **PASS**: This feature *enables* RAG chatbot functionality. Spec requires metadata capture (chapter origin, relevance scores). Session/message persistence supports RAG observability.

**Principle V: Progressive Complexity**
- ✅ **PASS**: Not applicable to integration feature. Complexity introduced is frontend library integration, not content.

**Principle VI: Industry Alignment**
- ✅ **PASS**: Using current standards (TypeScript 5.x, React 18.x, FastAPI, Docusaurus 3.x, Neon Postgres). No outdated frameworks.

**Principle VII: Extensibility & Bonus Features**
- ✅ **PASS**: Designed for extensibility: session metadata supports user context, message metadata extensible for future use-cases (e.g., user feedback, rating).

**Principle VIII: Deployment & Infrastructure**
- ✅ **PASS**: Integrates with existing infrastructure (Neon, Qdrant, Gemini, FastAPI). Deployed as part of Docusaurus build. No new infrastructure required.

**Overall**: ✅ **CONSTITUTION GATE PASSED** - All 8 principles satisfied. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/008-rag-frontend-integration/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (NEXT: research.md resolves unknowns)
├── data-model.md        # Phase 1 output (NEXT: data model + relationships)
├── quickstart.md        # Phase 1 output (NEXT: setup & integration guide)
├── contracts/           # Phase 1 output (NEXT: API contract specs)
│   ├── session-api.yml
│   ├── chat-api.yml
│   └── selected-text-api.yml
└── tasks.md             # Phase 2 output (NOT created by /sp.plan, created by /sp.tasks)
```

### Source Code (Web Application Structure)

**Frontend (Docusaurus)** - Existing structure, minimal modifications:
```text
docusaurus_textbook/
├── src/
│   └── components/
│       ├── ChatbotWidget/          # ← MODIFY (existing component)
│       │   ├── ChatbotWidget.tsx   # Main component (modify to add API integration)
│       │   ├── useChat.ts          # NEW: Custom hook for API calls
│       │   ├── useSession.ts       # NEW: Custom hook for session management
│       │   ├── useSelectedText.ts  # NEW: Custom hook for text selection handling
│       │   ├── ChatMessage.tsx     # NEW or MODIFY: Message display component
│       │   ├── SourcesList.tsx     # NEW: Component for displaying retrieval sources
│       │   ├── ErrorBoundary.tsx   # NEW: Error handling component
│       │   └── __tests__/
│       │       ├── ChatbotWidget.test.tsx
│       │       ├── useChat.test.ts
│       │       └── useSession.test.ts
│       └── ...other components
├── public/
│   └── .env.local       # NEW: Backend URL configuration (local dev)
└── .env.example         # NEW: Example env variables

docusaurus.config.js    # MODIFY: Ensure ChatbotWidget loaded on all pages
```

**Backend** (Feature 007, already exists):
```text
backend/
├── api/
│   ├── main.py         # VERIFY: CORS configured for Docusaurus origin
│   ├── sessions.py     # VERIFY: Session endpoints working
│   └── chat.py         # VERIFY: Whole-book & selected-text endpoints
├── db/
│   ├── repositories.py # VERIFY: Session/message persistence
│   └── schema.sql      # VERIFY: Tables for sessions/messages/metadata
└── .env                # VERIFY: All credentials available
```

**Integration Points**:
- Frontend calls: `POST /api/sessions` (create session)
- Frontend calls: `POST /api/sessions/{id}/chat` (whole-book query)
- Frontend calls: `POST /api/sessions/{id}/selected-text-chat` (selected-text query)
- Frontend calls: `GET /api/sessions/{id}/history` (load chat history)
- Backend persists: sessions, messages, metadata in Neon Postgres

**Structure Decision**: Web application (Option 2) - Docusaurus frontend + FastAPI backend already separated. Integration involves adding React hooks and API client to ChatbotWidget component. No new projects or major restructuring needed.

## Complexity Tracking

**No violations** - Constitution gate passed with no justified complexity required. Structure is minimal and focused.

---

## Phase 0: Research & Unknowns Resolution

**Status**: Starting Phase 0

**Unknowns to Resolve**:

1. **Docusaurus ChatbotWidget Implementation Details**
   - Current implementation location: `docusaurus_textbook/src/components/ChatbotWidget`
   - How is it initialized? What props/state does it accept?
   - Can we extend it via props or must we modify the component directly?
   - Research task: Examine existing ChatbotWidget to understand hooks, state management, API integration patterns

2. **Frontend-Backend CORS Configuration**
   - Is CORS already configured in FastAPI backend (backend/api/main.py)?
   - What origin header is expected? (http://localhost:3000 for dev, actual domain for prod)
   - Research task: Verify CORS middleware in backend and determine required Access-Control-Allow-Origin

3. **Session ID Persistence Strategy**
   - localStorage vs URL params: which to prefer?
   - How to restore session ID on page refresh?
   - Research task: Determine best practice for Docusaurus (server-rendered static site vs client-heavy)

4. **Right-Click Context Menu Implementation**
   - Standard browser API for context menu (mouseup event + custom menu)?
   - Or use a library like `react-contextmenu`?
   - Research task: Evaluate libraries and standard approaches for React context menus

5. **Error Handling & User Feedback**
   - What loading indicators currently exist in ChatbotWidget?
   - How to display error messages? (toast notifications, inline messages?)
   - Research task: Identify existing UI patterns and notification library used in Docusaurus

**Phase 0 Output**: `research.md` (to be created) with findings, technology choices, and rationale.

---

## Phase 1: Design & API Contracts

**Prerequisites**: Phase 0 research.md complete

### 1. Data Model (data-model.md)

Will define:
- **Session Entity**: id (UUID), created_at, updated_at, metadata (JSON)
- **Message Entity**: id (UUID), session_id (FK), role (user|assistant), content, mode (whole_book|selected_text), created_at, metadata (JSON)
- **SelectedTextMetadata Entity**: id (UUID), message_id (FK), selected_text, chapter_origin, created_at
- **Relationships & Constraints**: Foreign keys, unique constraints, default values
- **Frontend State**: Session state shape, chat message array structure, loading state

### 2. API Contracts (contracts/)

Will specify OpenAPI/Swagger specs for:
- **POST /api/sessions** - Create new session
  - Request: `{ metadata?: object }`
  - Response: `{ session_id: string, created_at: string, updated_at: string }`

- **POST /api/sessions/{session_id}/chat** - Whole-book query
  - Request: `{ query: string }`
  - Response: `{ answer: string, sources: [{ chapter, score, text }], metadata: {...} }`

- **POST /api/sessions/{session_id}/selected-text-chat** - Selected-text query
  - Request: `{ query: string, selected_text: string, chapter_origin?: string }`
  - Response: `{ answer: string, sources: null, metadata: {...} }`

- **GET /api/sessions/{session_id}/history** - Load chat history
  - Response: `{ messages: [{ id, role, content, mode, created_at, metadata }] }`

### 3. Quickstart (quickstart.md)

Will provide:
- Local development setup (running Docusaurus + backend side-by-side)
- Environment variable configuration (REACT_APP_BACKEND_URL)
- Testing ChatbotWidget integration (manual + unit tests)
- Deployment checklist

### 4. Agent Context Update

Run: `.specify/scripts/bash/update-agent-context.sh claude`
- Adds technology stack to agent context (React, Docusaurus 3.x, FastAPI integration)
- Updates templates for tasks generation

**Phase 1 Output**: data-model.md, contracts/*, quickstart.md, updated agent context

---

## Phase 2: Task Breakdown (Next: /sp.tasks)

After Phase 1 completes, run `/sp.tasks` to generate:
- Actionable task list (T-001 to T-N)
- Task dependencies and parallelization strategy
- Effort estimates (small/medium/large)
- Acceptance criteria for each task

---

## Key Decisions & Rationale

| Decision | Choice | Why | Alternatives Considered |
|----------|--------|-----|------------------------|
| Component approach | Extend existing ChatbotWidget | Non-breaking, reuses existing UI | Recreate from scratch (violates spec) |
| Session persistence | localStorage + URL params | Works for static site, survives refreshes | Server-side sessions (requires auth, extra infra) |
| Selected-text affordance | Right-click context menu | Browser standard, non-intrusive | Inline button (messy UX), floating toolbar (distracting) |
| Backend URL config | Environment variable `.env.local` | Standard React practice, flexible across environments | Hardcoded (inflexible), API endpoint (adds complexity) |
| API style | REST (existing FastAPI) | Matches backend, simpler than GraphQL | GraphQL (overkill for this scope) |

---

## Risk Analysis

| Risk | Severity | Mitigation |
|------|----------|-----------|
| ChatbotWidget API changes in Docusaurus | Medium | Keep changes minimal, extend via props, monitor Docusaurus releases |
| CORS misconfiguration | High | Verify backend CORS middleware, test with curl/Postman before UI integration |
| Session ID loss on hard refresh | Low | Implement recovery flow (create new session if ID invalid) |
| Backend down during demo | Medium | Implement graceful error UI ("Service unavailable. Please try again later") |
| Network latency >15s | Low | Implement timeout warning at 12s, allow user to cancel |

---

## Success Criteria (Phase Completion)

- ✅ Phase 0: research.md complete with all 5 unknowns resolved
- ✅ Phase 1: data-model.md, contracts/*, quickstart.md created and reviewed
- ✅ Phase 1: Constitution check re-validated (no new violations)
- ✅ Phase 2: Task breakdown generated and ready for implementation
- ✅ Overall: Plan is actionable, unambiguous, and approved by user
