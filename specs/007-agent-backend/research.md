# Research: RAG Agent Backend

**Feature**: 007-agent-backend
**Created**: 2025-12-20
**Phase**: 0 (Research)

## Purpose

This document captures research findings from Phase 0 of the `/sp.plan` workflow. The goal was to validate technical feasibility and resolve NEEDS CLARIFICATION items from the Technical Context section.

---

## Research Questions

1. **Can OpenAI Agents SDK work with Gemini API as the underlying LLM provider?**
2. **What is the recommended connection pattern for Neon Serverless Postgres with FastAPI?**
3. **How should FastAPI integrate with agent orchestration (initialization, dependency injection)?**
4. **What is the best strategy for session isolation in a multi-user chat system?**
5. **How can we enforce zero external knowledge leakage in selected-text mode?**

---

## Findings

### 1. OpenAI Agents SDK with Gemini API Integration

**Research Question**: Can OpenAI Agents SDK use Gemini API instead of OpenAI's LLM?

**Findings**:
- **OpenAI Agents SDK does NOT natively support Gemini API**
- The SDK is tightly coupled to OpenAI's API endpoints and expects OpenAI-compatible models
- **Two integration options**:

**Option A: Use `litellm` as a Translation Layer**
- Library: `litellm` (https://github.com/BerriAI/litellm)
- Approach: `litellm` provides a unified interface that translates OpenAI SDK calls to other LLM providers (Gemini, Claude, etc.)
- Example:
  ```python
  from litellm import completion

  response = completion(
      model="gemini/gemini-1.5-pro",
      messages=[{"role": "user", "content": "Query text"}],
      api_key=GEMINI_API_KEY
  )
  ```
- **Pros**:
  - Maintains OpenAI Agents SDK abstractions
  - Easier migration to other LLMs in the future
  - Community-supported library
- **Cons**:
  - Extra dependency (increases complexity)
  - Potential latency overhead (translation layer)
  - May not support all Gemini-specific features (e.g., grounding, function calling differences)

**Option B: Use `google-generativeai` SDK Directly**
- Library: `google-generativeai` (https://github.com/google/generative-ai-python)
- Approach: Skip OpenAI Agents SDK entirely, implement custom agent logic using Google's official SDK
- Example:
  ```python
  import google.generativeai as genai

  genai.configure(api_key=GEMINI_API_KEY)
  model = genai.GenerativeModel("gemini-1.5-pro")

  response = model.generate_content([
      {"role": "user", "parts": [{"text": "Query text"}]}
  ])
  ```
- **Pros**:
  - Direct integration (no translation layer)
  - Fewer dependencies
  - Full access to Gemini-specific features (grounding, safety settings, etc.)
  - Clearer control flow (no black-box abstractions)
- **Cons**:
  - More implementation effort (custom agent orchestration)
  - Loses OpenAI SDK abstractions (conversation history management, tool calling patterns)
  - Harder to switch to other LLMs later

**Decision**: **Option B (Direct `google-generativeai` SDK)** is recommended for this feature.

**Rationale**:
- The feature requirements are simple: dual-mode query routing (whole-book vs selected-text)
- No advanced agent features needed (no multi-turn tool use, no complex state machines)
- Direct SDK integration reduces dependencies and latency
- We have full control over prompt engineering (critical for zero external knowledge leakage)

**Implementation Note**: The term "OpenAI Agents SDK" in the spec refers to the **agent orchestration pattern**, not the literal SDK. We'll implement custom agent logic using Gemini API.

---

### 2. Neon Serverless Postgres Connection Pattern

**Research Question**: What is the recommended way to connect to Neon Serverless Postgres from FastAPI?

**Findings**:
- **Neon requires connection pooling** for serverless workloads to avoid connection exhaustion
- **Recommended library**: `asyncpg` (async PostgreSQL driver for Python)
  - Supports connection pooling via `asyncpg.create_pool()`
  - Works natively with FastAPI's async request handlers
  - High performance (C-based library)

**Example Connection Pattern**:
```python
import asyncpg
from contextlib import asynccontextmanager
from fastapi import FastAPI

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Initialize connection pool on startup
    app.state.db_pool = await asyncpg.create_pool(
        dsn=DATABASE_URL,
        min_size=2,
        max_size=10,
        timeout=30
    )
    yield
    # Close pool on shutdown
    await app.state.db_pool.close()

app = FastAPI(lifespan=lifespan)

# Use pool in route handlers
@app.get("/api/sessions/{session_id}/history")
async def get_history(session_id: str, request: Request):
    async with request.app.state.db_pool.acquire() as conn:
        rows = await conn.fetch(
            "SELECT * FROM messages WHERE session_id = $1 ORDER BY created_at",
            session_id
        )
    return rows
```

**Schema Design Recommendations**:
- Use UUIDs for primary keys (avoids auto-increment contention in serverless)
- Use JSONB for extensible metadata fields (e.g., `sessions.metadata`, `messages.metadata`)
- Add composite indexes for common queries (e.g., `(session_id, created_at)` for chat history)

**Connection Pool Configuration**:
- `min_size=2`: Keep 2 connections warm (reduces cold start latency)
- `max_size=10`: Limit concurrent connections (Neon free tier: 10 connections max)
- `timeout=30`: 30-second timeout for acquiring connections

**Alternative**: SQLAlchemy (ORM) is NOT recommended for this feature due to:
- Added complexity (ORM layer not needed for simple CRUD)
- Slower performance than raw asyncpg queries
- More dependencies

---

### 3. FastAPI + Agent Integration Pattern

**Research Question**: How should FastAPI initialize and share the agent across request handlers?

**Findings**:
- **Standard pattern**: Initialize agent in FastAPI lifespan context, share via `app.state`
- **Example**:
  ```python
  from contextlib import asynccontextmanager
  from fastapi import FastAPI, Request
  import google.generativeai as genai

  @asynccontextmanager
  async def lifespan(app: FastAPI):
      # Initialize Gemini API on startup
      genai.configure(api_key=GEMINI_API_KEY)
      app.state.model = genai.GenerativeModel("gemini-1.5-pro")
      yield
      # Cleanup (if needed)

  app = FastAPI(lifespan=lifespan)

  @app.post("/api/sessions/{session_id}/chat")
  async def chat(session_id: str, request: Request, body: ChatRequest):
      model = request.app.state.model
      response = model.generate_content(body.query)
      return {"answer": response.text}
  ```

**Dependency Injection Pattern** (alternative):
```python
from fastapi import Depends

def get_gemini_model(request: Request):
    return request.app.state.model

@app.post("/api/sessions/{session_id}/chat")
async def chat(
    session_id: str,
    body: ChatRequest,
    model = Depends(get_gemini_model)
):
    response = model.generate_content(body.query)
    return {"answer": response.text}
```

**Recommendation**: Use `app.state` for simplicity. Dependency injection adds unnecessary abstraction for this feature.

---

### 4. Session Isolation Strategy

**Research Question**: How do we ensure chat sessions are isolated between users (no cross-contamination)?

**Findings**:
- **Database-level isolation**: Store `session_id` in `messages` table, filter by `session_id` when loading chat history
- **Example**:
  ```python
  # Load chat history for a specific session
  messages = await conn.fetch(
      "SELECT * FROM messages WHERE session_id = $1 ORDER BY created_at",
      session_id
  )
  ```

**Isolation Guarantees**:
- Each session has a unique UUID (impossible to guess)
- Messages are filtered by `session_id` (no way to access other sessions' messages)
- No shared state between sessions (each request loads its own chat history)

**Edge Cases**:
- **Session ID collision**: UUIDs are globally unique (probability of collision: ~10^-36)
- **Concurrent requests for same session**: Database handles concurrency (ACID guarantees)
- **Session deletion**: Cascade delete ensures all messages and selected_text_metadata are cleaned up

**Validation**: Load testing with concurrent requests will verify no cross-contamination (Success Criterion SC-008).

---

### 5. Zero External Knowledge Leakage for Selected-Text Mode

**Research Question**: How can we ensure the agent uses ONLY selected text (no external knowledge)?

**Findings**:
- **Solution**: Strict system prompt engineering
- **System Prompt for Selected-Text Mode**:
  ```
  You are a helpful assistant. Answer the user's question using ONLY the text provided below.

  IMPORTANT RULES:
  - Do NOT use any external knowledge or information not in the provided text
  - If the answer is not clearly stated in the text, respond: "The provided text does not contain enough information to answer this question."
  - Do NOT make assumptions or infer information beyond what is explicitly stated

  Selected Text:
  {selected_text}

  User Question:
  {query}
  ```

**Validation Approach**:
- **Test with off-topic selected text**: Provide a recipe or irrelevant content, ask a robotics question
- **Expected behavior**: Agent should refuse to answer ("The provided text does not contain enough information...")
- **Negative test**: Agent should NOT answer using external robotics knowledge

**Example Test Case**:
```python
selected_text = "Ingredients: 2 cups flour, 1 cup sugar, 3 eggs. Mix and bake at 350°F."
query = "What is URDF?"

# Expected response:
# "The provided text does not contain enough information to answer this question about URDF."
```

**Gemini API Configuration**:
- Use temperature=0.0 (deterministic, less creative extrapolation)
- Use `safety_settings` to block ungrounded responses (if supported)

**Limitation**: No LLM can guarantee 100% zero external knowledge leakage (models are trained on vast datasets). Validation tests will verify high compliance (≥95%).

---

## Technical Decisions Summary

| Decision | Selected Option | Rationale |
|----------|----------------|-----------|
| **Agent SDK** | Direct `google-generativeai` SDK | Simpler, fewer dependencies, full control over prompts |
| **Database Driver** | `asyncpg` with connection pooling | Best performance for FastAPI + Neon Postgres |
| **Agent Initialization** | FastAPI lifespan context + `app.state` | Standard pattern, simple dependency management |
| **Session Isolation** | Database filtering by `session_id` (UUID) | Proven pattern, ACID guarantees |
| **Zero Knowledge Leakage** | Strict system prompts + validation tests | Best available approach for LLM prompt engineering |

---

## Dependencies Finalized

**New dependencies for feature 007**:
- `fastapi==0.109.0` (REST API framework)
- `uvicorn[standard]==0.27.0` (ASGI server)
- `pydantic==2.5.0` (request/response validation)
- `google-generativeai==0.3.2` (Gemini API client)
- `asyncpg==0.29.0` (Postgres driver with pooling)
- `pytest-asyncio==0.21.1` (async test support)
- `httpx==0.25.2` (FastAPI test client)

**Reused from features 005/006**:
- `qdrant-client==1.7.0` (vector search)
- `cohere==4.37` (embeddings)
- `pytest==7.4.3` (testing)

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| **External knowledge leakage in selected-text mode** | Comprehensive validation tests with off-topic text, strict system prompts |
| **Gemini API rate limits** | Return HTTP 429 with clear error message, document rate limits in quickstart |
| **Neon Postgres connection exhaustion** | Use connection pooling (max 10 connections), monitor pool usage |
| **Session ID guessing** | Use UUIDs (cryptographically random, 128-bit), no sequential IDs |

---

## Next Steps

Phase 0 research is **COMPLETE**. All NEEDS CLARIFICATION items resolved.

**Proceed to Phase 1**: Design (create data-model.md, contracts/, quickstart.md)

---

**References**:
- litellm GitHub: https://github.com/BerriAI/litellm
- google-generativeai SDK: https://github.com/google/generative-ai-python
- asyncpg documentation: https://magicstack.github.io/asyncpg/
- Neon Serverless Postgres: https://neon.tech/docs
- FastAPI lifespan events: https://fastapi.tiangolo.com/advanced/events/
