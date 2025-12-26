# Data Model: RAG Agent Backend

**Feature**: 007-agent-backend
**Created**: 2025-12-20
**Status**: Draft

## Overview

This document defines the database schema, entity relationships, and API data models for the RAG Agent Backend. The system uses Neon Serverless Postgres for persistent storage and Pydantic models for API request/response validation.

---

## Database Schema

### Table: `sessions`

Stores user chat sessions with metadata.

```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_sessions_created_at ON sessions(created_at DESC);
```

**Columns**:
- `id`: Unique session identifier (UUID v4)
- `created_at`: Session creation timestamp
- `updated_at`: Last activity timestamp (updated on each new message)
- `metadata`: Extensible JSON field for future session attributes (e.g., user_agent, ip_address)

**Rationale**: UUID primary keys ensure global uniqueness without coordination. JSONB metadata field allows future extension without schema migrations.

---

### Table: `messages`

Stores chat messages (questions and agent responses) for each session.

```sql
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    mode VARCHAR(20) NOT NULL CHECK (mode IN ('whole_book', 'selected_text')),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_messages_session_id ON messages(session_id, created_at ASC);
CREATE INDEX idx_messages_created_at ON messages(created_at DESC);
```

**Columns**:
- `id`: Unique message identifier
- `session_id`: Foreign key to sessions table (CASCADE delete ensures cleanup)
- `role`: Message author (`user` = question, `assistant` = agent response)
- `content`: Message text (questions or answers)
- `created_at`: Message timestamp
- `mode`: Query mode (`whole_book` = Qdrant retrieval, `selected_text` = user-selected text only)
- `metadata`: Extensible JSON field (e.g., latency_ms, model_version, retrieval_count)

**Rationale**:
- Separate messages table enables efficient pagination of chat history
- `mode` field distinguishes between whole-book and selected-text queries for analytics
- Composite index on `(session_id, created_at)` optimizes chat history retrieval

---

### Table: `selected_text_metadata`

Captures selected text used in queries for audit and debugging.

```sql
CREATE TABLE selected_text_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    selected_text TEXT NOT NULL,
    chapter_origin VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_selected_text_message_id ON selected_text_metadata(message_id);
```

**Columns**:
- `id`: Unique record identifier
- `message_id`: Foreign key to messages table (links selected text to specific query)
- `selected_text`: Full text selected by user
- `chapter_origin`: Optional chapter name if known (e.g., "Chapter 2: Isaac Sim")
- `created_at`: Timestamp

**Rationale**:
- Separate table isolates selected-text data (not needed for whole-book queries)
- Allows debugging and analysis of selected-text mode usage
- `chapter_origin` enables tracking which chapters users query most frequently

---

### Design Decision: RetrievalResult Persistence

**Question**: Should we persist Qdrant retrieval results in the database?

**Decision**: **No - compute on-demand**

**Rationale**:
- Retrieval results are deterministic (same query → same results from Qdrant)
- Storing results duplicates data already in Qdrant (storage waste)
- Results can be regenerated for debugging via `message.metadata` (stores query parameters)
- Exception: Store retrieval count and top chapter name in `messages.metadata` for analytics

**Example `messages.metadata` for whole-book queries**:
```json
{
  "latency_ms": 8234,
  "retrieval_count": 5,
  "top_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
  "gemini_model": "gemini-1.5-pro"
}
```

---

## API Data Models (Pydantic)

### Request Models

**CreateSessionRequest**
```python
from pydantic import BaseModel
from typing import Optional, Dict

class CreateSessionRequest(BaseModel):
    """Request to create a new chat session."""
    metadata: Optional[Dict[str, str]] = {}
```

**ChatRequest**
```python
from pydantic import BaseModel, Field
from typing import Literal

class ChatRequest(BaseModel):
    """Request to send a whole-book query."""
    query: str = Field(..., min_length=1, max_length=50000)
    mode: Literal["whole_book"] = "whole_book"
```

**SelectedTextChatRequest**
```python
from pydantic import BaseModel, Field
from typing import Optional

class SelectedTextChatRequest(BaseModel):
    """Request to send a selected-text-only query."""
    query: str = Field(..., min_length=1, max_length=50000)
    selected_text: str = Field(..., min_length=1, max_length=100000)
    chapter_origin: Optional[str] = None
    mode: Literal["selected_text"] = "selected_text"
```

---

### Response Models

**SessionResponse**
```python
from pydantic import BaseModel
from datetime import datetime
from typing import Dict, Optional

class SessionResponse(BaseModel):
    """Response for session creation or retrieval."""
    id: str  # UUID as string
    created_at: datetime
    updated_at: datetime
    metadata: Dict[str, str]
```

**MessageResponse**
```python
from pydantic import BaseModel
from datetime import datetime
from typing import Literal, Dict, Optional

class MessageResponse(BaseModel):
    """Response for a single message."""
    id: str  # UUID as string
    session_id: str
    role: Literal["user", "assistant"]
    content: str
    created_at: datetime
    mode: Literal["whole_book", "selected_text"]
    metadata: Dict[str, any]
```

**ChatResponse**
```python
from pydantic import BaseModel
from typing import List, Optional

class RetrievalSource(BaseModel):
    """Source attribution for whole-book queries."""
    chapter: str
    relevance_score: float
    text_preview: str  # First 200 characters

class ChatResponse(BaseModel):
    """Response for chat queries."""
    message: MessageResponse  # The assistant's answer
    sources: Optional[List[RetrievalSource]] = None  # Only for whole_book mode
```

**ChatHistoryResponse**
```python
from pydantic import BaseModel
from typing import List

class ChatHistoryResponse(BaseModel):
    """Response for retrieving chat history."""
    session_id: str
    messages: List[MessageResponse]
```

---

## Entity Relationships

```
sessions (1) ────── (N) messages
                         │
                         │ (1)
                         │
                         ▼
                    (0..1) selected_text_metadata
```

**Relationships**:
- One session has many messages (1:N)
- One message may have one selected_text_metadata record (1:0..1, only for selected-text mode)
- Cascade deletes: Deleting a session deletes all its messages and selected text metadata

---

## Indexes and Performance

**Query Patterns**:
1. **Retrieve chat history**: `SELECT * FROM messages WHERE session_id = ? ORDER BY created_at ASC`
   - Optimized by: `idx_messages_session_id` (composite index on session_id + created_at)

2. **Check session exists**: `SELECT id FROM sessions WHERE id = ?`
   - Optimized by: Primary key index

3. **Retrieve selected text for message**: `SELECT * FROM selected_text_metadata WHERE message_id = ?`
   - Optimized by: `idx_selected_text_message_id`

**Expected Performance**:
- Chat history retrieval: <50ms for sessions with <1000 messages
- Session lookup: <10ms (primary key lookup)
- Selected text retrieval: <20ms

---

## Migration Strategy

**Initial Schema Deployment**:
1. Run DDL scripts to create tables and indexes
2. Verify foreign key constraints and cascade deletes
3. Test with sample data (10 sessions, 100 messages)

**Future Schema Evolution**:
- Use Alembic or raw SQL migrations for schema changes
- JSONB metadata fields allow non-breaking extensions

---

## Data Retention

**Policy**: Retain all sessions and messages indefinitely (no automatic cleanup for MVP)

**Rationale**:
- MVP expected usage: ~1000 sessions, ~10,000 messages in first month
- At 1KB per message average: ~10MB total storage (negligible for Postgres)
- Future work: Implement session expiry (e.g., delete sessions inactive for >90 days)

---

## Security Considerations

**No PII Storage**: Session IDs are opaque UUIDs with no user identifiers. For MVP, no authentication/authorization is implemented.

**Input Validation**:
- Query length: Max 50,000 characters (prevent abuse)
- Selected text: Max 100,000 characters
- Pydantic models enforce constraints at API layer

**SQL Injection Prevention**: Use parameterized queries via asyncpg (no raw SQL concatenation)

---

**Next**: See `contracts/` for API endpoint specifications and `quickstart.md` for usage examples.
