# Quickstart: RAG Agent Backend

**Feature**: 007-agent-backend
**Created**: 2025-12-20

## Overview

This guide walks you through setting up and using the RAG Agent Backend locally. You'll learn how to:
1. Set up the development environment
2. Configure API keys and database connection
3. Run the FastAPI backend
4. Make API requests to chat with the agent

**Prerequisites**:
- Python 3.11 or higher
- Neon Serverless Postgres database (or local Postgres)
- Gemini API key (from Google AI Studio)
- Qdrant Cloud instance (reused from feature 005/006)
- Cohere API key (reused from feature 005/006)

---

## 1. Environment Setup

### Clone and Navigate to Backend

```bash
cd /path/to/ai_native-textbook/backend
```

### Install Dependencies

```bash
# Create virtual environment (if not exists)
python3.11 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

**Expected dependencies** (will be added in feature 007):
```
fastapi==0.109.0
uvicorn[standard]==0.27.0
pydantic==2.5.0
google-generativeai==0.3.2
asyncpg==0.29.0
qdrant-client==1.7.0  # Existing from feature 006
cohere==4.37  # Existing from feature 006
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2
```

---

## 2. Configuration

### Create `.env` File

```bash
cp .env.example .env
```

### Edit `.env` with Required API Keys

```bash
# Gemini API (Google AI)
GEMINI_API_KEY=your-gemini-api-key-here

# Neon Serverless Postgres
DATABASE_URL=postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/dbname?sslmode=require

# Qdrant Cloud (reused from feature 006)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Cohere API (reused from feature 006)
COHERE_API_KEY=your-cohere-api-key

# Collection name (reused from feature 005/006)
QDRANT_COLLECTION_NAME=ai-native-textbook
```

**How to obtain API keys**:
- **Gemini**: Visit [Google AI Studio](https://aistudio.google.com/app/apikey) and create an API key
- **Neon Postgres**: Create a free account at [Neon](https://neon.tech) and create a database
- **Qdrant**: Reuse existing cluster from feature 005/006
- **Cohere**: Reuse existing API key from feature 005/006

---

## 3. Database Setup

### Run Schema Migrations

```bash
# Initialize database schema
python -m backend.db.migrate
```

This will create the following tables:
- `sessions`: User chat sessions
- `messages`: Chat messages (questions and answers)
- `selected_text_metadata`: Selected text used in queries

**Manual Schema Setup** (if migrate script not available):

Connect to your Neon Postgres database and run:

```sql
-- Create sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_sessions_created_at ON sessions(created_at DESC);

-- Create messages table
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

-- Create selected_text_metadata table
CREATE TABLE selected_text_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    selected_text TEXT NOT NULL,
    chapter_origin VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_selected_text_message_id ON selected_text_metadata(message_id);
```

---

## 4. Run the Backend

### Start FastAPI Server

```bash
# From backend/ directory
uvicorn api.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using watchfiles
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Verify API is running**:
```bash
curl http://localhost:8000/health
# Expected: {"status": "healthy"}
```

---

## 5. Usage Examples

### Example 1: Create a Session

```bash
curl -X POST http://localhost:8000/api/sessions \
  -H "Content-Type: application/json" \
  -d '{
    "metadata": {
      "source": "quickstart"
    }
  }'
```

**Expected response**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "created_at": "2025-12-20T14:30:00Z",
  "updated_at": "2025-12-20T14:30:00Z",
  "metadata": {
    "source": "quickstart"
  }
}
```

**Save the session ID** for subsequent requests.

---

### Example 2: Whole-Book Query

Ask a question using whole-book context (Qdrant retrieval):

```bash
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -X POST http://localhost:8000/api/sessions/$SESSION_ID/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Isaac Sim and how is it used for robotics simulation?",
    "mode": "whole_book"
  }'
```

**Expected response** (abbreviated):
```json
{
  "message": {
    "id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "role": "assistant",
    "content": "Isaac Sim is NVIDIA's photorealistic robotics simulation platform built on Omniverse. It provides physically accurate simulations for robot navigation, manipulation, and perception tasks. Key features include...",
    "created_at": "2025-12-20T14:31:08Z",
    "mode": "whole_book",
    "metadata": {
      "latency_ms": 8234,
      "retrieval_count": 5,
      "top_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation"
    }
  },
  "sources": [
    {
      "chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
      "relevance_score": 0.89,
      "text_preview": "Isaac Sim is NVIDIA's photorealistic robotics simulation platform..."
    }
  ]
}
```

---

### Example 3: Selected-Text-Only Query

Ask a question using ONLY user-selected text (no Qdrant retrieval):

```bash
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -X POST http://localhost:8000/api/sessions/$SESSION_ID/selected-text-chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is URDF?",
    "selected_text": "URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines links, joints, sensors, and visual/collision geometries.",
    "chapter_origin": "Chapter 4: URDF and Sensor Simulation",
    "mode": "selected_text"
  }'
```

**Expected response** (abbreviated):
```json
{
  "message": {
    "id": "b2c3d4e5-f6a7-4b5c-9d0e-1f2a3b4c5d6e",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "role": "assistant",
    "content": "Based on your selected text: URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the structure of the robot including links (rigid bodies), joints (connections between links), sensors, and visual/collision geometries.",
    "created_at": "2025-12-20T14:35:03Z",
    "mode": "selected_text",
    "metadata": {
      "latency_ms": 3421,
      "selected_text_length": 134
    }
  },
  "sources": null
}
```

**Note**: `sources` is always `null` for selected-text mode (no external retrieval).

---

### Example 4: Retrieve Chat History

```bash
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -X GET http://localhost:8000/api/sessions/$SESSION_ID/history
```

**Expected response**:
```json
{
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "messages": [
    {
      "id": "f47ac10b-58cc-4372-a567-0e02b2c3d479",
      "session_id": "550e8400-e29b-41d4-a716-446655440000",
      "role": "user",
      "content": "What is Isaac Sim?",
      "created_at": "2025-12-20T14:31:00Z",
      "mode": "whole_book",
      "metadata": {}
    },
    {
      "id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
      "session_id": "550e8400-e29b-41d4-a716-446655440000",
      "role": "assistant",
      "content": "Isaac Sim is NVIDIA's photorealistic robotics simulation platform...",
      "created_at": "2025-12-20T14:31:08Z",
      "mode": "whole_book",
      "metadata": {
        "latency_ms": 8234,
        "retrieval_count": 5
      }
    }
  ]
}
```

---

## 6. Testing

### Run Unit Tests

```bash
# From backend/ directory
pytest tests/agent -v
pytest tests/api -v
pytest tests/db -v
```

**Expected output**:
```
tests/agent/test_whole_book_mode.py::test_whole_book_retrieval PASSED
tests/agent/test_selected_text_mode.py::test_selected_text_isolation PASSED
tests/api/test_sessions.py::test_create_session PASSED
tests/api/test_chat.py::test_whole_book_chat PASSED
tests/db/test_repositories.py::test_session_crud PASSED
...
```

### Run Integration Tests

```bash
pytest tests/ -v --integration
```

---

## 7. Deployment

### Deploy to Render Free Tier

1. **Create a new Web Service** on Render
2. **Connect GitHub repository**
3. **Configure build settings**:
   - Build Command: `pip install -r backend/requirements.txt`
   - Start Command: `uvicorn api.main:app --host 0.0.0.0 --port $PORT`
   - Root Directory: `backend/`
4. **Add environment variables**:
   - `GEMINI_API_KEY`
   - `DATABASE_URL` (Neon Postgres connection string)
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_COLLECTION_NAME`
5. **Deploy**

**Expected URL**: `https://your-app.onrender.com`

---

## 8. Troubleshooting

### Issue: "Gemini API key not found"
**Solution**: Verify `.env` file contains `GEMINI_API_KEY=...` and restart the server.

### Issue: "Database connection failed"
**Solution**: Check `DATABASE_URL` format and verify Neon Postgres is accessible from your IP.

### Issue: "Qdrant collection not found"
**Solution**: Ensure feature 005/006 Qdrant collection (`ai-native-textbook`) exists with 36+ vectors.

### Issue: "Selected text mode returns external knowledge"
**Solution**: Verify system prompts enforce strict local-context-only mode. Check `backend/agent/prompts.py`.

---

## 9. API Reference

For detailed API documentation, see:
- **Data Models**: `data-model.md`
- **Endpoint Specifications**: `contracts/api-endpoints.md`

---

## Next Steps

1. **Integrate with Docusaurus frontend**: Add chat widget that calls these API endpoints
2. **Add authentication**: Implement session-based auth with user accounts
3. **Enable streaming responses**: Use Server-Sent Events (SSE) for real-time agent responses
4. **Add analytics**: Track query patterns, latency, and user engagement

---

**Support**: For issues or questions, see project documentation or open a GitHub issue.
