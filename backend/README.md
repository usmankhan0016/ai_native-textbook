---
title: Physical AI RAG Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_file: api/main.py
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Textbook - RAG Backend

Production-ready RAG chatbot backend with dual-mode querying for the Physical AI textbook.

## Features

- **Dual-Mode Querying**: Whole-book and selected-text modes
- **Session Persistence**: PostgreSQL-backed conversation history
- **Vector Search**: Qdrant Cloud for semantic search with Cohere embeddings
- **LLM Integration**: Google Gemini API for intelligent response generation
- **Production-Ready**: Comprehensive error handling, timeouts, and CORS support
- **FastAPI**: Modern async Python web framework with automatic API documentation

## Quick Start

### Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Run server
uvicorn api.main:app --reload --port 8000
```

Server runs on `http://localhost:8000`

### API Endpoints

#### Health Check
```bash
GET /health
```
Response: `{"status": "healthy", "service": "rag-agent-backend"}`

#### Create Session
```bash
POST /api/sessions
Body: {}
Response: {"session_id": "uuid", "created_at": "timestamp"}
```

#### Send Message
```bash
POST /api/chat
Body: {
  "session_id": "uuid",
  "message": "Your question",
  "mode": "whole_book" | "selected_text",
  "context": "optional selected text"
}
```

## Environment Variables

### Required for Deployment

| Variable | Description | Source |
|----------|-------------|--------|
| `GEMINI_API_KEY` | Google Gemini API key | https://makersuite.google.com/app/apikeys |
| `COHERE_API_KEY` | Cohere embeddings API key | https://dashboard.cohere.com |
| `QDRANT_URL` | Qdrant Cloud cluster URL | https://cloud.qdrant.io |
| `QDRANT_API_KEY` | Qdrant API key | https://cloud.qdrant.io |
| `DATABASE_URL` | PostgreSQL connection string | Neon.tech / Supabase / ElephantSQL |

### Optional

| Variable | Default | Purpose |
|----------|---------|---------|
| `LOG_LEVEL` | INFO | Logging level (DEBUG, INFO, WARNING, ERROR) |
| `CORS_ORIGINS` | localhost:3000, GitHub Pages | CORS allowed origins |

## Getting API Keys

### Google Gemini API
1. Visit https://makersuite.google.com/app/apikeys
2. Click "Create API Key"
3. Copy the key

### Cohere API
1. Visit https://dashboard.cohere.com
2. Sign up/Login
3. Navigate to API keys
4. Create new API key

### Qdrant Cloud
1. Visit https://cloud.qdrant.io
2. Create a new cluster
3. Copy cluster URL and API key

### PostgreSQL Database (Free Options)
- **Neon.tech** (Recommended): https://neon.tech - Serverless Postgres
- **Supabase**: https://supabase.com - Open-source Firebase alternative
- **ElephantSQL**: https://www.elephantsql.com - Managed PostgreSQL

## Architecture

### Technology Stack
- **Framework**: FastAPI + Uvicorn
- **LLM**: Google Generative AI (Gemini)
- **Embeddings**: Cohere API
- **Vector DB**: Qdrant Cloud
- **Session Storage**: PostgreSQL
- **Async**: asyncpg, httpx

### Key Components

**`api/main.py`** - FastAPI application entry point
- Health check endpoint
- CORS middleware configuration
- Error handling and logging
- Lifespan management

**`api/routes/sessions.py`** - Session management
- Create new chat sessions
- Retrieve session history
- Clean up expired sessions

**`api/routes/chat.py`** - Chat messaging
- Process user messages
- Dual-mode query handling
- Response generation with Gemini
- Session persistence

**`agent/`** - RAG agent logic
- Content retrieval from Qdrant
- Query mode selection
- Prompt engineering
- Response synthesis

**`db/`** - Database layer
- Connection pooling
- Schema management
- Repository pattern
- Async operations

**`retrieval/`** - RAG retrieval logic
- Vector search
- Metadata filtering
- Result ranking
- Content formatting

## Deployment

### Hugging Face Spaces (Recommended)

This Space uses Docker for deployment. The FastAPI backend automatically:
1. Builds from `Dockerfile`
2. Installs dependencies from `requirements.txt`
3. Runs on port 7860
4. Serves the API with automatic documentation at `/docs`

**Setup Steps:**
1. Create environment variables in Space Settings
2. Space auto-builds and deploys
3. Monitor build progress in Logs tab
4. Test with health endpoint

### Docker Local Testing

```bash
# Build image
docker build -t rag-backend .

# Run container
docker run -p 7860:7860 \
  -e GEMINI_API_KEY=your_key \
  -e COHERE_API_KEY=your_key \
  -e QDRANT_URL=your_url \
  -e QDRANT_API_KEY=your_key \
  -e DATABASE_URL=your_db_url \
  rag-backend
```

## Testing the API

### Health Check
```bash
curl https://Usmankhan0016-textbook-rag-backend.hf.space/health
```

### Create Session
```bash
curl -X POST https://Usmankhan0016-textbook-rag-backend.hf.space/api/sessions
```

### Send Message (Whole-Book Mode)
```bash
curl -X POST https://Usmankhan0016-textbook-rag-backend.hf.space/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-id",
    "message": "What is ROS 2?",
    "mode": "whole_book"
  }'
```

### Send Message (Selected-Text Mode)
```bash
curl -X POST https://Usmankhan0016-textbook-rag-backend.hf.space/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-id",
    "message": "Explain this",
    "mode": "selected_text",
    "context": "The selected text from the book..."
  }'
```

## Error Handling

The API returns structured error responses:

```json
{
  "error": {
    "status_code": 400,
    "message": "Descriptive error message",
    "path": "/api/chat"
  }
}
```

### Common Status Codes
- `200` - Success
- `400` - Bad request (validation error)
- `404` - Session not found
- `429` - Rate limited
- `500` - Server error

## Troubleshooting

### Build Failures
- Check Logs tab in Space
- Verify all environment variables are set
- Ensure requirements.txt is valid

### Connection Errors
- Verify Qdrant URL and API key are correct
- Check PostgreSQL connection string format
- Ensure CORS origins include your frontend domain

### Empty Responses
- Verify GEMINI_API_KEY is set and valid
- Check Qdrant collection is populated with embeddings
- Review agent logs for query processing

### Timeouts
- Whole-book queries may take 10-15 seconds
- Selected-text queries: 3-5 seconds
- Check backend logs for stuck processes

## Development

### Local Setup
```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run with auto-reload
uvicorn api.main:app --reload
```

### Code Structure
```
backend/
├── api/                    # FastAPI application
│   ├── main.py            # App entry point
│   ├── models.py          # Request/response models
│   └── routes/            # API endpoints
├── agent/                  # RAG agent logic
├── db/                     # Database layer
├── retrieval/             # Vector search
├── requirements.txt       # Python dependencies
└── Dockerfile            # Container config
```

## Performance

- **Startup time**: ~5-10 seconds
- **Whole-book query**: 10-15 seconds average
- **Selected-text query**: 3-5 seconds average
- **Concurrent users**: ~10-20 (free Hugging Face tier)

## Security

- All API keys stored in HF Space secrets (encrypted)
- No sensitive data in logs
- CORS configured for specific origins only
- Request validation via Pydantic
- SQL injection protection via asyncpg

## Support & Documentation

- **API Docs**: https://your-space-url/docs (Swagger UI)
- **Alternative Docs**: https://your-space-url/redoc (ReDoc)
- **HF Spaces Docs**: https://huggingface.co/docs/hub/spaces
- **FastAPI Docs**: https://fastapi.tiangolo.com

## License

MIT License - See LICENSE file

---

**Status**: Production Ready 🚀

Built for the Physical AI & Humanoid Robotics Textbook with ❤️
