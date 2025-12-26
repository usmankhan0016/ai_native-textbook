---
title: Physical AI Textbook RAG Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Textbook - RAG Backend

Production-ready RAG chatbot backend with dual-mode querying for the Physical AI textbook.

## Features

- **Dual-Mode Querying**: Whole-book and selected-text modes
- **Session Persistence**: PostgreSQL-backed conversation history
- **Vector Search**: Qdrant Cloud for semantic search
- **LLM Integration**: Google Gemini API for response generation
- **Production-Ready**: Error handling, timeouts, and CORS support

## API Endpoints

### Health Check
```
GET /health
```
Returns: `{"status": "healthy", "service": "rag-agent-backend"}`

### Create Session
```
POST /api/sessions
```
Request: `{}`
Response: `{"session_id": "uuid", "created_at": "timestamp"}`

### Send Message
```
POST /api/chat
```
Request:
```json
{
  "session_id": "uuid",
  "message": "Your question here",
  "mode": "whole_book" | "selected_text",
  "context": "optional selected text"
}
```

## Environment Variables

Set these in your Hugging Face Space settings (Settings → Variables and Secrets):

| Variable | Required | Description |
|----------|----------|-------------|
| `GEMINI_API_KEY` | Yes | Google Gemini API key |
| `COHERE_API_KEY` | Yes | Cohere API key for embeddings |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | Qdrant API key |
| `DATABASE_URL` | Yes | PostgreSQL connection string |

### Getting API Keys

**Google Gemini API:**
1. Go to https://makersuite.google.com/app/apikeys
2. Create new API key
3. Copy the key

**Cohere API:**
1. Go to https://dashboard.cohere.com
2. Sign up/Login
3. Create new API key
4. Copy the key

**Qdrant Cloud:**
1. Go to https://cloud.qdrant.io
2. Create cluster
3. Copy cluster URL and API key

**PostgreSQL Database:**
- **Neon.tech** (Recommended): https://neon.tech
- **Supabase**: https://supabase.com
- **ElephantSQL**: https://www.elephantsql.com

## Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Set up environment
cp .env.example .env
# Edit .env with your API keys and database URL

# Run server
uvicorn api.main:app --reload
```

Server runs on `http://localhost:8000`

## Testing the API

```bash
# Health check
curl https://YOUR_SPACE_URL/health

# Create session
curl -X POST https://YOUR_SPACE_URL/api/sessions

# Send message
curl -X POST https://YOUR_SPACE_URL/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-id",
    "message": "What is ROS 2?",
    "mode": "whole_book"
  }'
```

## Architecture

**Backend Stack:**
- FastAPI + Uvicorn
- Google Generative AI (Gemini)
- Cohere embeddings API
- Qdrant Cloud vector database
- PostgreSQL for session persistence
- asyncpg for async DB operations

**Features:**
- Async request handling
- Dual-mode querying (whole-book and selected-text)
- Session management with history
- CORS support for frontend integration
- Comprehensive error handling
- Request logging

## Deployment

This Space automatically deploys on push. The FastAPI backend runs on port 7860.

## Support

For issues or questions:
1. Check Hugging Face Space logs (Logs tab)
2. Verify environment variables are set correctly
3. Test API endpoints with health check
4. Review error messages in logs

---

**Status**: Production Ready 🚀
