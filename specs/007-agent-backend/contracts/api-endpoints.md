# API Endpoints: RAG Agent Backend

**Feature**: 007-agent-backend
**Created**: 2025-12-20
**Base URL**: `http://localhost:8000` (development)

## Endpoints Overview

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/sessions` | POST | Create a new chat session |
| `/api/sessions/{session_id}/history` | GET | Retrieve chat history for a session |
| `/api/sessions/{session_id}/chat` | POST | Send a whole-book query |
| `/api/sessions/{session_id}/selected-text-chat` | POST | Send a selected-text-only query |

---

## 1. Create Session

**Endpoint**: `POST /api/sessions`

**Description**: Creates a new chat session and returns a unique session ID. Sessions are used to maintain conversation context across multiple queries.

### Request

**Headers**:
```
Content-Type: application/json
```

**Body** (optional):
```json
{
  "metadata": {
    "user_agent": "Mozilla/5.0",
    "source": "docusaurus"
  }
}
```

**Schema**:
```python
{
  "metadata": Optional[Dict[str, str]]  # Optional extensible metadata
}
```

### Response

**Status Code**: `201 Created`

**Body**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "created_at": "2025-12-20T14:30:00Z",
  "updated_at": "2025-12-20T14:30:00Z",
  "metadata": {
    "user_agent": "Mozilla/5.0",
    "source": "docusaurus"
  }
}
```

**Schema**: See `SessionResponse` in data-model.md

### Error Responses

**400 Bad Request**: Invalid metadata format
```json
{
  "detail": "Metadata must be a dictionary of string key-value pairs"
}
```

**500 Internal Server Error**: Database error
```json
{
  "detail": "Failed to create session"
}
```

---

## 2. Get Chat History

**Endpoint**: `GET /api/sessions/{session_id}/history`

**Description**: Retrieves all messages for a given session, ordered chronologically.

### Request

**Path Parameters**:
- `session_id` (UUID): The session identifier

**Example**:
```
GET /api/sessions/550e8400-e29b-41d4-a716-446655440000/history
```

### Response

**Status Code**: `200 OK`

**Body**:
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
        "retrieval_count": 5,
        "top_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation"
      }
    }
  ]
}
```

**Schema**: See `ChatHistoryResponse` in data-model.md

### Error Responses

**404 Not Found**: Session does not exist
```json
{
  "detail": "Session not found"
}
```

**500 Internal Server Error**: Database error
```json
{
  "detail": "Failed to retrieve chat history"
}
```

---

## 3. Whole-Book Chat

**Endpoint**: `POST /api/sessions/{session_id}/chat`

**Description**: Sends a question to the agent using whole-book context. The agent retrieves relevant chapters from Qdrant and generates an answer grounded in those results.

### Request

**Path Parameters**:
- `session_id` (UUID): The session identifier

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "query": "What is Isaac Sim and how is it used for robotics simulation?",
  "mode": "whole_book"
}
```

**Schema**:
```python
{
  "query": str,  # Length: 1-50,000 characters
  "mode": "whole_book"  # Literal constant
}
```

**Validation Rules**:
- `query`: Non-empty, max 50,000 characters
- `mode`: Must be `"whole_book"`

### Response

**Status Code**: `200 OK`

**Body**:
```json
{
  "message": {
    "id": "a1b2c3d4-e5f6-4a5b-8c9d-0e1f2a3b4c5d",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "role": "assistant",
    "content": "Isaac Sim is NVIDIA's photorealistic robotics simulation platform built on Omniverse. It provides physically accurate simulations for robot navigation, manipulation, and perception tasks...",
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
      "text_preview": "Isaac Sim is NVIDIA's photorealistic robotics simulation platform. It is built on top of NVIDIA Omniverse and provides a comprehensive environment for developing, testing..."
    },
    {
      "chapter": "Chapter 3: AI Perception with Isaac ROS",
      "relevance_score": 0.76,
      "text_preview": "Isaac ROS provides hardware-accelerated ROS 2 packages for AI perception. These packages integrate seamlessly with Isaac Sim for testing perception algorithms..."
    }
  ]
}
```

**Schema**: See `ChatResponse` in data-model.md

### Error Responses

**400 Bad Request**: Invalid query format
```json
{
  "detail": "Query must be between 1 and 50,000 characters"
}
```

**404 Not Found**: Session does not exist
```json
{
  "detail": "Session not found"
}
```

**429 Too Many Requests**: Gemini API rate limit exceeded
```json
{
  "detail": "Rate limit exceeded. Please try again later."
}
```

**500 Internal Server Error**: Agent or database error
```json
{
  "detail": "Failed to process query"
}
```

**503 Service Unavailable**: Qdrant or Gemini API unavailable
```json
{
  "detail": "Retrieval service temporarily unavailable"
}
```

---

## 4. Selected-Text-Only Chat

**Endpoint**: `POST /api/sessions/{session_id}/selected-text-chat`

**Description**: Sends a question to the agent using ONLY user-selected text as context. The agent does not retrieve from Qdrant and uses zero external knowledge.

### Request

**Path Parameters**:
- `session_id` (UUID): The session identifier

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "query": "What is URDF?",
  "selected_text": "URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines links, joints, sensors, and visual/collision geometries.",
  "chapter_origin": "Chapter 4: URDF and Sensor Simulation",
  "mode": "selected_text"
}
```

**Schema**:
```python
{
  "query": str,  # Length: 1-50,000 characters
  "selected_text": str,  # Length: 1-100,000 characters
  "chapter_origin": Optional[str],  # Chapter name if known
  "mode": "selected_text"  # Literal constant
}
```

**Validation Rules**:
- `query`: Non-empty, max 50,000 characters
- `selected_text`: Non-empty, max 100,000 characters
- `chapter_origin`: Optional string
- `mode`: Must be `"selected_text"`

### Response

**Status Code**: `200 OK`

**Body**:
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
      "gemini_model": "gemini-1.5-pro",
      "selected_text_length": 134
    }
  },
  "sources": null
}
```

**Note**: `sources` is always `null` for selected-text mode (no retrieval from Qdrant).

**Schema**: See `ChatResponse` in data-model.md

### Error Responses

**400 Bad Request**: Invalid input
```json
{
  "detail": "Selected text must be between 1 and 100,000 characters"
}
```

**400 Bad Request**: Empty selected text
```json
{
  "detail": "Selected text cannot be empty or whitespace-only"
}
```

**404 Not Found**: Session does not exist
```json
{
  "detail": "Session not found"
}
```

**429 Too Many Requests**: Gemini API rate limit exceeded
```json
{
  "detail": "Rate limit exceeded. Please try again later."
}
```

**500 Internal Server Error**: Agent or database error
```json
{
  "detail": "Failed to process query"
}
```

---

## HTTP Status Code Reference

| Status Code | Meaning | When Used |
|-------------|---------|-----------|
| 200 OK | Success | GET /history, POST /chat, POST /selected-text-chat |
| 201 Created | Resource created | POST /sessions |
| 400 Bad Request | Invalid input | Malformed JSON, validation errors |
| 404 Not Found | Resource not found | Session ID doesn't exist |
| 429 Too Many Requests | Rate limit exceeded | Gemini API rate limiting |
| 500 Internal Server Error | Server error | Database failures, agent crashes |
| 503 Service Unavailable | External service down | Qdrant or Gemini API unavailable |

---

## Example Usage

See `quickstart.md` for curl examples and usage patterns.

---

## Security Notes

**No Authentication**: For MVP, no authentication or authorization is implemented. Session IDs are opaque UUIDs that act as bearer tokens.

**Input Validation**: All inputs are validated by Pydantic models at the API layer. SQLi attacks are prevented via parameterized queries.

**Rate Limiting**: Not implemented in MVP. Future work: Add per-session rate limits (e.g., 10 queries/minute).

---

**Next**: See `quickstart.md` for developer setup and example requests.
