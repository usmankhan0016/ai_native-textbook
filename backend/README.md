# RAG Content Ingestion Pipeline

Production-grade Python script for discovering, extracting, chunking, and embedding Docusaurus book content for semantic retrieval in RAG pipelines.

## Features

- **Automated URL Discovery**: Fetches all published Docusaurus book URLs from `sitemap.xml`
- **Clean Content Extraction**: Removes navigation, sidebars, footers, and UI boilerplate
- **Intelligent Text Chunking**: Splits content into 500–800 token chunks with semantic overlap
- **Embedding Generation**: Uses Cohere's embedding API for 1024-dimensional vectors
- **Vector Storage**: Stores embeddings in Qdrant Cloud with rich metadata (chapter, section, source URL)
- **Idempotent Re-ingestion**: Safe re-runs without creating duplicates
- **Error Resilience**: Exponential backoff for transient failures and graceful degradation

## Prerequisites

- **Python 3.10+**
- **Cohere API Key**: Get free tier at [cohere.com](https://cohere.com)
- **Qdrant Cloud Account**: Free tier at [qdrant.io](https://qdrant.io)
- **Published Docusaurus Site**: The book must be built and deployed (supports static HTML only)

## Quick Start

### 1. Clone and Setup

```bash
cd backend
cp .env.example .env
```

### 2. Configure Environment Variables

Edit `.env` with your API keys and settings:

```bash
DOCUSAURUS_BASE_URL=https://your-docusaurus-site.com
DOCUSAURUS_BOOK_ID=your-book-id
COHERE_API_KEY=your-cohere-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
LOG_LEVEL=INFO
```

### 3. Install Dependencies

**Using UV (Recommended)**:
```bash
uv sync
```

**Using pip**:
```bash
pip install -r requirements.txt
```

### 4. Run Ingestion

```bash
# Run with environment variables from .env
python main.py

# Or pass parameters directly
python main.py --base-url https://my-book.com --book-id my-book
```

### 5. Verify Results

Check logs for success message:

```
✅ Discovered 456 URLs from sitemap.xml
✅ Extracted content from 456 pages
✅ Generated 12,340 chunks (from 2.1M tokens)
✅ Created embedding vectors (batch: 100 req/min)
✅ Stored 12,340 vectors in Qdrant
⏱ Ingestion completed in 18 minutes
```

## Expected Output

```
Starting ingestion...
✅ Discovered 456 URLs from sitemap.xml
✅ Extracted content from 456 pages
✅ Generated 12,340 chunks (from 2.1M tokens)
✅ Created embedding vectors (batch: 100 req/min)
✅ Stored 12,340 vectors in Qdrant
⏱ Ingestion completed in 18 minutes
```

Execution time depends on:
- Book size (number of pages)
- Network latency to Docusaurus site and Cohere API
- Cohere API rate limits (~100 requests/minute on free tier)
- Qdrant insert latency

## Architecture

### Pipeline Flow

```
Sitemap Discovery → Content Extraction → Text Chunking → Embedding → Vector Storage
      (URLs)            (Clean HTML)      (500-800 tokens)  (Cohere)    (Qdrant)
```

### Core Functions

- **`get_all_urls(base_url)`**: Fetch sitemap, extract all book URLs
- **`extract_text_from_url(url)`**: Fetch HTML, remove boilerplate, extract main content
- **`chunk_text(text, source_url)`**: Split into overlapping chunks with deterministic IDs
- **`embed(texts, api_key)`**: Send to Cohere API, return 1024-dim embeddings
- **`create_collection(qdrant_client, collection_name)`**: Create Qdrant collection (idempotent)
- **`save_chunk_to_qdrant(qdrant_client, collection_name, chunk, embedding)`**: Upsert vectors with metadata
- **`main(...)`**: Orchestrate full pipeline

### Data Model

**Chunk**:
- `chunk_id`: Deterministic ID based on source URL + sequence number
- `text`: 500–800 tokens
- `tokens`: Token count (validated)
- `source_url`: URL of source page
- `chapter`: Chapter title (extracted from headers)
- `section`: Section title (extracted from H2 headers)

**Vector Document** (in Qdrant):
- `id`: chunk_id (point ID)
- `vector`: 1024-dimensional embedding from Cohere
- `payload`: Metadata (book_id, chapter, section, source_url, chunk_id, text)

## Configuration

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `DOCUSAURUS_BASE_URL` | Yes | - | Base URL of your Docusaurus site |
| `DOCOSAURUS_BOOK_ID` | Yes | - | Unique identifier for the book |
| `COHERE_API_KEY` | Yes | - | API key from cohere.com |
| `QDRANT_URL` | Yes | - | Qdrant cluster URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
| `LOG_LEVEL` | No | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |

### Optional Tuning

```bash
COHERE_BATCH_SIZE=50          # Chunks per embedding request (default: 50)
HTTP_TIMEOUT=30               # HTTP request timeout in seconds (default: 30)
MAX_RETRIES=3                 # Max retry attempts for transient failures (default: 3)
```

## Performance

### Time Estimates

- **Small book** (50 pages): 5–10 minutes
- **Medium book** (100–500 pages): 15–30 minutes
- **Large book** (500+ pages): 30–60 minutes

Bottleneck: Cohere API rate limits (~100 requests/minute on free tier)

### Resource Usage

- **CPU**: Minimal (text parsing, not compute-intensive)
- **Memory**: ~200MB for typical book
- **Network**: Depends on Docusaurus site and API latencies

## Idempotency

Re-running ingestion is **safe and recommended** for:
- **Content updates**: Changed sections automatically re-embedded
- **Error recovery**: Failed ingestion can be retried; completed chunks won't be re-processed
- **Re-indexing**: Re-run anytime to sync Qdrant with latest content

**How it works**: Each chunk has a deterministic `chunk_id` based on its source URL and sequence. When upserting to Qdrant, duplicate IDs are updated (not inserted), ensuring zero duplicates even after multiple runs.

## Troubleshooting

### Common Errors

**`CohereFailed: 401 Unauthorized`**
- Check: `COHERE_API_KEY` is valid and has sufficient quota

**`ConnectionError: Unable to connect to Qdrant`**
- Check: `QDRANT_URL` is correct and cluster is online
- Check: `QDRANT_API_KEY` is valid
- Check: Network connectivity to Qdrant cluster

**`Timeout: Failed to fetch {url} after 3 retries`**
- Docusaurus site is slow or unreachable
- Check: `DOCUSAURUS_BASE_URL` is correct and responsive
- Try: Run again later or check Docusaurus site status

**`Ingestion failed: Chunks embedded but not all stored`**
- Partial failure (most chunks stored, some failed)
- Check: Qdrant quota not exceeded
- Re-run: Will resume from failure point

### Debug Logging

```bash
LOG_LEVEL=DEBUG python main.py
```

Enables detailed logging of:
- URL discovery
- Content extraction (per page)
- Chunk generation
- Cohere API calls
- Qdrant upsert operations

## Metrics & Validation

After ingestion, verify success:

```python
from qdrant_client import QdrantClient

client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-key")
collection_info = client.get_collection("your-book-id")
print(f"Total vectors: {collection_info.points_count}")
```

Expected metrics:
- **Vectors indexed**: 100% of chunks
- **Chunk count**: Typical 10k–50k for textbook
- **Storage**: 50–250 MB (Qdrant Free Tier quota: 1 GB)

## Advanced Usage

### Search Examples (T025 - Vector Search)

Once ingestion is complete, search vectors by semantic similarity with optional metadata filtering:

#### Example 1: Basic Semantic Search

```python
from qdrant_client import QdrantClient
from cohere import Cohere
from main import search_vectors

client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-key")
cohere_client = Cohere(api_key="your-cohere-key")

# Generate query embedding
query = "How do I create a ROS 2 node?"
query_embedding = cohere_client.embed([query], model="embed-english-v3.0").embeddings[0]

# Search without filters (returns top 5 results from entire book)
results = search_vectors(
    qdrant_client=client,
    collection_name="ai-native-textbook",
    query_embedding=query_embedding,
    limit=5
)

for result in results:
    print(f"Similarity: {result['similarity_score']:.3f}")
    print(f"  Chapter: {result['chapter']}")
    print(f"  Section: {result['section']}")
    print(f"  Text: {result['text'][:100]}...")
    print()
```

**Output**:
```
Similarity: 0.847
  Chapter: Chapter 1: Getting Started with ROS 2
  Section: Creating Nodes
  Text: A ROS 2 node is a basic computational unit that acts as a container for your...

Similarity: 0.835
  Chapter: Chapter 3: Advanced ROS 2 Concepts
  Section: Node Communication
  Text: Nodes communicate with each other through topics and services. Each node can...
```

#### Example 2: Filtered Search by Chapter

Search only within a specific chapter:

```python
# Search results limited to Chapter 1
results = search_vectors(
    qdrant_client=client,
    collection_name="ai-native-textbook",
    query_embedding=query_embedding,
    chapter_filter="Chapter 1: Getting Started with ROS 2",
    limit=5
)

for result in results:
    print(f"[{result['similarity_score']:.3f}] {result['section']}: {result['text'][:80]}...")
```

**Benefit**: Narrows results to relevant context (e.g., "Only search tutorials in Chapter 1")

#### Example 3: Batch Search

Search multiple queries in a single operation:

```python
# Search for multiple concepts
queries = [
    "How do I publish a message?",
    "What is a subscriber?",
    "How do timers work in ROS 2?"
]

for query in queries:
    query_embedding = cohere_client.embed([query], model="embed-english-v3.0").embeddings[0]
    results = search_vectors(
        qdrant_client=client,
        collection_name="ai-native-textbook",
        query_embedding=query_embedding,
        limit=3
    )

    print(f"\nQuery: {query}")
    for result in results:
        print(f"  {result['chapter']} → {result['section']} ({result['similarity_score']:.2f})")
```

#### Example 4: Metadata Inspection

Access all metadata from search results:

```python
results = search_vectors(
    qdrant_client=client,
    collection_name="ai-native-textbook",
    query_embedding=query_embedding,
    limit=1
)

result = results[0]
print(f"chunk_id:      {result['chunk_id']}")
print(f"source_url:    {result['source_url']}")
print(f"chapter:       {result['chapter']}")
print(f"section:       {result['section']}")
print(f"similarity:    {result['similarity_score']:.3f}")
print(f"text_preview:  {result['text'][:200]}...")
```

#### Search Function Reference

**Function Signature**:
```python
def search_vectors(
    qdrant_client: QdrantClient,
    collection_name: str,
    query_embedding: List[float],           # 1024-dimensional vector from Cohere
    chapter_filter: Optional[str] = None,   # Filter by chapter (optional)
    limit: int = 5,                         # Max results to return
    logger: Optional[logging.Logger] = None
) -> List[Dict]:
```

**Return Format**:
```python
[
    {
        "chunk_id": "abc123def456789f#0000",
        "source_url": "https://...",
        "chapter": "Chapter 1: ...",
        "section": "Section Title",
        "text": "First 200 characters of chunk text...",
        "similarity_score": 0.847
    },
    ...
]
```

**Metadata Available in Results**:
- `chunk_id`: Unique identifier for this chunk (deterministic)
- `source_url`: Full URL of the source page
- `chapter`: Chapter title (extracted from page header)
- `section`: Section title (extracted from H2 headers)
- `similarity_score`: Cosine similarity (0-1, higher = more relevant)
- `text`: First 200 characters of chunk (full text available in Qdrant payload)

### Content Updates

If book content changes:

```bash
# Re-run ingestion—only changed pages will have new embeddings
python main.py
```

Changed chunks are automatically re-embedded; unchanged chunks are skipped.

## Testing

Run unit tests:

```bash
pytest tests/ -v
```

Run integration test (against real Docusaurus site):

```bash
pytest tests/test_ingestion.py::test_end_to_end -v
```

---

## RAG Agent Backend (Feature 007)

Production-ready FastAPI backend for dual-mode RAG queries with persistent chat sessions.

### Features

- **Dual-Mode Querying**:
  - **Whole-book**: Retrieve relevant chapters from Qdrant, generate grounded answers
  - **Selected-text**: Answer using ONLY user-selected text, zero external knowledge leakage
- **Persistent Sessions**: Multi-turn conversations with full chat history
- **Gemini API Integration**: Uses Google's Gemini 1.5 Pro for answer generation
- **Neon Postgres**: Session management and message storage
- **REST API**: FastAPI endpoints for session management and chat

### Quick Start

#### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**Note**: Feature 007 requires FastAPI, uvicorn, google-generativeai, asyncpg, and other dependencies listed in requirements.txt.

#### 2. Configure Environment

Add to your `.env` file:

```bash
# Gemini API (Google AI)
GEMINI_API_KEY=your-gemini-api-key-here

# Neon Serverless Postgres
DATABASE_URL=postgresql://user:password@ep-xyz.us-east-2.aws.neon.tech/dbname?sslmode=require

# Collection name (must match ingestion)
QDRANT_COLLECTION_NAME=ai-native-textbook
```

#### 3. Initialize Database Schema

Create tables in your Neon Postgres database:

```sql
-- Sessions table
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_sessions_created_at ON sessions(created_at DESC);

-- Messages table
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

-- Selected text metadata table
CREATE TABLE selected_text_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    selected_text TEXT NOT NULL,
    chapter_origin VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_selected_text_message_id ON selected_text_metadata(message_id);
```

#### 4. Run FastAPI Server

```bash
cd backend
uvicorn api.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using watchfiles
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

#### 5. Test Health Check

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{"status": "healthy", "service": "rag-agent-backend"}
```

### API Endpoints

**Note**: Full endpoint implementations are in progress (Phase 2-5). See `specs/007-agent-backend/contracts/api-endpoints.md` for complete API specifications.

| Endpoint | Method | Purpose | Status |
|----------|--------|---------|--------|
| `/health` | GET | Health check | ✅ Available |
| `/api/sessions` | POST | Create new chat session | 🚧 Phase 5 |
| `/api/sessions/{id}/history` | GET | Retrieve chat history | 🚧 Phase 3 |
| `/api/sessions/{id}/chat` | POST | Whole-book query | 🚧 Phase 3 |
| `/api/sessions/{id}/selected-text-chat` | POST | Selected-text query | 🚧 Phase 4 |

### Usage Examples

**Coming Soon**: Full usage examples will be available after Phase 3-5 implementation.

For detailed API documentation and curl examples, see:
- `specs/007-agent-backend/quickstart.md`
- `specs/007-agent-backend/contracts/api-endpoints.md`

### Architecture

```
FastAPI Backend
├── agent/          # Gemini API integration, dual-mode routing
├── api/            # REST endpoints, Pydantic models
├── db/             # Neon Postgres connection, repositories
└── retrieval/      # Reused from Feature 006 (Qdrant search)
```

### Troubleshooting

**`ModuleNotFoundError: No module named 'fastapi'`**
- Run: `pip install -r requirements.txt`

**`google.generativeai.types.generation_types.StopCandidateException`**
- Gemini API safety filter triggered
- Check: Query content is appropriate
- Try: Rephrase query or adjust safety settings

**`asyncpg.exceptions.ConnectionDoesNotExistError`**
- Database connection failed
- Check: `DATABASE_URL` is correct and Neon Postgres is accessible
- Verify: Connection string includes `?sslmode=require`

**Session not found (404)**
- Session ID is invalid or doesn't exist
- Create new session with `POST /api/sessions`

For more troubleshooting, see `specs/007-agent-backend/quickstart.md#8-troubleshooting`

---

## License

MIT

## Support

For issues or questions:
1. Check [Troubleshooting](#troubleshooting)
2. Enable debug logging: `LOG_LEVEL=DEBUG`
3. Review logs for specific error messages
4. Check external service status (Docusaurus site, Cohere API, Qdrant cluster)
