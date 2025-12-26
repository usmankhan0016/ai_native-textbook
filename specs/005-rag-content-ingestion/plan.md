# Implementation Plan: Content Ingestion & Embeddings for RAG Chatbot

**Branch**: `005-rag-content-ingestion` | **Date**: 2025-12-19 | **Spec**: `/specs/005-rag-content-ingestion/spec.md`
**Input**: Feature specification for RAG content ingestion pipeline

## Summary

Build a production-grade Python script (`main.py`) that discovers all published Docusaurus book content via `sitemap.xml`, extracts clean main content, chunks text with metadata, generates embeddings via Cohere API, and stores vectors in Qdrant Cloud with full idempotency and error resilience. Reference implementation will enable AI engineers to index technical books for semantic retrieval in RAG pipelines.

**Primary deliverable**: Single Python script with modular functions (get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant) orchestrated by a main() function.

**Tech stack**: Python 3.10+, UV for package management, Cohere for embeddings, Qdrant Python SDK, BeautifulSoup for HTML extraction.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:
- `requests` — HTTP client for fetching URLs and sitemaps
- `beautifulsoup4` — HTML parsing and content extraction
- `qdrant-client` — Python SDK for Qdrant Cloud vector database
- `cohere` — Cohere API client for embedding generation
- `python-dotenv` — Environment variable management
- `tiktoken` — Token counting for chunk size validation (OpenAI compatible)

**Storage**: Qdrant Cloud Free Tier (vector embeddings + metadata), no secondary database required for this phase
**Testing**: pytest + integration tests against published Docusaurus site
**Target Platform**: Linux/macOS CLI script (portable, no platform-specific dependencies)
**Project Type**: Single Python script with modular functions
**Performance Goals**: Complete full-book ingestion in ≤ 30 minutes (SC-005)
**Constraints**:
- Cohere API rate limits (~100 req/min on free tier, batch embedding API for efficiency)
- Qdrant Free Tier quota (~1GB storage)
- Static HTML discovery only (sitemap.xml or static navigation, no client-side JS)

**Scale/Scope**: Single Docusaurus book (~100-500 pages), ~10k-50k chunks per book, complete ingestion in one batch operation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Principles Alignment**:

✅ **Principle IV: RAG-First Documentation**
- Semantic chunking strategy: Each section (Learning Objectives, Key Concepts, Tutorials) = discrete semantic unit for embedding
- Metadata richness: chunk_id, source_url, book_id, chapter, section captured with every vector
- Idempotent re-ingestion: Deterministic chunk_id ensures no duplicates on re-runs
- Hierarchical structure: Leverage H1→H2→H3 headers for context extraction (Docusaurus standard)

✅ **Principle VI: Industry Alignment**
- Python 3.10+ (modern type hints, match statements)
- Standard HTTP library (requests) for web scraping
- Industry-standard vector DB (Qdrant, used in production RAG pipelines)
- Industry-standard embeddings provider (Cohere)

✅ **Principle VIII: Deployment & Infrastructure**
- Leverages Qdrant Cloud Free Tier (documented in constitution)
- Cohere API (scalable, production-ready)
- No custom infrastructure required; relies on external managed services
- Self-contained Python script: portable, reproducible, Docker-friendly

**Gates Status**: ✅ PASS — All relevant constitution principles satisfied. No violations detected.

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-content-ingestion/
├── plan.md                      # This file (architecture & design)
├── research.md                  # Phase 0 output (dependency research)
├── data-model.md                # Phase 1 output (chunk/vector data model)
├── quickstart.md                # Phase 1 output (setup & usage guide)
├── tasks.md                     # Phase 2 output (implementation tasks)
└── checklists/requirements.md   # Quality validation checklist
```

### Source Code (repository root)

```text
backend/
├── main.py                      # PRIMARY: Single-file ingestion script
├── pyproject.toml               # UV package manifest
├── .env.example                 # Required environment variables
├── requirements.txt             # Frozen dependencies for reproducibility
└── tests/
    ├── test_ingestion.py        # Integration tests against published site
    └── test_chunking.py         # Unit tests for text chunking
```

**Structure Decision**: Single-file Python script approach (`main.py`) per user specification. All system design functions contained in one file for clarity and ease of deployment. No separate service/model subdirectories needed for this phase. UV for modern Python package management (aligns with Constitution VI: Industry Alignment 2025+ standards).

## Phase 0: Research & Dependency Analysis

**Status**: Complete ✅

### Key Research Areas Resolved

**1. Docusaurus Sitemap Discovery (Static HTML)**
- **Decision**: Use `sitemap.xml` as primary discovery mechanism
- **Rationale**: Standard on all published Docusaurus sites; reliable, requires no client-side JS
- **Implementation**: Fetch `{base_url}/sitemap.xml`, parse XML, extract all `<loc>` URLs
- **Fallback**: Parse navigation JSON (Docusaurus exposes via `/sidebars.json`) if sitemap absent

**2. HTML Content Extraction (BeautifulSoup)**
- **Decision**: BeautifulSoup4 for parsing, target `<article>` or `<main>` semantic elements
- **Rationale**: Lightweight, no JS rendering overhead, sufficient for static HTML from published sites
- **Docusaurus-specific**: Docusaurus wraps content in `<article class="docusaurus_content">` or `<main>` tag
- **Boilerplate removal**: CSS selectors to remove `.docSidebar`, `.toc`, `.pagination`, etc.

**3. Text Chunking Strategy**
- **Decision**: Token-based chunking (500–800 tokens) with 10% overlap
- **Tool**: `tiktoken` (OpenAI tokenizer, compatible with Cohere embeddings)
- **Deterministic chunk_id**: `f"{source_url}#{sequence_number}"` (ensures uniqueness, stability across re-runs)
- **Chunk metadata**: source_url, chapter, section, chunk_id, original_text

**4. Cohere Embedding API**
- **Decision**: Cohere `embed-english-v3.0` (or latest stable model)
- **Rate limiting**: Batch API for efficiency (~100 req/min), implement exponential backoff
- **Cost**: Free tier ~100k embeddings/month (sufficient for textbook)
- **Dimension**: Cohere v3 = 1024-dimensional embeddings

**5. Qdrant Vector Storage**
- **Decision**: Qdrant Cloud Free Tier with persistent collection
- **Schema**: Collection named `{book_id}` with payload schema (chunk_id, source_url, chapter, section, text)
- **Upsert logic**: Use `chunk_id` as point ID; update if exists, insert if new (idempotency)
- **Quota**: Free Tier = 1GB; estimate ~10-50k chunks per textbook = ~50-250MB (safe margin)

**6. Error Handling & Resilience**
- **Network failures**: Retry with exponential backoff (3 attempts, 5s → 10s → 20s)
- **Cohere rate limits**: Catch 429 errors, sleep & retry
- **Qdrant connection loss**: Transaction-like behavior (save to memory, retry insertion)
- **Partial failures**: Log skipped pages/chunks; continue with remaining content

### No Blocking Unknowns

All technical decisions mapped to available libraries and free services. Ready for Phase 1 design.

## Phase 1: Design & Data Model

### Data Model

**Chunk Entity**
```
Chunk {
  chunk_id: str                    # Deterministic: {source_url}#{sequence_number}
  source_url: str                  # Full URL to Docusaurus page
  book_id: str                     # Identifier for book (e.g., "ai-native-textbook")
  chapter: str                     # Chapter title (extracted from breadcrumb or header)
  section: str                     # Section title (extracted from H2/H3)
  text: str                        # Actual chunk text (500–800 tokens)
  tokens: int                      # Token count (for validation)
  created_at: datetime             # Timestamp
}

VectorDocument {
  id: str                          # chunk_id (Qdrant point ID)
  vector: list[float]              # 1024-dim Cohere embedding
  payload: dict                    # Metadata (book_id, chapter, section, source_url, text)
}
```

**Qdrant Collection Schema**
```
Collection: {book_id}
Points: {
  id: chunk_id,
  vector: [1024-dim float array],
  payload: {
    book_id: str,
    chapter: str,
    section: str,
    source_url: str,
    chunk_id: str,
    text: str (or compressed)
  }
}
```

### Function Signatures (main.py)

```python
def get_all_urls(base_url: str, book_id: str) -> list[str]:
    """Fetch sitemap.xml, extract all URLs for the book."""

def extract_text_from_url(url: str) -> str:
    """Fetch page HTML, extract clean main content (no boilerplate)."""

def chunk_text(text: str, source_url: str, chunk_size: int = 500) -> list[dict]:
    """Split text into overlapping chunks. Return list of {chunk_id, text, tokens}."""

def embed(texts: list[str], api_key: str) -> list[list[float]]:
    """Send texts to Cohere API, return embeddings (batch for efficiency)."""

def create_collection(qdrant_client, collection_name: str) -> None:
    """Create Qdrant collection with schema (idempotent)."""

def save_chunk_to_qdrant(qdrant_client, collection_name: str, chunk: dict, embedding: list[float]) -> None:
    """Upsert chunk + embedding to Qdrant (deterministic ID = chunk_id)."""

def main(base_url: str, book_id: str, cohere_api_key: str, qdrant_url: str, qdrant_key: str) -> None:
    """Orchestrate: discover URLs → extract text → chunk → embed → store in Qdrant."""
```

### API Contracts

**Environment Variables** (.env)
```
DOCUSAURUS_BASE_URL=https://ai-native-textbook.example.com
DOCUSAURUS_BOOK_ID=ai-native-textbook
COHERE_API_KEY=<your-key>
QDRANT_URL=https://<your-cluster>.qdrant.io
QDRANT_API_KEY=<your-key>
LOG_LEVEL=INFO
```

**Ingestion Output** (STDOUT)
```
Starting ingestion...
✅ Discovered 456 URLs from sitemap.xml
✅ Extracted content from 456 pages
✅ Generated 12,340 chunks (from 2.1M tokens)
✅ Created embedding vectors (batch: 100 req/min)
✅ Stored 12,340 vectors in Qdrant
⏱ Ingestion completed in 18 minutes
```

### Validation Checklist (Phase 1 Gate)

- [ ] Data model captures all required metadata (chunk_id, book_id, chapter, section, source_url)
- [ ] Chunk_id deterministic (same input → same ID)
- [ ] Qdrant upsert logic prevents duplicates on re-run
- [ ] Function signatures match user specification
- [ ] Error handling strategy documented (retries, partial failures)
- [ ] Performance estimate: 30-minute SLA achievable (given rate limits)

---

## Phase 1 Deliverables (Pending Completion)

- [ ] `research.md` — Full research documentation
- [ ] `data-model.md` — Detailed entity and schema documentation
- [ ] `quickstart.md` — Setup, usage, and troubleshooting guide
- [ ] Backend folder structure initialized with `pyproject.toml` and `.env.example`

## Complexity Tracking

> **No violations detected** — Constitution principles fully satisfied.

| Area | Status | Notes |
|------|--------|-------|
| RAG-First Design (Principle IV) | ✅ Clear | Chunk metadata + deterministic IDs support semantic retrieval |
| Industry Alignment (Principle VI) | ✅ Clear | Python 3.10+, standard libraries, Qdrant/Cohere industry-standard |
| Infrastructure (Principle VIII) | ✅ Clear | Leverages free managed services; no custom infra |
