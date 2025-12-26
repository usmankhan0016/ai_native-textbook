# Implementation Plan: RAG Retrieval Validation

**Branch**: `006-rag-retrieval-validation` | **Date**: 2025-12-20 | **Spec**: [spec.md](spec.md)
**Input**: Validate semantic retrieval accuracy for Docusaurus-based textbook RAG system with existing embeddings from 005-rag-content-ingestion

## Summary

This feature validates the quality of semantic retrieval from Qdrant using pre-computed Cohere embeddings. The implementation provides a production-ready retrieval script that:
1. **Queries Qdrant** using cosine similarity with configurable top-k results
2. **Tests whole-book and section-level retrieval** against curated test queries
3. **Validates chunk metadata** (chapter, source URL, book_id, section)
4. **Logs results and accuracy metrics** for systematic validation
5. **Enables consistency testing** through repeated query execution

The script reuses existing infrastructure (005's Qdrant collection + Cohere embeddings) with no re-embedding or data changes.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: qdrant-client (>=1.7.0), cohere (>=5.0.0), python-dotenv, requests
**Storage**: Qdrant Cloud (cloud-hosted, existing collection "ai-native-textbook" from 005)
**Testing**: pytest (unit/integration), manual validation scripts
**Target Platform**: Linux CLI (no UI, no API server)
**Project Type**: Single Python script + test suite (validation tool)
**Performance Goals**: <500ms p95 latency per query; 100% metadata accuracy; 0% variation across repeated queries
**Constraints**: Read-only access to Qdrant; no re-embedding; no caching; no LLM reasoning; stateless queries
**Scale/Scope**: 36+ stored vectors; 10-20 test queries; 5-10 validation scenarios; <200 lines core logic

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Technical Accuracy First** ✅
- Requirement: All retrieval validation MUST use actual Qdrant API calls with real embeddings, not mocks
- Validation: Script queries live "ai-native-textbook" collection; all test results verified against actual stored data
- Status: **PASS** - No mocks; production-ready validation against live data

**Principle III: Modular Architecture** ✅
- Requirement: Validation tool MUST function independently; retrievable as standalone module
- Validation: Retrieval functions can be imported and reused by downstream chatbot/Q&A systems
- Status: **PASS** - Single script with modular function design; no dependency on feature 007+

**Principle IV: RAG-First Documentation** ✅
- Requirement: Feature MUST document itself for RAG ingestion; validation results MUST be logged and queryable
- Validation: All queries, results, and accuracy metrics logged to structured JSON/CSV for analysis
- Status: **PASS** - Structured logging enables future RAG indexing of validation results

**Principle V: Progressive Complexity** ✅
- Requirement: Validation MUST start simple (basic retrieval) before advancing to complex (cross-chapter filtering)
- Validation: User Stories ordered by priority; P1 covers basic retrieval, P2 covers advanced filtering
- Status: **PASS** - Implementation follows user story order (whole-book → metadata → top-k → consistency)

**Overall Gate Result**: ✅ **PASS** - No violations; ready for Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-retrieval-validation/
├── plan.md                          # This file (implementation plan)
├── research.md                      # Phase 0: Decision rationale and research findings
├── data-model.md                    # Phase 1: Data structures and validation schemas
├── quickstart.md                    # Phase 1: How to run the retrieval validator
├── checklists/
│   └── requirements.md              # Specification quality checklist
├── contracts/
│   └── retrieval-schema.json        # Query/response JSON schemas
└── tasks.md                         # Phase 2: Detailed task breakdown (from /sp.tasks)
```

### Source Code (repository root)

```text
backend/retrieval/
├── retriever.py                     # Core retrieval logic (cosine similarity, top-k)
├── validators.py                    # Metadata and accuracy validators
├── test_queries.py                  # Curated test queries (whole-book, section-level)
├── logger.py                        # Structured logging (JSON, CSV output)
└── __init__.py

backend/tests/retrieval/
├── test_retriever.py                # Unit tests for cosine similarity, top-k, metadata
├── test_consistency.py              # Integration tests for repeated queries
├── test_metadata_filtering.py       # Tests for chapter/module filtering
└── fixtures/
    ├── sample_queries.json          # Test query fixtures
    └── expected_results.json        # Ground-truth results for accuracy validation
```

**Structure Decision**: Single Python module under `backend/retrieval/` following the existing project layout from 005-rag-content-ingestion. Validation is a CLI tool, not a service, so minimal structure. Tests follow pytest convention with fixtures for reproducibility.

## Complexity Tracking

> No Constitution violations requiring justification. Simple, focused validation tool with clear dependencies on existing infrastructure (005's Qdrant + Cohere embeddings).

---

## Design Approach

### Core Components

**1. Retriever Module** (`retriever.py`)
- Initializes QdrantClient with environment credentials (reuses `.env` from 005)
- Accepts query text → embeds via Cohere API → searches Qdrant
- Returns top-k chunks with:
  - chunk_id, text, relevance_score
  - metadata: source_url, chapter, section, book_id
- Supports optional metadata filtering (e.g., chapter="Module 2")
- **No caching**: Each query is independent

**2. Validators Module** (`validators.py`)
- `validate_metadata()`: Confirms chunk_id, source_url, chapter, section, book_id are non-null
- `validate_relevance()`: Checks that scores are between 0 and 1 and monotonically decreasing
- `validate_correctness()`: Maps query → expected module/chapter; confirms top-3 include correct content
- `validate_consistency()`: Compares results from N repeated queries; flags any variation

**3. Test Queries** (`test_queries.py`)
```python
WHOLE_BOOK_QUERIES = [
    {"query": "What is ROS 2?", "expected_module": "Module 1"},
    {"query": "How does sensor simulation work?", "expected_chapter": "Chapter 3"},
    # ... 8+ more conceptual queries
]

METADATA_FILTER_QUERIES = [
    {"query": "Digital twin", "filter": {"chapter": "Module 2"}, "expected_count": ">0"},
    # ... 4+ more filtered queries
]

EDGE_CASE_QUERIES = [
    {"query": "Quantum computing", "expected_result": "zero_results"},  # not in textbook
    # ... 4+ more edge cases
]
```

**4. Logger Module** (`logger.py`)
- Writes to JSON Lines format: one retrieval event per line
- Schema: `{timestamp, query, top_k, results: [{id, text, score, metadata}], duration_ms, validation_status}`
- Optional CSV export for spreadsheet analysis
- Enables RAG-indexing of validation results

### Data Flow

```
Test Query
    ↓
[Cohere Embed] → Query Vector (1024-dim)
    ↓
[Qdrant Search] → Top-k Chunks + Scores
    ↓
[Metadata Validation] → Check: source_url, chapter, section, book_id
    ↓
[Correctness Validation] → Check: relevance, ranking, expected_module
    ↓
[Logger] → JSON output (queryable, analyzable)
    ↓
Test Results (accuracy metrics, latency, consistency)
```

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| Reuse Cohere embed-english-v3.0 | Ensures consistency with 005's embeddings; no new API calls to train on |
| Cosine similarity only (no reranking) | Meets spec requirement "no reranking"; validates pure vector search quality |
| Read-only Qdrant access | Validates existing data; enables parallel test runs without interference |
| Structured JSON logging | Enables future RAG indexing of validation results; supports metrics analysis |
| No caching between queries | Tests actual system behavior; detects non-determinism; validates consistency |
| CLI script (not API) | Meets spec requirement "script-based validation (no API, no UI)" |

### Dependencies and Constraints

**Must Have**:
- QdrantClient (1.7.0+) - Already installed in 005, reused here
- Cohere SDK (5.0.0+) - Already installed in 005, reused here
- python-dotenv - Already in 005, reused here
- pytest (for tests)

**Must NOT Have**:
- No LLM for query intent understanding
- No caching layer
- No reranking or hybrid search
- No UI or API server
- No data modification (read-only)

**Deployment**:
- Single Python package: `backend/retrieval/`
- Runs via CLI: `python -m backend.retrieval --query "What is ROS 2?" --top-k 5`
- Configuration via `.env` (reuses 005's environment)

### Testing Strategy

1. **Unit Tests** (`test_retriever.py`)
   - Mock Qdrant responses? NO - use live data
   - Test cosine similarity logic? NO - rely on Qdrant
   - Test metadata parsing from results? YES
   - Test top-k filtering? YES (verify ordering, no score inversion)

2. **Integration Tests** (`test_consistency.py`)
   - Execute same query 10× consecutively
   - Verify 100% identical results (same chunks, same order, same scores)
   - Detect any non-determinism

3. **Validation Tests** (`test_metadata_filtering.py`)
   - Execute filtered queries (e.g., "chapter=Module 2")
   - Verify all results have correct chapter tag
   - Test cross-chapter queries return mixed results correctly sorted

4. **Manual Acceptance Tests** (run from test_queries.py)
   - Execute WHOLE_BOOK_QUERIES; inspect top-3 for correctness
   - Execute METADATA_FILTER_QUERIES; verify filtering works
   - Execute EDGE_CASE_QUERIES; confirm zero-result handling

---

## Interfaces & API Contracts

### Retriever Function Signature

```python
def search_chunks(
    query: str,
    top_k: int = 5,
    collection_name: str = "ai-native-textbook",
    metadata_filter: Optional[Dict[str, str]] = None,
    qdrant_client: Optional[QdrantClient] = None,
    cohere_client: Optional[ClientV2] = None,
) -> List[Dict]:
    """
    Search Qdrant for semantically similar chunks using Cohere embeddings.

    Args:
        query: User query text (question, topic, or keyword phrase)
        top_k: Number of results to return (default 5)
        collection_name: Qdrant collection to search
        metadata_filter: Optional filter, e.g., {"chapter": "Module 2"}
        qdrant_client: QdrantClient instance (auto-created if None)
        cohere_client: ClientV2 instance (auto-created if None)

    Returns:
        List of dicts:
        [
            {
                "chunk_id": "abc123#0001",
                "text": "ROS 2 is a middleware...",
                "relevance_score": 0.87,
                "source_url": "https://docusaurus.../module-1/chapter-1",
                "chapter": "Chapter 1: ROS 2 Basics",
                "section": "Introduction",
                "book_id": "ai-native-textbook"
            },
            ...
        ]

    Raises:
        ConnectionError: If Qdrant or Cohere API unavailable
        ValueError: If query is empty or top_k <= 0
    """
```

### Response Schema (JSON)

See `contracts/retrieval-schema.json`:
```json
{
  "query": "What is ROS 2?",
  "top_k": 5,
  "results": [
    {
      "rank": 1,
      "chunk_id": "abc123#0001",
      "text": "ROS 2 is a middleware for robotics...",
      "relevance_score": 0.92,
      "source_url": "https://example.com/module-1/chapter-1",
      "chapter": "Chapter 1: ROS 2 Basics",
      "section": "What is ROS 2?",
      "book_id": "ai-native-textbook",
      "latency_ms": 145
    }
  ],
  "total_results": 36,
  "validation": {
    "all_metadata_valid": true,
    "scores_monotonic": true,
    "expected_module_found": true
  }
}
```

---

## Dependencies on Feature 005

**What we reuse**:
1. Qdrant Cloud collection: "ai-native-textbook" (36+ vectors)
2. Cohere embeddings: embed-english-v3.0 (1024-dim)
3. Environment config: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY (in .env)
4. Chunk schema: chunk_id, text, source_url, chapter, section, book_id

**What we validate**:
1. Metadata integrity (all chunks have required fields)
2. Embedding quality (cosine similarity produces sensible results)
3. Retrieval consistency (same query → same results)
4. Accuracy (correct chapters retrieved for known topics)

**No modifications to 005's data**: Read-only access; validation script is purely observational.

---

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Qdrant API downtime | Validation cannot run | Retry logic with exponential backoff; document known outage windows |
| Cohere API rate limits | Queries throttled mid-test | Use batch embedding; test during off-peak hours; implement request queuing |
| Floating-point non-determinism | Consistency test may fail spuriously | Document tolerance threshold (e.g., ±0.00001 difference acceptable) |
| Metadata corruption in Qdrant | Validators flag bad data | Early validation pass; skip corrupted chunks if <10% affected |
| Network latency variance | p95 latency metric noisy | Run tests from same cloud region; measure 100+ queries for stability |

---

## Success Criteria Alignment

| Spec Criterion | Implementation Validation |
|---|---|
| SC-001: 100% accuracy on conceptual queries (top-3) | WHOLE_BOOK_QUERIES test; manual inspection of top-3 |
| SC-002: 100% precision on metadata filtering | METADATA_FILTER_QUERIES test; verify all results have correct chapter tag |
| SC-003: top-1 ⊂ top-5 ⊂ top-10 (100% nested) | test_retriever.py: verify same results across k=1,5,10 |
| SC-004: 0% variation across 10 runs | test_consistency.py: execute same query 10×, compare results |
| SC-005: <500ms p95 latency | Logger records latency_ms; analyze from JSON output |
| SC-006: Zero false positives (top-5) | WHOLE_BOOK_QUERIES + manual inspection |
| SC-007: Valid metadata on all chunks | validate_metadata() in validators.py; flag missing fields |
| SC-008: Zero-result queries handled <100ms | EDGE_CASE_QUERIES; timer for "no results" response |
