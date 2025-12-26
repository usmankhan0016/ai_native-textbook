# Phase 1 Design: Data Model & Validation Schemas

**Completed**: 2025-12-20
**From Plan**: [plan.md](plan.md) | **From Research**: [research.md](research.md)

## Data Entities

### 1. Query
**Purpose**: User-submitted text for semantic search

**Fields**:
- `text` (string, required): Query (question, topic, or keyword phrase)
  - Min: 1 char, Max: 1000 chars
  - No special escaping needed

### 2. Chunk
**Purpose**: Semantic unit from textbook stored in Qdrant

**Fields**:
- `chunk_id` (string): Unique identifier (e.g., `34af8123eabc3065#0001`)
- `text` (string): Extracted content (500-800 tokens)
- `source_url` (string): Original page URL
- `chapter` (string): Chapter/module designation
- `section` (string, optional): Section header
- `book_id` (string): Fixed to `"ai-native-textbook"`

**Validation Rules**:
- All required fields non-null
- chunk_id matches pattern: `^[a-f0-9]{32}#\d{4}$`
- text length > 100 chars
- source_url is valid HTTPS URL pointing to ai_native-textbook domain
- chapter non-empty
- book_id == "ai-native-textbook"

### 3. Embedding
**Purpose**: 1024-dimensional semantic vector

**Fields**:
- `vector` (array[float]): 1024 dimensions
- `source` (string): "chunk" or "query"

**Validation Rules**:
- Length exactly 1024
- All values are finite floats
- Norm approximately 1.0 (±0.1 tolerance)

### 4. Relevance Score
**Purpose**: Cosine similarity between query and chunk

**Fields**:
- `score` (float): Value in [0, 1]

**Validation Rules**:
- 0 ≤ score ≤ 1
- Scores monotonically decreasing by rank
- Typical good matches: > 0.7

### 5. Search Result
**Purpose**: Single chunk returned with metadata

**Fields**:
- `rank` (integer): Position in results (1-indexed, 1 to top_k)
- All chunk fields (chunk_id, text, source_url, chapter, section, book_id)
- `relevance_score` (float): Cosine similarity
- `latency_ms` (integer): Query execution time

### 6. Query Result (Aggregated)
**Purpose**: Complete response from search_chunks()

**Fields**:
- `query` (string): Original user query
- `top_k` (integer): Number requested
- `results` (array[SearchResult]): Ranked results
- `total_results` (integer): Total collection size (36)
- `validation` (object): Accuracy checks
  - `all_metadata_valid` (boolean)
  - `scores_monotonic` (boolean)
  - `expected_module_found` (boolean, if applicable)
  - `latency_acceptable` (boolean)
- `timestamp` (string, ISO 8601)
- `duration_ms` (integer): Total latency

## Validation Schemas

### Metadata Validation
Verify all chunk metadata fields present and valid.

```python
def validate_metadata(chunk: Dict) -> bool:
    required = ["chunk_id", "text", "source_url", "chapter", "book_id"]
    for field in required:
        if field not in chunk or chunk[field] is None:
            return False
    # Check chunk_id format, text length, URL validity, etc.
    return True
```

**Expected**: All 36 chunks pass validation.

### Relevance Validation
Verify scores are in range and monotonically decreasing.

```python
def validate_relevance(results: List[Dict]) -> Dict[str, bool]:
    if not results:
        return {"scores_in_range": True, "scores_monotonic": True}

    scores = [r["relevance_score"] for r in results]
    return {
        "scores_in_range": all(0 <= s <= 1 for s in scores),
        "scores_monotonic": all(scores[i] >= scores[i+1] for i in range(len(scores)-1)),
        "top_score_meaningful": scores[0] > 0.6
    }
```

### Consistency Validation
Execute query N times; verify all results identical.

```python
def validate_consistency(query: str, num_runs: int = 10) -> Dict[str, bool]:
    results_list = [search_chunks(query, top_k=5) for _ in range(num_runs)]
    baseline = results_list[0]

    all_consistent = all(
        len(r) == len(baseline) and
        all(r[i]["chunk_id"] == baseline[i]["chunk_id"]
            for i in range(len(baseline)))
        for r in results_list[1:]
    )

    return {"fully_consistent": all_consistent, "num_runs": num_runs}
```

**Expected**: 100% consistency (0% variation).

### Correctness Validation
Verify expected chapter in top-3 for conceptual queries.

```python
EXPECTED_MAPPINGS = {
    "What is ROS 2?": "Module 1",
    "How does sensor simulation work?": "Sensor Simulation",
    # ... more mappings
}

def validate_correctness(query: str, results: List[Dict]) -> Dict[str, bool]:
    if query not in EXPECTED_MAPPINGS:
        return {"skipped": True}

    expected = EXPECTED_MAPPINGS[query]
    found = any(expected in r["chapter"] for r in results[:3])

    return {"expected_module": expected, "found_in_top_3": found}
```

**Expected**: Expected chapter in top-3 for all conceptual queries.

## Data Flow

```
Query Text
  ↓ [Cohere Embed]
Query Vector (1024-dim)
  ↓ [Qdrant Search]
Top-K Chunks + Scores
  ↓ [Validate Metadata]
Metadata Check (non-null, valid format)
  ↓ [Validate Relevance]
Relevance Check (scores monotonic)
  ↓ [Validate Correctness]
Correctness Check (expected module in top-3)
  ↓ [Log Results]
JSON Lines Output
  ↓
QueryResult {query, results[], validation{}, timestamp, duration_ms}
```

## Key Notes

- **Read-only**: No writes to Qdrant; purely observational
- **No caching**: Each query independent
- **No intermediate storage**: Results logged but not persisted to database
- **Scaling**: Linear with query count; Qdrant handles cosine similarity efficiently

**Phase 1 Complete**: Data model and validation schemas defined. Ready for Quickstart and Task Breakdown.
