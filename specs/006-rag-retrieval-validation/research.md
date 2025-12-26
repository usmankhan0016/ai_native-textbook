# Phase 0 Research: RAG Retrieval Validation

**Completed**: 2025-12-20
**From Plan**: [plan.md](plan.md)

## Research Summary

This document resolves all technical decisions from the implementation plan. All items are based on the existing infrastructure from 005-rag-content-ingestion and do not require clarification.

---

## Decision 1: Retrieval Architecture (Cosine Similarity, No Reranking)

**Decision**: Implement pure semantic retrieval using Qdrant's native cosine similarity search.

**Rationale**:
- Spec explicitly requires "cosine similarity" search method with "no reranking"
- Qdrant natively supports cosine distance for vectors
- This approach validates raw embedding quality without masking problems via reranking
- Simplest implementation path: leverage Qdrant's built-in search

**Alternatives Considered**:
1. Hybrid search (BM25 keyword + vector): Violates "no hybrid search" constraint
2. Learned reranker: Violates "no reranking" constraint
3. Similarity threshold filtering: Could mask poor embeddings; using raw top-k is more transparent

**Implementation Path**:
```python
# Qdrant's search() returns cosine similarity scores natively
# No additional ranking logic needed
results = qdrant_client.search(
    collection_name=collection_name,
    query_vector=query_embedding,  # 1024-dim from Cohere
    limit=top_k,
    with_payload=True
)
# results[].score is cosine similarity in [0, 1]
```

---

## Decision 2: Embedding Method (Reuse Cohere v5+, No Re-Embedding)

**Decision**: Embed all test queries using Cohere's embed-english-v3.0 (same model as documents in 005).

**Rationale**:
- 005-rag-content-ingestion used `ClientV2` with embed-english-v3.0 for all 262 document chunks
- Using the same model for queries ensures consistency: query embedding space == document embedding space
- ClientV2 is the current (v5.0+) Cohere API; older models unavailable
- Avoids re-embedding existing documents (read-only access)

**Alternatives Considered**:
1. Different Cohere model (embed-english-v2.0): Incompatible; documents use v3.0
2. Open-source embedder (e.g., BGE): Would require re-indexing all documents; violates read-only constraint
3. OpenAI embeddings: Cost prohibitive; documents already embedded with Cohere

**Implementation Path**:
```python
from cohere import ClientV2

cohere_client = ClientV2(api_key=os.getenv("COHERE_API_KEY"))

response = cohere_client.embed(
    model="embed-english-v3.0",
    input_type="search_query",  # Different from "search_document" used for chunks
    texts=[query]
)

query_embedding = list(response.embeddings[0])  # 1024-dim vector
```

---

## Decision 3: Metadata Filtering Strategy (Qdrant Payload Filtering)

**Decision**: Implement optional metadata filtering using Qdrant's native payload filter API.

**Rationale**:
- Spec requires "optional metadata filtering (e.g., filter results by chapter or module)"
- Qdrant's `query_filter` parameter applies at search time (efficient)
- All documents in 005 have payload with chapter, section, source_url, book_id
- Avoids post-processing; scales with index

**Alternatives Considered**:
1. Post-search filtering in Python: Wastes Qdrant search results; slower for large k values
2. Separate BM25 index for metadata: Over-engineered; Qdrant's payload filter sufficient
3. No filtering: Doesn't meet spec requirement

**Implementation Path**:
```python
# Filter by chapter = "Module 2"
search_filter = {
    "key": "chapter",
    "match": {"value": "Module 2"}
}

results = qdrant_client.search(
    collection_name=collection_name,
    query_vector=query_embedding,
    query_filter=search_filter,  # Applied at search time
    limit=top_k,
    with_payload=True
)
```

---

## Decision 4: Validation Approach (No Mocks, Live Data)

**Decision**: All validation tests execute against live Qdrant Cloud data (36+ vectors).

**Rationale**:
- Spec requirement: "validate semantic retrieval accuracy" — requires real data
- Testing against mocks would validate nothing about actual system behavior
- Constitution Principle I (Technical Accuracy First) requires real infrastructure
- 005-rag-content-ingestion successfully populated collection; data is stable

**Alternatives Considered**:
1. Mock Qdrant responses: Doesn't validate real behavior; violates constitution
2. Test against smaller local Qdrant instance: Additional infra; requires syncing with cloud data
3. Use synthetic test vectors: Doesn't reflect real embedding quality

**Implementation Path**:
```python
# All tests use live QdrantClient pointing to Qdrant Cloud
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),  # Cloud endpoint
    api_key=os.getenv("QDRANT_API_KEY")
)

# No mocking; no local database
# Tests fail if API unavailable, which is valuable signal
```

---

## Decision 5: Logging Strategy (JSON Lines + Optional CSV)

**Decision**: Log all queries and results to JSON Lines format (one object per line); optional CSV export.

**Rationale**:
- Spec requirement: "log all retrieval queries and results for audit and debugging"
- JSON Lines is queryable, supports RAG indexing (Constitutional Principle IV)
- CSV export for non-technical users (spreadsheet analysis)
- Enables future analytics: latency distribution, accuracy by query type, etc.

**Alternatives Considered**:
1. Plain text logs: Not analyzable; loses structure
2. Database (SQLite/PostgreSQL): Overkill; adds deployment complexity
3. Cloud logging (CloudWatch, Stackdriver): Requires AWS/GCP account; single JSON files sufficient

**Implementation Path**:
```python
import json
import csv
from datetime import datetime

# Single file: retrieval_results.jsonl
with open("retrieval_results.jsonl", "a") as f:
    result = {
        "timestamp": datetime.utcnow().isoformat(),
        "query": query,
        "top_k": top_k,
        "results": [
            {
                "rank": i,
                "chunk_id": r.payload.get("chunk_id"),
                "text": r.payload.get("text")[:200],  # Truncate
                "relevance_score": r.score,
                "chapter": r.payload.get("chapter"),
                "latency_ms": elapsed_ms
            }
            for i, r in enumerate(results, 1)
        ],
        "validation": {
            "all_metadata_valid": all(r["chapter"] for r in results),
            "scores_monotonic": is_monotonic(scores),
            "expected_module_found": check_expected_module(query, results)
        }
    }
    f.write(json.dumps(result) + "\n")

# Optional CSV export
def export_to_csv(jsonl_file, csv_file):
    with open(csv_file, "w", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=["timestamp", "query", "rank", "chunk_id", "score", "chapter"])
        with open(jsonl_file) as f:
            for line in f:
                record = json.loads(line)
                for result in record["results"]:
                    writer.writerow({
                        "timestamp": record["timestamp"],
                        "query": record["query"],
                        "rank": result["rank"],
                        "chunk_id": result["chunk_id"],
                        "score": result["relevance_score"],
                        "chapter": result["chapter"]
                    })
```

---

## Decision 6: Test Query Curation (Mix of Conceptual + Edge Cases)

**Decision**: Define ~10-20 curated test queries covering:
- Whole-book conceptual queries (e.g., "What is ROS 2?")
- Section-level queries with expected chapter
- Edge cases (topics not in book, single-word vs. paragraph)

**Rationale**:
- Spec identifies these test categories in user stories
- Curated queries are reproducible and manually inspectable
- Automated generation (e.g., random) would not validate domain knowledge
- Small set (10-20) is manageable for CI/CD

**Alternatives Considered**:
1. No predefined queries: Tests would be non-deterministic
2. Generate from query templates (e.g., "How does [concept] work?"): Requires NLP; over-engineered
3. Exhaustive query space: Infeasible; small representative sample sufficient

**Implementation Path**:
```python
# test_queries.py

WHOLE_BOOK_QUERIES = [
    {"query": "What is ROS 2?", "expected_module": "Module 1", "description": "Core framework question"},
    {"query": "How does sensor simulation work?", "expected_chapter": "Sensor Simulation"},
    {"query": "What is Isaac Sim?", "expected_module": "Module 3"},
    # ... 7+ more
]

EDGE_CASE_QUERIES = [
    {"query": "ROS", "description": "Single-word query"},  # Short
    {"query": "Tell me everything about ROS 2 and how it relates to robotics middleware...", "description": "Long paragraph-style"},  # Long
    {"query": "Quantum computing in robotics", "expected_result": "zero_or_low", "description": "Topic not in book"},
    # ... 3+ more
]
```

---

## Decision 7: Test Execution (No Continuous Integration, Manual Validation)

**Decision**: Run validation suite as local CLI tool; tests are pytest-based but designed for manual inspection.

**Rationale**:
- Spec requires "script-based validation (no API, no UI)"
- Results are human-readable (JSON + CSV); analysts inspect output
- Cloud API calls (Qdrant, Cohere) make CI flaky; better run against live data manually
- No automated pass/fail; validation is observational (accuracy metrics, not test assertions)

**Alternatives Considered**:
1. Automated CI pipeline: Introduces flakiness; cloud API calls non-deterministic
2. HTTP API server: Violates spec requirement "no API"
3. Dashboard/UI: Violates spec requirement "no UI"

**Implementation Path**:
```bash
# Run locally: python -m backend.retrieval --validate

# Output: retrieval_results.jsonl + optional .csv
# Human review: Check accuracy metrics, metadata validity, consistency
# Example invocation:
python -m backend.retrieval --mode validate --queries test_queries.py --output results.jsonl

# Analyze results:
# 1. Open results.jsonl in text editor or jq
# 2. Check validation.all_metadata_valid == true for all entries
# 3. Inspect top-3 manually for accuracy
# 4. Analyze latency_ms distribution (p95 < 500ms)
```

---

## Decision 8: Consistency Testing (Repeated Query Execution)

**Decision**: Execute each test query 10 times consecutively; assert identical results (same chunks, scores, order).

**Rationale**:
- Spec requires testing "consistent retrieval behavior across multiple test queries"
- Determinism is critical for production trust
- Qdrant + Cohere should be deterministic (no randomness)
- 10 repetitions sufficient to detect non-determinism

**Alternatives Considered**:
1. Run queries in parallel: Introduces timing variability; harder to debug
2. Run once: Doesn't detect non-determinism
3. Tolerance-based comparison (e.g., ±0.0001 score difference): Masks potential issues; "0% variation" requirement is strict

**Implementation Path**:
```python
def test_consistency(query: str, num_runs: int = 10):
    """Execute query N times; assert all results identical."""
    results_list = []
    for i in range(num_runs):
        results = search_chunks(query, top_k=5)
        results_list.append(results)

    # Compare all to first
    baseline = results_list[0]
    for i, results in enumerate(results_list[1:], 1):
        assert len(results) == len(baseline), f"Run {i}: count mismatch"
        for j, (baseline_r, result_r) in enumerate(zip(baseline, results)):
            assert baseline_r["chunk_id"] == result_r["chunk_id"], f"Run {i}, rank {j}: ID mismatch"
            assert baseline_r["relevance_score"] == result_r["relevance_score"], f"Run {i}, rank {j}: score mismatch"

    print(f"✅ Query '{query}' consistent across {num_runs} runs")
```

---

## No Clarifications Required

All technical decisions are grounded in:
1. Existing infrastructure (005-rag-content-ingestion)
2. Explicit spec requirements (cosine similarity, no reranking, read-only, script-based)
3. Constitutional principles (accuracy first, modular, RAG-first)
4. Industry best practices (no mocks for system validation)

**Phase 0 Complete**: Ready for Phase 1 design (data model, contracts, quickstart).
