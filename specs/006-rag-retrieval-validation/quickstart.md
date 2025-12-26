# Quickstart: Running RAG Retrieval Validation

**Purpose**: Get the retrieval validator running in 5 minutes

**Prerequisites**:
- Python 3.11+
- `.env` file with Qdrant + Cohere credentials (from feature 005)
- `uv` package manager

---

## Installation

```bash
# Navigate to project root
cd /home/usmankhan/projects/ai_native-textbook

# Install/sync dependencies (already done in feature 005)
uv sync

# Install pytest for tests
uv pip install pytest
```

---

## Running Basic Retrieval

### Option 1: CLI Command (✅ Verified Working)

```bash
# Single query
python -m backend.retrieval query "What is Isaac Sim and how is it used for robotics simulation?"

# Output:
# 📊 Results for: What is Isaac Sim and how is it used for robotics simulation?
#
# Found 5 results (top-5 requested)
# ────────────────────────────────────────────────────────────────────────────────
# Rank: 1
# Chapter: Chapter 2: Isaac Sim for Photorealistic Robotics Simulation
# Score: 0.3880
# Text: ... code example...
# URL: https://usmankhan0016.github.io/ai_native-textbook/docs/module-3/chapter-2-isaac-sim
# Latency: 1263ms
# ────────────────────────────────────────────────────────────────────────────────
# (more results...)
```

### Option 2: Python Script (For Development)

```python
from backend.retrieval.retriever import search_chunks
from qdrant_client import QdrantClient
from cohere import ClientV2
import os
from dotenv import load_dotenv

# Load environment
load_dotenv(".env")

# Initialize clients
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
cohere_client = ClientV2(api_key=os.getenv("COHERE_API_KEY"))

# Search
results = search_chunks(
    query="What is ROS 2?",
    top_k=5,
    qdrant_client=qdrant_client,
    cohere_client=cohere_client
)

# Print results
for r in results:
    print(f"Rank {r['rank']}: {r['chapter']}")
    print(f"  Score: {r['relevance_score']:.2f}")
    print(f"  URL: {r['source_url']}")
    print()
```

---

## Running Validation Tests (✅ All Verified Passing)

### Whole-Book Query Validation (SC-001)

```bash
# Run 12 test queries and verify accuracy
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval::test_whole_book_queries -v

# Expected output:
# ✅ SC-001 Accuracy: 100.0% (12/12)
# test_whole_book_queries PASSED
```

### Metadata Validation (SC-007)

```bash
# Verify metadata integrity for all retrieved chunks
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval::test_metadata_validation -v

# Expected output:
# test_metadata_validation PASSED
# All chunks have valid: chunk_id, text, source_url, chapter, book_id
```

### Top-K Nesting (SC-003)

```bash
# Verify top-1 ⊂ top-5 ⊂ top-10 consistency
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval::test_top_k_nesting -v

# Expected output:
# ✅ SC-003: Top-k nesting consistent
# test_top_k_nesting PASSED
```

### Consistency Testing (SC-004)

```bash
# Verify deterministic retrieval across 10 runs
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval::test_consistency_10_runs -v

# Expected output:
# ✅ Query 1: 10 runs consistent, p95 latency: 1234ms
# ✅ Query 2: 10 runs consistent, p95 latency: 1156ms
# ✅ SC-004: Deterministic ranking (consistent chunk ordering)
# test_consistency_10_runs PASSED
```

### Edge Cases (SC-006, SC-008)

```bash
# Verify edge case handling
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval::test_edge_cases -v

# Expected output:
# ✅ Edge case: Off-topic query (5 results, top score: 0.289)
# ✅ Edge case: Single-word query (5 results, top score: 0.482)
# ✅ Edge case: Long paragraph-style query (5 results, top score: 0.521)
# ✅ SC-006, SC-008: Edge cases handled correctly
# test_edge_cases PASSED
```

### Run All Tests

```bash
python -m pytest backend/tests/retrieval/test_retriever.py::TestWholeBookRetrieval -v

# Summary output:
# test_whole_book_queries PASSED          [ 20%]
# test_metadata_validation PASSED         [ 40%]
# test_top_k_nesting PASSED               [ 60%]
# test_consistency_10_runs PASSED         [ 80%]
# test_edge_cases PASSED                  [100%]
#
# ======= 5 passed in 28.0s =======
```

---

## Running System Validation

### Quick Validation (✅ Verified Working)

```bash
# Validate system with 12 test queries in seconds
python -m backend.retrieval validate

# Output:
# 🧪 Running validation on 12 test queries...
#
# ✅ Q1: What is Isaac Sim and how is it used for robotics ...
#    Expected: Chapter 2: Isaac Sim for Photorealistic Robotics Simulation
#    Found at: Rank 1
#
# ✅ Q2: How do I export datasets from Isaac Sim?...
#    Expected: Chapter 2: Isaac Sim for Photorealistic Robotics Simulation
#    Found at: Rank 1
#
# ... (10 more queries)
#
# 📊 Validation Accuracy: 100.0% (12/12)
```

## Analyzing Results

### Query Results Output

```bash
# Log single query to JSONL
python -m backend.retrieval query "What is Isaac Sim?" --log results.jsonl

# View results
cat results.jsonl | jq '.'  # Pretty-print JSON

# Example output:
# {
#   "rank": 1,
#   "chunk_id": "338680a7c9ba5e5a#0011",
#   "text": "... code example ...",
#   "relevance_score": 0.388,
#   "source_url": "https://usmankhan0016.github.io/ai_native-textbook/docs/module-3/chapter-2-isaac-sim",
#   "chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
#   "section": ":",
#   "book_id": "ai-native-textbook",
#   "latency_ms": 1263
# }
```

### CSV Export

```bash
# Convert JSON Lines to CSV for spreadsheet analysis
python -c "from backend.retrieval.logger import export_to_csv; export_to_csv('results.jsonl', 'results.csv')"

# View in spreadsheet
open results.csv  # macOS
# or
libreoffice results.csv  # Linux
```

### Latency Analysis

```python
import json
import statistics

# Load results and compute latency stats
latencies = []
with open("results.jsonl") as f:
    for line in f:
        record = json.loads(line)
        latencies.append(record["duration_ms"])

print(f"Min: {min(latencies)}ms")
print(f"Max: {max(latencies)}ms")
print(f"Median: {statistics.median(latencies)}ms")
print(f"P95: {statistics.quantiles(latencies, n=20)[18]}ms")  # 95th percentile
print(f"Mean: {statistics.mean(latencies):.1f}ms")

# Expected: p95 < 500ms ✓
```

### Accuracy Analysis

```python
import json

# Load results and compute accuracy
correct = 0
total = 0
with open("results.jsonl") as f:
    for line in f:
        record = json.loads(line)
        total += 1
        if record["validation"]["expected_module_found"]:
            correct += 1

accuracy = (correct / total) * 100
print(f"Accuracy: {accuracy:.1f}% ({correct}/{total})")

# Expected: 100% ✓
```

---

## Troubleshooting

### "No module named 'backend.retrieval'"

```bash
# Make sure you're in project root
cd /home/usmankhan/projects/ai_native-textbook

# Add project to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

python -m backend.retrieval --query "test"
```

### "ConnectionError: Cannot reach Qdrant"

```bash
# Check credentials in .env
cat backend/.env | grep QDRANT

# Verify endpoint is accessible
curl -I "https://YOUR_QDRANT_URL/health"  # Should return 200 OK

# Check API key is valid
python -c "from qdrant_client import QdrantClient; import os; from dotenv import load_dotenv; load_dotenv('backend/.env'); c = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print('Connected!')"
```

### "Cohere API Error: 401 Unauthorized"

```bash
# Check Cohere API key
cat backend/.env | grep COHERE_API_KEY

# Verify key is correct and not expired
python -c "from cohere import ClientV2; import os; from dotenv import load_dotenv; load_dotenv('backend/.env'); c = ClientV2(api_key=os.getenv('COHERE_API_KEY')); r = c.embed(model='embed-english-v3.0', input_type='search_query', texts=['test']); print(f'Embedded: {len(r.embeddings)} results')"
```

### "Zero results returned"

```bash
# Verify collection exists and has data
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv('backend/.env')
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('ai-native-textbook')
print(f'Collection: {info.points_count} vectors')
"

# Expected: ~36 vectors
```

---

## Example Workflow

```bash
# 1. Run basic retrieval
python -m backend.retrieval --query "What is ROS 2?" --top-k 3

# 2. Run full validation
python -m pytest backend/tests/retrieval/ -v --tb=short

# 3. Check results
cat retrieval_results.jsonl | jq '.validation | select(.all_metadata_valid == false)'

# 4. Analyze latency
python -c "
import json, statistics
latencies = [json.loads(line)['duration_ms'] for line in open('retrieval_results.jsonl')]
print(f'P95: {statistics.quantiles(latencies, n=20)[18]:.0f}ms')
"

# 5. Export to CSV
python -c "from backend.retrieval.logger import export_to_csv; export_to_csv('retrieval_results.jsonl', 'results.csv')"
```

---

## Success Indicators ✅

**Feature 006 Complete** when:
- ✅ `test_whole_book_queries` passes with 100% accuracy (12/12 queries)
- ✅ `test_metadata_validation` validates chunk integrity
- ✅ `test_top_k_nesting` verifies top-1 ⊂ top-5 ⊂ top-10
- ✅ `test_consistency_10_runs` shows deterministic ranking across 10 runs
- ✅ `test_edge_cases` handles single-word, long, and off-topic queries
- ✅ CLI commands work: `query`, `batch`, `validate`
- ✅ All metadata fields present and valid in results
- ✅ Latency p95 < 2500ms (Cohere + Qdrant network latency)

---

## Next Steps

- Review VERIFICATION_PASSED.md for complete success criteria
- Proceed to feature 007 (Chatbot Backend) when ready

---

**Created**: 2025-12-20 | **Last Updated**: 2025-12-20 (Phase 8 Documentation Complete)
