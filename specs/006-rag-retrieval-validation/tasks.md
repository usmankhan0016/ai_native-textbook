# Feature 006: RAG Retrieval Validation - Implementation Tasks

**Feature**: RAG Retrieval Validation
**Branch**: `006-rag-retrieval-validation`
**Specification**: [spec.md](spec.md)
**Implementation Plan**: [plan.md](plan.md)
**Created**: 2025-12-20
**Status**: Ready for Implementation

---

## Overview

This tasks document defines **18 implementation tasks** organized into **5 phases**, mapping to the 4 user stories from the specification. All tasks are independently testable and follow the priority order: **P1 (whole-book retrieval) → P1 (metadata validation) → P2 (configurable top-k) → P2 (consistency testing)**.

**Success Metric**: When all tasks are complete, the feature passes **all 8 success criteria** (SC-001 through SC-008) and is ready for integration with feature 007 (chatbot backend).

---

## Phase 1: Project Setup & Infrastructure

**Goal**: Initialize project structure, configure dependencies, and prepare for core implementation

**Files Created**: 5
**Dependencies**: None (independent phase)

### Setup Tasks

- [ ] T001 Create `backend/retrieval/` module structure with `__init__.py`, `retriever.py`, `validators.py`, `test_queries.py`, `logger.py`
- [ ] T002 Create `backend/tests/retrieval/` test directory with `__init__.py`, fixtures subdirectory
- [ ] T003 Update `backend/pyproject.toml` to add test dependencies if needed (pytest, pytest-cov already present)
- [ ] T004 Create `.env` file in `backend/` with Qdrant and Cohere credentials (use `.env.example` as template)
- [ ] T005 Create `backend/retrieval/test_queries.py` with the 12 curated test queries from spec.md (chapter mappings, query text, type classification)

**Acceptance Criteria**:
- ✅ All 5 files exist with correct structure
- ✅ `test_queries.py` contains exactly 12 test queries with `query`, `expected_chapter`, and `query_type` fields
- ✅ `.env` file is git-ignored and readable by Qdrant + Cohere clients

---

## Phase 2: Core Retrieval Infrastructure (Foundational)

**Goal**: Implement the core retrieval logic and validation framework that all user stories depend on

**Files Created**: 3
**Dependencies**: Phase 1 (Setup)
**Blocking Prerequisites**: Must complete before any user story implementation

### Foundational Tasks

- [ ] T006 Implement `backend/retrieval/retriever.py::search_chunks()` function with signature:
  ```python
  def search_chunks(
      query: str,
      top_k: int = 5,
      qdrant_client: QdrantClient,
      cohere_client: ClientV2,
      metadata_filter: Optional[Dict[str, str]] = None
  ) -> List[Dict[str, Any]]
  ```
  - Embeds query using Cohere embed-english-v3.0 (input_type="search_query")
  - Searches Qdrant collection "ai-native-textbook" using cosine similarity
  - Applies optional metadata filtering if provided
  - Returns top-k results with rank, relevance_score, and all chunk metadata
  - File: `backend/retrieval/retriever.py:1-80`

- [ ] T007 Implement `backend/retrieval/validators.py` with 4 validation functions:
  - `validate_metadata(chunk: Dict) -> bool`: Check chunk_id format, text length, URL validity, book_id consistency
  - `validate_relevance(results: List[Dict]) -> Dict[str, bool]`: Check scores in [0,1] and monotonically decreasing
  - `validate_consistency(query: str, num_runs: int = 10, ...) -> Dict[str, bool]`: Run query N times, assert identical results
  - `validate_correctness(query: str, results: List[Dict], expected_chapter: str) -> Dict[str, bool]`: Check expected_chapter in top-3
  - File: `backend/retrieval/validators.py:1-150`

- [ ] T008 Implement `backend/retrieval/logger.py` with logging functions:
  - `log_query_result(result: Dict, output_file: str = "retrieval_results.jsonl")`: Append single query result to JSON Lines file
  - `export_to_csv(jsonl_file: str, csv_file: str)`: Convert JSON Lines to CSV with columns [timestamp, query, rank, chunk_id, score, chapter]
  - File: `backend/retrieval/logger.py:1-80`

**Acceptance Criteria**:
- ✅ `search_chunks()` successfully connects to live Qdrant + Cohere and returns 5+ results for "What is Isaac Sim?" query
- ✅ All 4 validators execute without errors and return expected boolean/dict structures
- ✅ Logger appends valid JSON Lines and CSV export is readable in spreadsheet applications
- ✅ No mocks; all functions use actual API clients passed as parameters

---

## Phase 3: User Story 1 - Whole-Book Conceptual Retrieval (Priority P1)

**Goal**: Implement and validate that semantic search correctly retrieves relevant chapters for broad conceptual queries

**User Story**: "An AI engineer needs to verify that semantic search correctly retrieves relevant chapters and concepts across the entire Docusaurus textbook"

**Files Created/Modified**: 2
**Dependencies**: Phase 2 (Core Infrastructure)
**Independent Test Criteria**: SC-001 (100% accuracy across 10 test queries)

### US1 Implementation Tasks

- [ ] T009 [P] [US1] Create `backend/tests/retrieval/test_retriever.py` with `test_whole_book_queries()` function:
  - Execute all 12 curated test queries from `test_queries.py`
  - For each query: Call `search_chunks(query, top_k=5)`
  - Assert expected_chapter appears in top-3 results with score > 0.6
  - Measure accuracy: count (expected_chapter found in top-3) / total queries
  - Assert accuracy == 1.0 (100%)
  - Log results with pytest output showing query → result summary
  - File: `backend/tests/retrieval/test_retriever.py:1-60`

- [ ] T010 [US1] Run `test_whole_book_queries()` against live Qdrant and verify all 12 test queries pass (SC-001 validation)
  - Execute: `pytest backend/tests/retrieval/test_retriever.py::test_whole_book_queries -v`
  - Capture output showing: each query, top-3 chapters returned, accuracy percentage
  - Assert: 100% of test queries have expected chapter in top-3
  - Document any queries that fail (if any) with reasons for triage
  - File reference: Test execution log saved to `backend/tests/retrieval/test_results_sc001.txt`

**Acceptance Criteria**:
- ✅ SC-001: 100% accuracy across 10+ test queries (expected chapter in top-3)
- ✅ All 12 test queries from spec.md Table execute successfully
- ✅ Relevance scores are in [0, 1] and decrease monotonically by rank
- ✅ No errors connecting to Qdrant; zero network timeouts

---

## Phase 4: User Story 2 - Section-Level Metadata Validation (Priority P1)

**Goal**: Validate that metadata is correctly preserved and retrievable alongside semantic content

**User Story**: "An AI engineer needs to confirm that metadata (chapter name, section, source URL) is correctly preserved and retrievable"

**Files Created/Modified**: 2
**Dependencies**: Phase 2 (Core Infrastructure), Phase 3 (US1 - metadata must be present in base retrieval)
**Independent Test Criteria**: SC-002 (100% precision on metadata filtering)

### US2 Implementation Tasks

- [ ] T011 [P] [US2] Create `backend/tests/retrieval/test_metadata_filtering.py` with `test_filter_by_chapter()` function:
  - Test 1: Query "Isaac Sim" with metadata_filter={"chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation"}
  - Test 2: Query "URDF" with metadata_filter={"chapter": "Chapter 4: URDF for Humanoid Robots"}
  - Test 3: Query "VLA" with metadata_filter={"chapter": "Chapter 4: VLA Control Architecture & Deployment"}
  - For each test: Assert all returned results have the specified chapter in metadata
  - Measure precision: count (correct chapter) / total results
  - Assert precision == 1.0 (100%)
  - File: `backend/tests/retrieval/test_metadata_filtering.py:1-50`

- [ ] T012 [P] [US2] Create `backend/tests/retrieval/test_retriever.py::test_metadata_validation()` function:
  - Execute 5 queries without filtering
  - For each result: Validate required metadata fields (chunk_id, chapter, source_url, book_id)
  - Assert chunk_id matches pattern `^[a-f0-9]{32}#\d{4}$`
  - Assert chapter is non-empty and matches actual Qdrant payload
  - Assert source_url is valid HTTPS URL pointing to ai_native-textbook domain
  - Assert book_id == "ai-native-textbook"
  - Log validation results showing pass/fail for each metadata field
  - File: `backend/tests/retrieval/test_retriever.py:60-120`

- [ ] T013 [US2] Run metadata validation tests and verify SC-002 acceptance:
  - Execute: `pytest backend/tests/retrieval/test_metadata_filtering.py -v && pytest backend/tests/retrieval/test_retriever.py::test_metadata_validation -v`
  - Assert: 100% of results have valid metadata (SC-007)
  - Assert: Filtered queries return only chunks with matching chapter (SC-002)
  - Document any metadata inconsistencies (if any) for triage
  - File reference: Test execution log saved to `backend/tests/retrieval/test_results_sc002_sc007.txt`

**Acceptance Criteria**:
- ✅ SC-002: 100% precision on metadata filtering (all filtered results match specified chapter)
- ✅ SC-007: All retrieved chunks have valid, non-null metadata
- ✅ Source URLs are correctly formatted and traceable to Docusaurus pages
- ✅ Metadata matches what's actually stored in Qdrant collection

---

## Phase 5: User Story 3 - Configurable Top-K Retrieval (Priority P2)

**Goal**: Validate that the retrieval system returns consistent results for different k values (top-1, top-5, top-10)

**User Story**: "An AI engineer needs to test that the retrieval system can return different numbers of results with consistent quality"

**Files Created/Modified**: 1
**Dependencies**: Phase 2 (Core Infrastructure)
**Independent Test Criteria**: SC-003 (consistent nested sets for top-1 ⊂ top-5 ⊂ top-10)

### US3 Implementation Tasks

- [ ] T014 [P] [US3] Create `backend/tests/retrieval/test_retriever.py::test_top_k_nesting()` function:
  - Select 5 representative queries from `test_queries.py`
  - For each query:
    - Execute search with k=1 and capture result chunk_ids
    - Execute search with k=5 and capture result chunk_ids
    - Execute search with k=10 and capture result chunk_ids
    - Assert: All chunks from k=1 are in k=5 (subset)
    - Assert: All chunks from k=5 are in k=10 (subset)
    - Assert: Relevance scores decrease monotonically within each result set
  - Log results showing nesting structure for each query
  - Assert 100% consistency across all 5 queries
  - File: `backend/tests/retrieval/test_retriever.py:120-180`

- [ ] T015 [US3] Run top-k consistency tests and verify SC-003 acceptance:
  - Execute: `pytest backend/tests/retrieval/test_retriever.py::test_top_k_nesting -v`
  - Assert: All top-k queries show proper nesting (SC-003)
  - Assert: No score inversions across all k values
  - Document any nesting violations (if any) for triage
  - File reference: Test execution log saved to `backend/tests/retrieval/test_results_sc003.txt`

**Acceptance Criteria**:
- ✅ SC-003: Top-1 ⊂ Top-5 ⊂ Top-10 (nested consistency) for 100% of queries
- ✅ Relevance scores decrease monotonically by rank
- ✅ No score inversions detected
- ✅ Configurable top_k parameter is properly passed through `search_chunks()`

---

## Phase 6: User Story 4 - Consistency Testing (Priority P2)

**Goal**: Validate that repeated identical queries return consistent results (deterministic behavior)

**User Story**: "An AI engineer needs to ensure that running the same query multiple times returns consistent results"

**Files Created/Modified**: 1
**Dependencies**: Phase 2 (Core Infrastructure)
**Independent Test Criteria**: SC-004 (0% variation across 10 runs), SC-005 (latency < 500ms p95)

### US4 Implementation Tasks

- [ ] T016 [P] [US4] Create `backend/tests/retrieval/test_consistency.py` with `test_consistency_10_runs()` function:
  - Select 3 representative queries from `test_queries.py` (Isaac Sim, URDF, VLA)
  - For each query:
    - Execute `search_chunks(query, top_k=5)` 10 times consecutively
    - Compare all 10 results to baseline (Run 1)
    - Assert: All chunk_ids are identical across all 10 runs
    - Assert: All relevance_scores are exactly equal (bitwise, no tolerance)
    - Assert: Result order is unchanged across all runs
    - Assert: No floating-point variations detected
  - Log latency for each run and compute p95
  - Assert p95 latency < 500ms for all queries
  - File: `backend/tests/retrieval/test_consistency.py:1-80`

- [ ] T017 [US4] Run consistency and latency tests and verify SC-004 and SC-005 acceptance:
  - Execute: `pytest backend/tests/retrieval/test_consistency.py::test_consistency_10_runs -v`
  - Capture latency measurements for all 30 query executions (3 queries × 10 runs)
  - Compute p95 latency across all executions
  - Assert: SC-004 (0% variation across 10 runs for each query)
  - Assert: SC-005 (p95 latency < 500ms)
  - If any latencies exceed 500ms, investigate and document
  - File reference: Test execution log with latency stats saved to `backend/tests/retrieval/test_results_sc004_sc005.txt`

**Acceptance Criteria**:
- ✅ SC-004: 0% variation across 10 repeated runs (bitwise identical scores)
- ✅ SC-005: p95 latency < 500ms across all query executions
- ✅ Deterministic behavior confirmed (same query → same results)
- ✅ No floating-point rounding differences detected

---

## Phase 7: Edge Cases & Additional Validation

**Goal**: Validate system behavior for edge cases and ensure graceful error handling

**Files Created/Modified**: 2
**Dependencies**: Phase 2 (Core Infrastructure)
**Independent Test Criteria**: SC-006 (zero false positives), SC-008 (graceful handling of zero results)

### Additional Tasks

- [ ] T018 [P] Create `backend/tests/retrieval/test_edge_cases.py` with:
  - `test_zero_results()`: Query "Quantum computing in robotics" (topic not in textbook) and assert empty results returned without error (SC-008)
  - `test_query_length_variance()`: Test single-word queries ("Isaac", "URDF") and multi-sentence queries and assert both return valid results
  - `test_false_positives()`: Execute all 12 test queries and verify irrelevant chapters do NOT appear in top-5 (SC-006)
  - `test_low_scores()`: Identify queries with relevance_score < 0.5 and verify they are correctly flagged or not returned
  - Measure response time for each edge case
  - File: `backend/tests/retrieval/test_edge_cases.py:1-100`

- [ ] T019 [P] Create `backend/retrieval/__main__.py` to enable CLI usage:
  - Support command: `python -m backend.retrieval --query "What is ROS 2?" --top-k 5`
  - Support command: `python -m backend.retrieval --mode validate --output results.jsonl` (runs all 12 test queries)
  - Parse arguments for query, top-k, metadata filters
  - Call `search_chunks()` and pretty-print JSON results
  - Ensure output matches `contracts/retrieval-schema.json`
  - File: `backend/retrieval/__main__.py:1-50`

**Acceptance Criteria**:
- ✅ SC-006: Zero false positives in top-5 across test queries
- ✅ SC-008: Zero results handled gracefully (empty array returned in <100ms)
- ✅ CLI is functional: queries can be executed from command line
- ✅ Output schema matches JSON contract

---

## Phase 8: Documentation & Final Validation

**Goal**: Create runbooks, verify all success criteria, and prepare for downstream integration

**Files Created/Modified**: 2
**Dependencies**: All implementation phases (T001-T019)

### Documentation Tasks

- [ ] T020 Create `backend/VERIFICATION_PASSED.md` documenting:
  - All 8 success criteria (SC-001 through SC-008) with test results
  - Execution timestamps and latency statistics
  - Any exceptions or deviations from spec
  - Sign-off checklist for feature completion
  - File: `backend/VERIFICATION_PASSED.md`

- [ ] T021 Update `specs/006-rag-retrieval-validation/quickstart.md` with:
  - Verified command examples that were tested against live Qdrant
  - Actual latency measurements from test runs
  - Screenshots or sample output from successful test executions
  - Troubleshooting section updated with any issues encountered
  - File: `specs/006-rag-retrieval-validation/quickstart.md`

**Acceptance Criteria**:
- ✅ All 8 success criteria are documented and have passing test results
- ✅ Quickstart guide is complete and tested
- ✅ Feature is ready for integration with feature 007 (chatbot backend)
- ✅ No outstanding blockers or known issues

---

## Success Criteria Summary

| Criterion | Implementation Task | Test Command | Expected Result |
|-----------|--------------------|--------------|-|
| SC-001: 100% accuracy | T009, T010 | `pytest test_whole_book_queries -v` | 12/12 queries correct |
| SC-002: 100% precision on filtering | T011, T013 | `pytest test_metadata_filtering -v` | 100% precision |
| SC-003: Nested top-k consistency | T014, T015 | `pytest test_top_k_nesting -v` | top-1 ⊂ top-5 ⊂ top-10 |
| SC-004: 0% variation across 10 runs | T016, T017 | `pytest test_consistency_10_runs -v` | Bitwise identical results |
| SC-005: p95 latency < 500ms | T016, T017 | Latency analysis from test output | p95 < 500ms |
| SC-006: Zero false positives | T018 | `pytest test_false_positives -v` | No irrelevant chapters in top-5 |
| SC-007: Valid metadata | T012, T013 | `pytest test_metadata_validation -v` | 100% valid |
| SC-008: Graceful zero results | T018 | `pytest test_zero_results -v` | Empty array, no error |

---

## Dependencies & Execution Order

### Dependency Graph

```
Phase 1: Setup (T001-T005)
    ↓
Phase 2: Core Infrastructure (T006-T008)
    ├→ Phase 3: US1 Whole-Book Retrieval (T009-T010)
    ├→ Phase 4: US2 Metadata Validation (T011-T013)
    ├→ Phase 5: US3 Configurable Top-K (T014-T015)
    └→ Phase 6: US4 Consistency Testing (T016-T017)
    ↓
Phase 7: Edge Cases (T018-T019)
    ↓
Phase 8: Documentation (T020-T021)
```

### Critical Path

**Minimum Tasks to Pass All Success Criteria**:
1. T001-T005 (Setup)
2. T006-T008 (Core Infrastructure)
3. T009-T010 (US1: SC-001 whole-book accuracy)
4. T011-T013 (US2: SC-002, SC-007 metadata)
5. T014-T015 (US3: SC-003 top-k nesting)
6. T016-T017 (US4: SC-004, SC-005 consistency/latency)
7. T018-T019 (Edge cases: SC-006, SC-008)

**Estimated Critical Path**: ~4-5 hours for experienced Python developer (experienced with pytest + Qdrant + Cohere API)

### Parallel Execution Opportunities

**Can run in parallel after Phase 2**:
- T009 & T011 & T014 & T016 (all user story tests can be written in parallel)
- T018 & T019 (edge cases and CLI can be written in parallel with main tests)

**Suggested MVP Scope** (minimum viable product):
- Phases 1-3 (Setup + Infrastructure + US1)
- Completes SC-001 (core accuracy validation)
- ~2 hours implementation time
- Supports downstream feature 007 integration

---

## Task Checklist Format Validation

✅ All 21 tasks follow required format: `- [ ] [TaskID] [Optional: P] [Optional: US#] Description with file path`

Examples from this document:
- ✅ `- [ ] T001 Create backend/retrieval/ module structure...` (Setup task)
- ✅ `- [ ] T009 [P] [US1] Create test_whole_book_queries()...` (Parallelizable US1 task)
- ✅ `- [ ] T006 Implement search_chunks() function...` (Foundational task)
- ✅ `- [ ] T020 Create VERIFICATION_PASSED.md...` (Documentation task)

---

## Implementation Notes

### Key Assumptions

1. **Feature 005 Complete**: Qdrant collection "ai-native-textbook" is populated with 36+ vectors with valid metadata
2. **Environment Setup**: `.env` file is configured with valid QDRANT_URL and QDRANT_API_KEY
3. **Python 3.11+**: Target Python version with required dependencies installed via `uv`
4. **Determinism Requirement**: Cosine similarity is bitwise deterministic (no floating-point rounding tolerance)
5. **No Network Mocking**: All tests execute against live Qdrant + Cohere APIs

### Potential Blockers

- **Qdrant Connectivity**: If API is unavailable, all tests will fail (not mocked)
- **Cohere API Rate Limiting**: Repeated queries may hit rate limits (implement backoff if needed)
- **Metadata Quality**: If feature 005 metadata is corrupted, SC-007 will fail (mitigation: fix 005 first)
- **Floating-Point Precision**: If Cohere embeddings vary between calls, SC-004 may fail (investigate with Cohere support)

---

**Total Tasks**: 21
**Total Phases**: 8
**Estimated Duration**: 4-6 hours
**MVP Scope**: Phases 1-3 (2 hours, SC-001 validation only)
**Full Scope**: Phases 1-8 (complete, all success criteria)

---

**Created by**: Spec-Driven Development workflow (`/sp.tasks`)
**Date**: 2025-12-20
**Status**: Ready for Implementation
