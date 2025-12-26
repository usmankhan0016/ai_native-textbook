# Tasks: Content Ingestion & Embeddings for RAG Chatbot

**Input**: Design documents from `/specs/005-rag-content-ingestion/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md, data-model.md, contracts/ (optional)

**Tests**: Tests are OPTIONAL - NOT included in this task list unless integration validation is needed.

**Organization**: Tasks grouped by user story to enable independent implementation and testing. Core ingestion pipeline (US1) is MVP; idempotency (US2) and search (US3) extend the foundation.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no hard dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4, US5)
- **Exact file paths**: `backend/main.py`, `backend/tests/`, etc.

## Path Conventions

Per plan.md, structure is:
```
backend/
├── main.py                    # PRIMARY: Single-file ingestion script
├── pyproject.toml             # UV package manifest
├── .env.example               # Required environment variables
├── requirements.txt           # Frozen dependencies
└── tests/
    ├── test_ingestion.py      # Integration tests
    └── test_chunking.py       # Unit tests
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency management

- [x] T001 Create backend folder structure (backend/, backend/tests/)
- [x] T002 Initialize UV project with pyproject.toml (Python 3.10+, dependencies: requests, beautifulsoup4, qdrant-client, cohere, python-dotenv, tiktoken)
- [x] T003 [P] Create .env.example with required variables (DOCUSAURUS_BASE_URL, DOCUSAURUS_BOOK_ID, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, LOG_LEVEL)
- [x] T004 [P] Create requirements.txt for frozen dependency pinning (uv export --no-header --no-emit-project)
- [x] T005 [P] Create README.md with setup instructions, CLI usage, and quickstart

**Checkpoint**: ✅ Backend project structure initialized, dependencies managed, documentation ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure shared by all user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Implement logging configuration in backend/main.py (INFO level by default, controllable via LOG_LEVEL env var)
- [x] T007 [P] Implement HTTP client utility in backend/main.py with exponential backoff (base 5s, max 3 retries)
- [x] T008 [P] Implement error handling strategy (custom exception classes for network, rate limit, Qdrant errors)
- [x] T009 Implement token counting utility using tiktoken for validation (tokens in range 500–800)
- [x] T010 [P] Load and validate environment variables from .env (required keys present, APIs accessible)

**Checkpoint**: ✅ Foundation ready - all utility functions in place, error handling strategy defined, logging configured

---

## Phase 3: User Story 1 - Ingest Complete Book Content (Priority: P1) 🎯 MVP

**Goal**: Discover all Docusaurus book URLs, extract content, chunk text, generate embeddings, store in Qdrant with full metadata

**Independent Test**: Run ingestion script against published Docusaurus site → verify all URLs discovered, chunks generated, vectors stored in Qdrant with correct metadata (chunk_id, source_url, book_id, chapter, section)

### Implementation for User Story 1

- [x] T011 [P] [US1] Implement `get_all_urls(base_url: str) -> list[str]` function in backend/main.py
  - Fetch `{base_url}/sitemap.xml`
  - Parse XML, extract all `<loc>` URLs
  - Filter to book-specific pages (exclude homepage, admin, etc.)
  - Validate URLs are absolute and accessible
  - Log discovered count

- [x] T012 [P] [US1] Implement `extract_text_from_url(url: str) -> str` function in backend/main.py
  - Fetch page HTML using HTTP client (with retry/backoff)
  - Target `<article>` or `<main>` semantic elements
  - Remove Docusaurus boilerplate: `.docSidebar`, `.toc`, `.pagination`, navbar, footer
  - Normalize whitespace and HTML entities
  - Return clean text; log extraction success/skip

- [x] T013 [P] [US1] Implement `chunk_text(text: str, source_url: str, chunk_size: int = 500) -> list[dict]` function in backend/main.py
  - Split text into overlapping chunks (500–800 tokens, 10% overlap)
  - Generate deterministic chunk_id as `f"{sha256(source_url)}#{sequence_number}"`
  - Validate token count (warn if <500 or >800)
  - Return list of `{chunk_id, text, tokens, source_url}`
  - Log chunk generation summary

- [x] T014 [P] [US1] Implement `embed(texts: list[str], cohere_api_key: str) -> list[list[float]]` function in backend/main.py
  - Use Cohere batch embedding API (cohere-english-v3.0 model)
  - Implement rate limiting: max 100 req/min, batch texts appropriately
  - Catch 429 errors, implement exponential backoff + sleep
  - Return list of 1024-dimensional embeddings
  - Log batch completion (X texts → X embeddings)

- [x] T015 [P] [US1] Implement `create_collection(qdrant_client, collection_name: str, vector_size: int = 1024) -> None` function in backend/main.py
  - Create collection with name `{book_id}` (idempotent - skip if exists)
  - Configure vector size 1024, distance metric: cosine
  - Define payload schema: chunk_id (keyword), source_url (text), chapter (text), section (text), book_id (keyword), text (text)
  - Log collection creation/skip

- [x] T016 [P] [US1] Implement `save_chunk_to_qdrant(qdrant_client, collection_name: str, chunk: dict, embedding: list[float]) -> None` function in backend/main.py
  - Upsert point to Qdrant: id = chunk_id (deterministic), vector = embedding, payload = chunk metadata
  - Handle upsert (update if exists, insert if new) - critical for idempotency
  - Catch Qdrant errors, implement retry logic
  - Log upsert success/skip

- [x] T017 [US1] Implement `main(base_url: str, book_id: str, cohere_api_key: str, qdrant_url: str, qdrant_api_key: str) -> None` function in backend/main.py
  - Load environment variables (or use function parameters)
  - Initialize Qdrant client and Cohere client
  - Orchestrate pipeline: get_all_urls → extract_text_from_url → chunk_text → embed → create_collection → save_chunk_to_qdrant
  - Batch processing: collect chunks, batch embed (50-100 at a time to manage rate limits)
  - Log progress: discovered URLs, extracted pages, generated chunks, embedded batches, stored vectors
  - Log final summary: total time, chunk count, vector count, success/failure

- [x] T018 [US1] Add main entry point to backend/main.py (if __name__ == '__main__')
  - Parse command-line arguments or read from environment
  - Call main() with required parameters
  - Exit with status code (0 on success, 1 on error)

**Checkpoint**: ✅ User Story 1 complete - Full ingestion pipeline functional, all vectors stored in Qdrant, MVP ready for testing

---

## Phase 4: User Story 2 - Enable Idempotent Re-ingestion (Priority: P1)

**Goal**: Ensure re-runs of ingestion do not create duplicate vectors; support resume-from-failure and content updates

**Independent Test**: Run ingestion twice with same input → verify vector count identical, no duplicates; run ingestion after intentional failure → verify resume from failure point

### Implementation for User Story 2

- [x] T019 [US2] Implement idempotency in chunk_id generation
  - Verify deterministic chunk_id formula (`sha256(source_url)#{sequence}`) produces identical IDs across runs
  - Update `chunk_text()` to ensure stable ordering
  - Add unit test: same text → same chunk IDs (backend/tests/test_chunking.py)
  - Document chunk_id stability assumption

- [x] T020 [US2] Implement upsert-based vector storage in `save_chunk_to_qdrant()`
  - Verify Qdrant SDK upsert() method updates existing points by ID (chunk_id)
  - Test: insert vector → re-insert same vector → verify count unchanged
  - Add integration test in backend/tests/test_ingestion.py: re-ingest → verify no duplicates

- [x] T021 [US2] Implement resume-from-failure logic in main()
  - Track completed pages/chunks in memory or temporary file
  - On failure, log last successful URL and chunk_id
  - Document recovery procedure: manual re-run will skip completed chunks (upsert handles it)
  - Add warning message: "Partial ingestion detected, resuming..."

- [x] T022 [US2] Add integrity validation after ingestion
  - Query Qdrant for total vectors in collection
  - Verify count matches expected (logged chunk count)
  - Log mismatch warning if counts differ
  - Add function: `validate_ingestion(qdrant_client, collection_name: str, expected_count: int) -> bool`

- [x] T023 [US2] Document content update procedure
  - Explain that re-running ingestion with updated source automatically updates vectors (upsert)
  - Document re-ingestion SLA: should complete in ≤30 minutes
  - Add note: old chunks for deleted pages remain in Qdrant (manual cleanup may be needed)

**Checkpoint**: ✅ User Story 2 complete - Idempotency guaranteed, re-ingestion safe, content updates supported

---

## Phase 5: User Story 3 - Support Section-Level Vector Search (Priority: P1)

**Goal**: Enable vector search filtered by chapter and section metadata; retrieve context with full metadata

**Independent Test**: Store vectors with section metadata → search with chapter filter → verify results scoped correctly; retrieve vector with metadata → verify source_url, chapter, section populated

### Implementation for User Story 3

- [x] T024 [US3] Extract and store chapter/section metadata in chunks
  - Update `extract_text_from_url()` to extract chapter from breadcrumb or H1 header
  - Update `chunk_text()` to extract section from H2 header (closest preceding)
  - Add to chunk dict: `chapter`, `section`
  - Update `save_chunk_to_qdrant()` to include chapter and section in payload
  - Test: extract from real Docusaurus page → verify chapter/section populated

- [x] T025 [US3] Implement vector search helper function in backend/main.py
  - Add function: `search_vectors(qdrant_client, collection_name: str, query_embedding: list[float], chapter_filter: str = None, limit: int = 5) -> list[dict]`
  - Search by embedding similarity
  - Apply metadata filters (chapter) if provided
  - Return list of `{chunk_id, source_url, chapter, section, text, similarity_score}`
  - Log search results count

- [x] T026 [US3] Add metadata payload validation in `save_chunk_to_qdrant()`
  - Ensure all required metadata fields present: chunk_id, source_url, book_id, chapter, section
  - Warn if optional fields missing (book_id can be inferred)
  - Reject vectors with missing critical metadata
  - Log validation results

- [x] T027 [US3] Document search capabilities in README
  - Example: search by semantic similarity
  - Example: search with chapter filter ("Chapter 3" → results from Chapter 3 only)
  - Explain metadata fields returned with each result
  - Show sample output format

**Checkpoint**: ✅ User Story 3 complete - Metadata captured, search filtering enabled, context retrieval fully functional

---

## Phase 6: User Story 4 - Clean Content Extraction (Priority: P2)

**Goal**: Verify extracted content removes boilerplate (≥95%) while preserving article text (≥99%); no nav/footer artifacts remain

**Independent Test**: Extract from real Docusaurus page → manual inspection of sample → verify no navigation, sidebars, footers, pagination remain; article text intact

### Implementation for User Story 4

- [ ] T028 [P] [US4] Enhance `extract_text_from_url()` with Docusaurus-specific selectors
  - Target: `<article class="docusaurus_content">` or `<main role="main">`
  - Remove: `.docSidebar`, `.tableOfContents`, `.pagination`, `nav`, `footer`, `header` (with exceptions for article-level headers)
  - Test against actual AI Native Textbook Docusaurus pages
  - Measure: extract page → compare before/after character count
  - Document boilerplate removal pattern

- [ ] T029 [US4] Add HTML entity normalization in `extract_text_from_url()`
  - Decode HTML entities: `&nbsp;` → space, `&mdash;` → —, etc.
  - Normalize whitespace: multiple spaces → single space, leading/trailing → strip
  - Preserve code blocks as-is (keep indentation)
  - Test: extract page with code → verify code indentation preserved

- [ ] T030 [US4] Create sample extraction validation test
  - In backend/tests/test_ingestion.py: Test extract_text_from_url() on real Docusaurus page
  - Verify no CSS classes, no `<script>` tags, no nav elements in output
  - Measure percentage of boilerplate removed
  - Document baseline quality metrics

- [ ] T031 [US4] Add manual validation workflow in README
  - Document process: inspect extracted text samples manually
  - Run extraction on a few pages, save to temp files
  - Visual inspection: does output look like clean article text?
  - Success: no artifacts, full article content preserved

**Checkpoint**: User Story 4 complete - Content extraction verified clean, boilerplate removed, article text intact

---

## Phase 7: User Story 5 - Support Flexible Script Implementation (Priority: P2)

**Goal**: Provide production-grade Python reference implementation; document so Node.js version can be built independently

**Independent Test**: Run main.py → success; read quickstart.md → developer can set up and run independently; document function contracts so Node.js impl is straightforward

### Implementation for User Story 5

- [ ] T032 [US5] Document all function contracts in backend/main.py
  - Add docstrings to every function: inputs, outputs, exceptions, side effects
  - Example: `def get_all_urls(base_url: str) -> list[str]: """Fetch sitemap, return all book URLs. Raises: HTTPError, XMLParseError"""`
  - Document rate limits (Cohere ~100 req/min), batch size recommendations
  - Document schema expectations (chunk_id format, payload fields)

- [ ] T033 [US5] Create comprehensive quickstart.md in specs/005-rag-content-ingestion/
  - Setup: UV install, .env configuration
  - Example .env with real values
  - Running ingestion: `uv run backend/main.py` or `python backend/main.py`
  - Expected output: progress logs, final summary
  - Troubleshooting: common errors (API key invalid, Qdrant connection failed, rate limits)
  - Time estimate: ~20-30 minutes for full textbook ingestion
  - Cost estimate: Cohere free tier, Qdrant free tier

- [ ] T034 [US5] Add type hints to all functions in backend/main.py
  - Python 3.10+ style: `def func(x: str) -> list[dict]:`
  - Use typing.List, Dict, Optional, Union where needed
  - Example: `def chunk_text(...) -> list[dict[str, Union[str, int]]]:`
  - Benefits: enables autocomplete, type checking, IDE support

- [ ] T035 [US5] Create data contract documentation for Node.js implementation
  - Document Chunk entity: field names, types, constraints (chunk_id format, tokens 500–800)
  - Document Qdrant payload schema: exact field names and types
  - Document function input/output contracts
  - Document Cohere API version and embedding dimension (1024)
  - Document error handling expectations (rate limits, network retries)
  - File: specs/005-rag-content-ingestion/data-model.md

- [ ] T036 [P] [US5] Create reference Node.js stub in backend/ (optional)
  - skeleton Node.js script showing function structure (no implementation, just stubs)
  - demonstrates how to structure Node.js version to match Python contracts
  - File: backend/main.js (commented out or in separate node/ folder)

**Checkpoint**: User Story 5 complete - Python implementation documented, contracts defined, Node.js implementation path clear

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, documentation, and quality improvements

- [ ] T037 [P] Run full end-to-end ingestion on AI Native Textbook production site
  - Ingest all chapters from https://ai-native-textbook.vercel.app
  - Verify all pages discovered, content extracted, chunks generated, vectors stored
  - Measure ingestion time; verify ≤30 minutes (SC-005)
  - Validate vector count: expect ~10k-50k vectors depending on book size

- [ ] T038 [P] Run idempotency test
  - Ingest once: record vector count (should be N)
  - Ingest again: verify vector count still N (no duplicates)
  - Ingest third time with one page updated: verify updated page's vectors changed, others unchanged

- [ ] T039 [P] Run search validation test
  - Store vectors from full ingestion
  - Query: "How do I create a ROS 2 node?" → search for similar chunks
  - Verify results from correct chapters (e.g., Chapter 1 if asking about ROS 2 basics)
  - Apply chapter filter: search only in "Module 1" → verify results scoped correctly

- [ ] T040 Test error resilience
  - Simulate network failure during URL fetch → verify retry and recovery
  - Simulate Cohere API rate limit (429) → verify backoff and retry
  - Simulate Qdrant connection loss → verify graceful degradation or re-connection
  - Document error scenarios and recovery behavior

- [ ] T041 [P] Update documentation in specs/005-rag-content-ingestion/
  - Finalize README.md with tested commands and expected output
  - Finalize quickstart.md with validated setup steps
  - Add troubleshooting section with real error messages and solutions
  - Add performance notes: typical ingestion time, resource usage

- [ ] T042 Code cleanup and documentation
  - Remove debug logging, add production-grade logging
  - Ensure all functions have docstrings
  - Format code with consistent style (black, ruff)
  - Add inline comments for complex logic (token counting, chunk ID generation)

- [ ] T043 Create deployment guide
  - Document how to run on Linux/macOS (target platform per plan.md)
  - Document how to containerize in Docker (optional, for future)
  - Document how to schedule recurring ingestion (cron job example)

- [ ] T044 Validate against specification success criteria
  - SC-001: All published pages discovered and indexed? ✓
  - SC-002: Boilerplate removal ≥95%, article preservation ≥99%? ✓
  - SC-003: Re-ingestion produces zero duplicates? ✓
  - SC-004: Chapter/section filtering works? ✓
  - SC-005: Ingestion ≤30 minutes? ✓
  - SC-006: Chunk_id collisions impossible? ✓
  - SC-007: Setup in ≤1 hour possible? ✓
  - SC-008: Qdrant Free Tier quota not exceeded? ✓

**Checkpoint**: All user stories tested, documentation complete, specification criteria validated, ready for release

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories 1-5 (Phases 3-7)**: All depend on Foundational completion
  - US1, US2, US3 are P1 (core, highest priority)
  - US4, US5 are P2 (quality, documentation enhancements)
  - Stories can proceed in parallel once Foundational completes
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (Ingest Content)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (Idempotency)**: Can start after Foundational - Builds on US1 but independently testable
- **User Story 3 (Search)**: Can start after Foundational - Builds on US1 but independently testable
- **User Story 4 (Content Quality)**: Can start after Foundational - Enhances US1 but independently testable
- **User Story 5 (Flexible Implementation)**: Can start after all others - Documents US1 contracts

### Within Each User Story

- Models/data structures before services/functions
- Core functions before orchestration (main)
- Implementation before testing/validation
- Validation before documentation

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003, T004, T005 all [P] → run in parallel

**Phase 2 (Foundational)**:
- T007, T008, T010 all [P] → run in parallel

**Phase 3 (US1 - MVP)**:
- T011, T012, T013, T014, T015, T016 all [P] → can run in parallel (different functions)
- T017 depends on all above (orchestration)
- T018 depends on T017

**Phase 3-7 (All Stories)**:
- Once Foundational complete, all user stories can proceed in parallel
- Developer A: US1
- Developer B: US2
- Developer C: US3
- etc.

**Phase 8 (Polish)**:
- T037, T038, T039, T041, T042, T043 all [P] → run in parallel
- T040, T044 sequential (validation checkpoints)

---

## Parallel Example: Phase 3 (User Story 1)

```bash
# Can launch all function implementations in parallel:
Task: "Implement get_all_urls() in backend/main.py"
Task: "Implement extract_text_from_url() in backend/main.py"
Task: "Implement chunk_text() in backend/main.py"
Task: "Implement embed() in backend/main.py"
Task: "Implement create_collection() in backend/main.py"
Task: "Implement save_chunk_to_qdrant() in backend/main.py"

# Then orchestrate with main():
Task: "Implement main() and entry point in backend/main.py" (depends on all above)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

**Stop here to validate core ingestion:**

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (T011–T018)
4. **STOP and TEST**: Run ingestion on live Docusaurus site
5. Verify: all URLs discovered, all chunks generated, all vectors stored
6. If successful: Deploy / Demo MVP

### Recommended Incremental Delivery

1. **Sprint 1**: Complete Setup + Foundational + US1 → **MVP ready**
2. **Sprint 2**: Add US2 (Idempotency) → **Production-grade re-ingestion**
3. **Sprint 3**: Add US3 (Search) + US4 (Quality) → **Full RAG-ready ingestion**
4. **Sprint 4**: Add US5 (Documentation) + Polish → **Release v1.0**

### Single Developer Timeline

- Phase 1 (Setup): ~30 min
- Phase 2 (Foundational): ~1 hour
- Phase 3 (US1): ~3-4 hours (with testing)
- **Subtotal: ~5-6 hours for MVP**
- Phase 4 (US2): ~1 hour
- Phase 5 (US3): ~1 hour
- Phase 6 (US4): ~1 hour
- Phase 7 (US5): ~2 hours
- Phase 8 (Polish): ~2 hours
- **Total: ~12-14 hours for complete feature**

### Parallel Team Strategy (3 developers)

1. All: Setup + Foundational (1 hour)
2. Dev A: US1 (3 hours) | Dev B: US2 (1 hour) | Dev C: US3 (1 hour) → **overlap 3 hours**
3. All: US4, US5, Polish (2 hours)
4. **Total: ~6-7 wall-clock hours**

---

## Notes

- All [P] tasks can run in parallel (different files, no hard dependencies within phase)
- [Story] label enables traceability to spec requirements
- Each user story independently completable and testable
- Commit after each logical group (e.g., after implementing a function)
- Use Git commits to mark phase completion checkpoints
- Avoid: vague task descriptions, same-file task conflicts, cross-story blocking dependencies
- MVP validation: if US1 works, the core pipeline is proven; US2-US5 enhance but don't break MVP
