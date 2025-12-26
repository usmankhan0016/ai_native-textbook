# Feature Specification: Content Ingestion & Embeddings for RAG Chatbot

**Feature Branch**: `005-rag-content-ingestion`
**Created**: 2025-12-19
**Status**: Draft
**Input**: Content Ingestion & Embeddings for RAG Chatbot - ingest Docusaurus book content, generate embeddings using Cohere, and store vectors in Qdrant for semantic retrieval in RAG pipeline.

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Ingest Complete Book Content (Priority: P1)

An AI engineer needs to index all published content from the Docusaurus technical book into a vector database so that the RAG chatbot can retrieve relevant sections for question-answering.

**Why this priority**: This is the foundational capability—without content ingestion, no retrieval or search is possible. This is the core value of the feature.

**Independent Test**: Can be fully tested by running the ingestion script against the published Docusaurus site and verifying that all book chapters and sections are indexed in Qdrant with correct metadata.

**Acceptance Scenarios**:

1. **Given** the Docusaurus site is published and accessible, **When** the ingestion script runs, **Then** all book URLs are discovered and their main content is extracted
2. **Given** extracted content, **When** text is chunked, **Then** chunks contain 500–800 tokens each with deterministic chunk IDs
3. **Given** chunks exist, **When** embeddings are generated via Cohere, **Then** each chunk has an embedding vector stored in Qdrant with metadata (book_id, chapter, section, source_url, chunk_id)

---

### User Story 2 - Enable Idempotent Re-ingestion (Priority: P1)

An AI engineer needs to re-run the ingestion process without creating duplicate vectors or data inconsistencies, supporting updates to the book content and fixes to ingestion logic.

**Why this priority**: Critical for operational reliability and maintenance. Without idempotency, re-runs corrupt the vector database and require manual cleanup.

**Independent Test**: Can be fully tested by running ingestion twice and verifying that vector count remains the same and no duplicate embeddings exist in Qdrant.

**Acceptance Scenarios**:

1. **Given** vectors already exist in Qdrant for a section, **When** ingestion is run again for that section, **Then** existing vectors are updated (not duplicated) based on chunk_id
2. **Given** the ingestion script fails midway, **When** it is restarted, **Then** it resumes from the failure point without re-processing completed chunks
3. **Given** book content is updated, **When** ingestion is re-run, **Then** changed sections are re-embedded and old vectors are replaced

---

### User Story 3 - Support Section-Level Vector Search (Priority: P1)

An AI engineer needs vectors to be queryable by section and chapter metadata, enabling the chatbot to retrieve context for both broad (chapter-level) and narrow (section-level) questions.

**Why this priority**: Search relevance depends on proper metadata and chunk boundaries. This enables the RAG pipeline to scope retrieval correctly.

**Independent Test**: Can be fully tested by performing vector searches filtered by chapter and section metadata and verifying that results are scoped correctly.

**Acceptance Scenarios**:

1. **Given** vectors with section metadata, **When** searching for a query filtered by chapter "Chapter 3", **Then** only results from Chapter 3 are returned
2. **Given** vectors from multiple sections, **When** searching without filters, **Then** results are ranked by semantic similarity across all sections
3. **Given** a query, **When** results are retrieved, **Then** metadata (source_url, section name, chapter name) is included to help reconstruct context

---

### User Story 4 - Clean Content Extraction (Priority: P2)

An AI engineer needs extracted content to contain only the book's main text—free from navigation menus, footers, sidebars, and UI boilerplate—so embeddings reflect semantic content, not repetitive page scaffolding.

**Why this priority**: Embedding quality depends on clean input. Noisy extraction degrades search relevance and wastes token budget.

**Independent Test**: Can be fully tested by inspecting extracted content samples and verifying that navigation, sidebars, and footers are removed while chapter text is preserved.

**Acceptance Scenarios**:

1. **Given** a Docusaurus page HTML, **When** content is extracted, **Then** main article content is preserved and navigation/sidebar/footer HTML is removed
2. **Given** extracted content, **When** text is normalized, **Then** extra whitespace and HTML entities are cleaned
3. **Given** a chunk of extracted content, **When** reviewed, **Then** no navigation or page structure artifacts remain

---

### User Story 5 - Support Flexible Script Implementation (Priority: P2)

An AI engineer needs the ingestion pipeline to be implementable in either Node.js or Python, with clear documentation and examples for both, enabling teams to choose the language that fits their stack.

**Why this priority**: Flexibility reduces adoption friction and allows teams to integrate with existing infrastructure.

**Independent Test**: Can be fully tested by providing working reference implementations in both Node.js and Python that produce identical vector outputs.

**Acceptance Scenarios**:

1. **Given** a choice between Node.js and Python, **When** the engineer implements ingestion, **Then** both implementations produce identical chunk IDs and embedding results
2. **Given** the ingestion script, **When** run with the same input, **Then** output Qdrant vectors are identical regardless of implementation language

### Edge Cases

- What happens if a Docusaurus page fails to fetch (network error, 404, timeout)?
- How does the system handle pages with very long sections that require multiple chunks?
- What if a chunk_id collision occurs (same content, different section)?
- How is content handled if Docusaurus generates dynamic pages or uses client-side rendering?
- What if Cohere API rate limits are hit during embedding generation?
- How does the system handle very short pages (< 500 tokens)?
- What happens if Qdrant connection is lost during bulk insertion?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST discover all published Docusaurus book URLs from sitemap.xml or static navigation structure (static HTML, no client-side JS rendering required)
- **FR-002**: System MUST extract main article content from each Docusaurus page, removing navigation, sidebars, footers, and UI scaffolding
- **FR-003**: System MUST split extracted text into chunks of 500–800 tokens with deterministic chunk IDs based on section + sequence
- **FR-004**: System MUST support chunk overlap (recommendation: 10–15% overlap) to preserve context across chunk boundaries
- **FR-005**: System MUST generate embeddings for each chunk using Cohere's text embedding model
- **FR-006**: System MUST store embeddings in Qdrant Cloud (Free Tier) with metadata: book_id, chapter, section, source_url, chunk_id, original_text (or compressed representation)
- **FR-007**: System MUST enable vector search filtered by chapter and section metadata
- **FR-008**: System MUST implement idempotent ingestion: re-runs must not create duplicate vectors; chunks must be identified by deterministic chunk_id
- **FR-009**: System MUST handle ingestion failures gracefully: log failures, support resume-from-failure, and avoid corrupting existing vectors
- **FR-010**: System MUST provide reference implementations in Node.js or Python (or both) with clear README and usage examples
- **FR-011**: System MUST validate that all chunks are successfully embedded before marking ingestion as complete
- **FR-012**: System MUST support updating existing vectors when book content changes (e.g., typo fixes, new content)

### Key Entities

- **Book**: A Docusaurus-based technical book (e.g., "AI Native Textbook") with unique book_id, title, and base URL
- **Chapter**: A top-level organizational unit within the book (e.g., "Chapter 1: Introduction")
- **Section**: A subdivision within a chapter (e.g., "1.1 Overview", "1.2 Key Concepts")
- **Page**: A published Docusaurus web page corresponding to a section or chapter
- **Chunk**: A contiguous text segment (500–800 tokens) extracted from a page, identified by deterministic chunk_id
- **Embedding**: A dense vector representation of a chunk, generated by Cohere API
- **Vector Document**: A stored record in Qdrant containing: embedding vector, chunk_id, metadata (book_id, chapter, section, source_url, chunk text or pointer)

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: All published Docusaurus pages are discovered and indexed (100% coverage of published content)
- **SC-002**: Content extraction removes ≥95% of navigation/UI boilerplate while preserving ≥99% of main article text
- **SC-003**: Re-ingestion of the same content produces identical vectors and zero duplicate records in Qdrant
- **SC-004**: Vector search with chapter/section filters returns only results from the specified filters
- **SC-005**: Ingestion completes for a full book in ≤ 30 minutes
- **SC-006**: Chunk_id collisions are impossible (deterministic scheme ensures uniqueness for identical content in different sections)
- **SC-007**: Documentation and reference implementations enable a new engineer to run ingestion without support in ≤ 1 hour
- **SC-008**: Qdrant Free Tier storage remains under quota (vectors + metadata for full book)

## Clarifications

### Session 2025-12-19

- Q: How should the system handle Docusaurus URLs with dynamic/client-side rendered content? → A: Use static-exported HTML from published site; discover URLs via sitemap.xml or static navigation structure. Client-side JS rendering is handled by Docusaurus build process (not ingestion concern).

## Assumptions

- Docusaurus is configured with standard HTML structure (article/main content area is identifiable via common selectors)
- Book URLs are predictable and stable (URLs do not change between ingestion runs)
- Content discovery uses static HTML from the published site (sitemap.xml or static navigation); no client-side JS rendering required from the ingestion script
- Cohere API key is provisioned and valid; rate limits accommodate the book's content volume
- Qdrant Cloud account is created and accessible; Free Tier has sufficient quota for the book's vectors
- "Published" means the Docusaurus site is built and deployed (not a local development build)
- Chunk overlap of 10–15% is acceptable (standard for RAG pipelines)
- Book content is in English; no multilingual support is required at this stage

## Out of Scope

- Retrieval or ranking logic (handled by the RAG pipeline separately)
- Agent or backend orchestration (orchestration of ingestion is responsibility of the caller)
- Chat UI or frontend integration
- Session management or user-specific data storage
- Support for selected-text-only QA or query-specific filtering
- Fine-tuning embeddings or custom embedding models
- Multi-language support
- Real-time incremental ingestion (batch ingestion only)

## Constraints & Non-Functional Requirements

- **Embedding Service**: Must use Cohere's text embedding model (specified constraint)
- **Vector Database**: Must use Qdrant Cloud Free Tier (specified constraint)
- **Chunk Size**: 500–800 tokens (specified constraint)
- **Chunk Overlap**: Required (specified constraint)
- **Metadata**: Must include book_id, chapter, section, source_url, chunk_id (specified constraint)
- **Implementation**: Node.js or Python (specified constraint)
- **Idempotency**: Must support re-runs without data loss or duplication
- **Error Resilience**: Must handle network failures, API errors, and partial failures gracefully
