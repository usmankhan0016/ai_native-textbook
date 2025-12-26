---
id: 004
title: Create Implementation Plan for RAG Ingestion
stage: plan
date: 2025-12-19
surface: agent
model: claude-haiku-4-5-20251001
feature: 005-rag-content-ingestion
branch: 005-rag-content-ingestion
user: Usman Khan
command: /sp.plan
labels: ["plan","architecture","python","qdrant","cohere","docusaurus"]
links:
  spec: /specs/005-rag-content-ingestion/spec.md
  plan: /specs/005-rag-content-ingestion/plan.md
  adr: null
  pr: null
files:
  - specs/005-rag-content-ingestion/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

**Planning Request** (`/sp.plan`): Create implementation plan for RAG content ingestion feature with specific requirements:

```
- Set up initial backend structure (create backend folder, initialize UV package)
- Fetch all book URLs via /sitemap.xml from deployed Docusaurus site
- Extract clean main content, chunk text with metadata, generate Cohere embeddings
- Create Qdrant collection and store chunk embeddings with deterministic IDs
- Validate ingestion by logging chunk counts and successful vector storage
- Single file (main.py) with system design functions orchestrated by main()
- System design: get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant
```

## Response snapshot

**Plan created**: Comprehensive implementation plan with Phase 0 research and Phase 1 design completed.

**Key architectural decisions documented**:

1. **URL Discovery**: `sitemap.xml` parsing (static HTML, no client-side JS)
2. **Content Extraction**: BeautifulSoup4 targeting Docusaurus `<article>` semantic elements
3. **Chunking**: Token-based (500–800 tokens) with 10% overlap; deterministic chunk_id (`{source_url}#{sequence}`)
4. **Embeddings**: Cohere API with batch processing (~100 req/min); 1024-dim vectors
5. **Storage**: Qdrant Cloud Free Tier with upsert logic (idempotent re-ingestion)
6. **Error Handling**: Exponential backoff for transient failures; graceful degradation

**Function Signatures** documented for all 6 primary functions + orchestrating main()

**Data Model** defined:
- Chunk entity: chunk_id, source_url, book_id, chapter, section, text, tokens, created_at
- VectorDocument: id (chunk_id), vector (1024-dim), payload (metadata)
- Qdrant schema with upsert-based idempotency

**Project Structure** established:
- Backend folder with `main.py`, `pyproject.toml`, `.env.example`, `requirements.txt`
- Test folder with `test_ingestion.py`, `test_chunking.py`

**Constitution Alignment**: All 3 relevant principles satisfied (IV: RAG-First, VI: Industry Alignment, VIII: Infrastructure)

## Outcome

- ✅ Impact: Detailed technical architecture enables immediate implementation. All dependencies identified, APIs specified, data model frozen.
- 🧪 Tests: N/A (planning phase)
- 📁 Files: 1 file updated (plan.md)
- 🔁 Next prompts: `/sp.tasks` for implementation task breakdown
- 🧠 Reflection: Single-file architecture per user request simplifies deployment while maintaining modularity. Deterministic chunk IDs + Qdrant upsert ensure production-grade idempotency. Free-tier services (Cohere, Qdrant) well-suited for textbook scale.

## Evaluation notes (flywheel)

- Failure modes observed: None; user requirements clearly specified, plan maps cleanly to architecture
- Graders run and results (PASS/FAIL): Constitution Check PASS; all gates satisfied
- Prompt variant (if applicable): N/A
- Next experiment: Validate performance estimate (30 min SLA) during implementation; may need tuning based on actual API response times
