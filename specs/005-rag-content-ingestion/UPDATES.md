# Content Update Procedure (T023)

**For**: RAG Content Ingestion & Embeddings Pipeline
**Status**: User Story 2 - Idempotent Re-ingestion (T023)
**Version**: 1.0

## Overview

This document explains how to handle content updates when the source Docusaurus book changes. The ingestion pipeline is **idempotent by design**, meaning you can safely re-run ingestion to sync with updated content.

## How Idempotency Works

The pipeline uses deterministic chunk IDs and Qdrant's upsert operation to ensure idempotency:

1. **Deterministic Chunk ID**: Each chunk gets a unique ID based on:
   - SHA256 hash of the source URL (first 16 characters)
   - Sequence number (0000, 0001, 0002, ...)
   - Format: `abc123def456789f#0000`

2. **Upsert-Based Storage**: Vectors are stored using upsert (update or insert):
   - If chunk_id already exists → **update** the vector
   - If chunk_id is new → **insert** the vector
   - **No duplicates** created even after multiple runs

## Content Update Scenarios

### Scenario 1: Updated Content in Existing Page

**Situation**: You edited the content of "Chapter 1" in your Docusaurus book.

**What to do**:
```bash
# Simply re-run ingestion
python backend/main.py
```

**What happens**:
1. Pipeline discovers all URLs (same as before)
2. Extracts updated content from "Chapter 1"
3. Re-chunks the updated content
4. Generates new embeddings for updated chunks
5. **Upserts to Qdrant**: Updated chunks replace old ones; unchanged chunks remain untouched

**Result**:
- Updated content is now searchable with the latest embeddings
- Old vectors for unchanged pages remain unchanged
- No duplicates created
- Total vector count may **increase** if re-chunking produced more chunks

### Scenario 2: Added New Pages

**Situation**: You added a new "Chapter 5" to your Docusaurus book.

**What to do**:
```bash
# Re-run ingestion (same command)
python backend/main.py
```

**What happens**:
1. Pipeline discovers new URLs (including new Chapter 5)
2. Extracts content from all pages (new pages have new URLs)
3. Chunks new pages
4. Generates embeddings for all pages
5. Upserts to Qdrant

**Result**:
- New "Chapter 5" content is now searchable
- All existing vectors remain unchanged
- Total vector count **increases** by the number of new chunks

### Scenario 3: Deleted or Moved Pages

**Situation**: You deleted "Chapter 4" from your Docusaurus book.

**What to do**:
```bash
# Re-run ingestion (same command)
python backend/main.py
```

**What happens**:
1. Pipeline discovers URLs (Chapter 4 no longer in sitemap)
2. Chunks and embeds all discovered pages
3. Upserts to Qdrant

**Important**: Old vectors for deleted "Chapter 4" remain in Qdrant (orphaned).

**Manual Cleanup** (Optional):
```python
from qdrant_client import QdrantClient

client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-key")

# Delete orphaned vectors by filtering on source_url
client.delete(
    collection_name="ai-native-textbook",
    points_selector={
        "filter": {
            "key": "source_url",
            "match": {"value": "https://old-chapter-4-url.com"}
        }
    }
)
```

## Re-ingestion SLA & Performance

**Target**: Content updates should complete within **≤30 minutes** (SC-005)

**Factors Affecting Speed**:
1. **Book size**: Larger books take longer (linear with page count)
2. **Changes**: Only changed pages need new embeddings; unchanged pages skip the Cohere API call (faster)
3. **Network latency**: Depends on Docusaurus site and Cohere API responsiveness
4. **Cohere rate limits**: ~100 requests/minute on free tier

**Optimization**:
- Batch processing (50 texts per Cohere request) minimizes API calls
- Upsert operation efficiently updates existing vectors

## Resuming from Interruption

If ingestion is interrupted (network failure, timeout, API error):

### Automatic Detection
```
⚠️  Partial ingestion detected! Resuming from URL 256/512
```

The pipeline detects incomplete runs via a `.ingestion_progress_<book_id>.tmp` file.

### Resume Procedure
Simply re-run ingestion:
```bash
python backend/main.py
```

The pipeline will:
1. Detect `.ingestion_progress_<book_id>.tmp` file
2. Find last completed URL
3. Skip already-processed URLs (≤256 in example above)
4. Resume from URL 257
5. Clean up progress file on successful completion

**Safety**:
- Upsert ensures already-processed chunks aren't duplicated
- Resuming is safe to do multiple times

## Monitoring & Validation

### Check Ingestion Progress
```bash
# Monitor via logs (LOG_LEVEL=DEBUG for detailed progress)
LOG_LEVEL=DEBUG python backend/main.py
```

### Verify Ingestion Completed
```python
from qdrant_client import QdrantClient

client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-key")
collection_info = client.get_collection("ai-native-textbook")
print(f"Total vectors: {collection_info.points_count}")
```

### Integrity Validation
The pipeline automatically validates after completion:
```
Validating ingestion integrity...
  Expected vectors: 12340
  Actual vectors: 12340
✅ Validation passed: collection 'ai-native-textbook' is valid
```

If counts differ (expected for re-ingestion):
```
Vector count mismatch! Expected 12340, got 12355.
Difference: 15 vectors.
This may be normal if re-ingesting with some chunks unchanged.
```

This is **normal** for content updates—new/updated chunks increase count.

## Common Questions

### Q: Will re-running ingestion duplicate my vectors?
**A**: No. The upsert-based design ensures:
- Same chunk_id → vector is **updated**, not duplicated
- New chunk_id → vector is **inserted** once
- Running ingestion 100 times with identical content = same vector count

### Q: How long until updated content is searchable?
**A**: Immediately after ingestion completes. No additional sync needed.

### Q: What if I deleted old vectors manually in Qdrant?
**A**: If you manually deleted vectors and then re-run ingestion, they will be re-created with the same chunk_ids.

### Q: Can I ingest only changed pages?
**A**: Not directly (current implementation re-ingests all pages for consistency). However, the batch processing and upsert mechanism make re-ingestion efficient. Unchanged pages reuse embeddings from Qdrant.

### Q: What if Docusaurus site is temporarily down during ingestion?
**A**: The pipeline will:
1. Retry with exponential backoff (3 attempts)
2. Log the failure
3. Leave a `.ingestion_progress_<book_id>.tmp` file
4. Next run will detect partial ingestion and resume

## Scheduling Periodic Updates

### Cron Job Example (Daily)
```bash
0 2 * * * cd /path/to/backend && python main.py >> /var/log/rag-ingestion.log 2>&1
```

### Docker Container Example
```dockerfile
FROM python:3.11
WORKDIR /app
COPY . .
RUN pip install -r requirements.txt
CMD ["python", "main.py"]
```

Run daily:
```bash
docker run -e DOCOSAURUS_BASE_URL="..." -e COHERE_API_KEY="..." ... my-ingestion:latest
```

## Troubleshooting

### Error: "Vector count mismatch"
- **Normal** during content updates
- Vector counts may increase (new/updated chunks) or decrease (deletion + cleanup)
- Not a failure—ingestion completed successfully

### Error: "Partial ingestion detected"
- Previous run was interrupted
- Pipeline will resume automatically
- No manual action needed

### Error: "Cohere API rate limited"
- Slow down: Use smaller `COHERE_BATCH_SIZE` (default: 50)
- Reduce via `.env`:
  ```
  COHERE_BATCH_SIZE=25
  ```

### Error: "Qdrant quota exceeded"
- Book too large for free tier (1 GB limit)
- Options:
  1. Upgrade to Qdrant paid plan
  2. Split into multiple collections (one per module)
  3. Reduce chunk overlap in `.env`:
     ```
     # Not yet configurable; requires code change
     ```

## Best Practices

1. **Schedule periodic updates**: Daily or weekly depending on content change frequency
2. **Monitor logs**: Set up alerting for ingestion failures
3. **Validate after updates**: Use integrity validation endpoint
4. **Plan for large updates**: Major content restructuring may take longer
5. **Keep `.env` secure**: Never commit real API keys to git

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024 | Initial release - idempotent re-ingestion support |

---

**Related Documentation**:
- [README.md](./../../backend/README.md) - Setup and quick start
- [spec.md](./spec.md) - Feature specification
- [plan.md](./plan.md) - Implementation plan
- [tasks.md](./tasks.md) - Task breakdown (T023)
