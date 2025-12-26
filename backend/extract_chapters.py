#!/usr/bin/env python3
"""Extract clean chapter information from Qdrant collection."""

import os
import json
from collections import defaultdict
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv(".env")

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_api_key:
    print("ERROR: Credentials missing")
    exit(1)

client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
collection_name = "ai-native-textbook"

print(f"🔗 Querying {collection_name} collection...\n")

points, _ = client.scroll(
    collection_name=collection_name,
    limit=1000,
    with_payload=True,
    with_vectors=False
)

chapters_set = set()
chapter_to_chunks = defaultdict(list)

for point in points:
    payload = point.payload
    chapter = payload.get("chapter", "UNKNOWN")
    chunk_id = payload.get("chunk_id", "")
    text = payload.get("text", "")[:100]  # First 100 chars

    chapters_set.add(chapter)
    chapter_to_chunks[chapter].append({
        "chunk_id": chunk_id,
        "text_preview": text
    })

print("=" * 80)
print(f"CHAPTERS IN QDRANT COLLECTION")
print("=" * 80)
print(f"\nTotal distinct chapters: {len(chapters_set)}\n")

for i, chapter in enumerate(sorted(chapters_set), 1):
    chunks = chapter_to_chunks[chapter]
    print(f"{i}. {chapter}")
    print(f"   Chunks: {len(chunks)}")
    for chunk in chunks[:2]:  # Show first 2 chunks
        print(f"     - {chunk['chunk_id']}: {chunk['text_preview'][:60]}...")
    print()

print("\n" + "=" * 80)
print("CHAPTERS AS JSON")
print("=" * 80)
print(json.dumps({
    "total_chapters": len(chapters_set),
    "chapters": sorted(list(chapters_set))
}, indent=2))

print("\n✅ Ready to curate test queries based on these chapters")
