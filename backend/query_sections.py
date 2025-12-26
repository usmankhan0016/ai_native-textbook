#!/usr/bin/env python3
"""
Query Qdrant collection from feature 005 and extract distinct section names.
This ensures test queries map exactly to stored metadata.
"""

import os
import json
from collections import defaultdict
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv(".env")

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_api_key:
    print("ERROR: QDRANT_URL and QDRANT_API_KEY must be set in .env")
    exit(1)

print(f"🔗 Connecting to Qdrant: {qdrant_url}")
client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

# Get collection info
collection_name = "ai-native-textbook"
print(f"📦 Retrieving collection: {collection_name}")

try:
    collection_info = client.get_collection(collection_name)
    total_points = collection_info.points_count
    print(f"✅ Collection found with {total_points} vectors")
except Exception as e:
    print(f"❌ Error accessing collection: {e}")
    exit(1)

# Scroll through all points and extract metadata
print("\n🔍 Extracting metadata from all vectors...\n")

sections_by_chapter = defaultdict(set)
all_sections = set()
chunk_count = 0

try:
    # Use scroll with limit to get all points
    points, _ = client.scroll(
        collection_name=collection_name,
        limit=1000,
        with_payload=True,
        with_vectors=False
    )

    for point in points:
        payload = point.payload
        chunk_count += 1

        chapter = payload.get("chapter", "MISSING")
        section = payload.get("section", "NONE")

        sections_by_chapter[chapter].add(section)
        if section != "NONE":
            all_sections.add(section)

    print(f"✅ Processed {chunk_count} chunks\n")

except Exception as e:
    print(f"❌ Error scrolling collection: {e}")
    import traceback
    traceback.print_exc()
    exit(1)

# Display results
print("=" * 80)
print(f"DISTINCT SECTIONS IN QDRANT COLLECTION ({collection_name})")
print("=" * 80)

print(f"\n📊 Total unique sections: {len(all_sections)}")
print(f"📊 Total chunks with sections: {sum(1 for s in all_sections if s != 'NONE')}")
print(f"📊 Total chunks without sections: {chunk_count - len([s for ch_secs in sections_by_chapter.values() for s in ch_secs if s != 'NONE'])}")

print(f"\n📚 Sections organized by chapter:\n")

# Sort by chapter name
for chapter in sorted(sections_by_chapter.keys()):
    sections = sorted(sections_by_chapter[chapter])
    print(f"  {chapter}:")
    for section in sections:
        if section == "NONE":
            print(f"    - [NO SECTION]")
        else:
            print(f"    - {section}")
    print()

# Display as JSON for easy copying
print("\n" + "=" * 80)
print("SECTIONS AS JSON (for test query curation):")
print("=" * 80)
print(json.dumps({
    "total_sections": len(all_sections),
    "sections_by_chapter": {
        chapter: sorted(list(sections))
        for chapter, sections in sorted(sections_by_chapter.items())
    }
}, indent=2))

print("\n✅ Query complete. Use section names above to curate test queries.")
