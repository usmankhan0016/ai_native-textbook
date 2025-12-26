"""
Unit tests for text chunking and chunk_id determinism (T019).

Verifies that chunk_id generation is deterministic and produces
identical IDs across multiple runs for the same input.
"""

import sys
sys.path.insert(0, '/home/usmankhan/projects/ai_native-textbook/backend')

from main import chunk_text, TokenCounter
import logging


def test_chunk_id_determinism():
    """
    T019: Verify deterministic chunk_id formula produces identical IDs across runs.

    Tests that the same text from the same source URL always produces the same chunk_ids,
    ensuring idempotency across multiple ingestion runs.
    """
    logger = logging.getLogger(__name__)
    token_counter = TokenCounter(logger)

    # Sample text to chunk
    sample_text = """
    Chapter 1: Introduction to ROS 2.

    ROS 2 is a middleware platform for building robot applications.
    It provides tools and libraries for building robot software.

    Getting Started.

    ROS 2 requires Python 3.6 or higher.
    You can install ROS 2 using apt-get on Ubuntu.
    Or you can compile from source using colcon.

    Basic Concepts.

    A ROS 2 node is the basic building block of a ROS 2 application.
    Each node can publish and subscribe to topics.
    Topics are used for asynchronous communication between nodes.
    Services are used for synchronous request-reply communication.
    """ * 5  # Repeat to generate multiple chunks

    source_url = "https://example.com/chapter-1-intro"

    # Generate chunks first time
    chunks_run_1 = chunk_text(sample_text, source_url, token_counter=token_counter, logger=logger)
    chunk_ids_run_1 = [c["chunk_id"] for c in chunks_run_1]

    # Generate chunks second time (same input)
    chunks_run_2 = chunk_text(sample_text, source_url, token_counter=token_counter, logger=logger)
    chunk_ids_run_2 = [c["chunk_id"] for c in chunks_run_2]

    # Verify: same count
    assert len(chunks_run_1) == len(chunks_run_2), \
        f"Chunk count mismatch: {len(chunks_run_1)} vs {len(chunks_run_2)}"

    # Verify: identical IDs in same order
    assert chunk_ids_run_1 == chunk_ids_run_2, \
        f"Chunk IDs differ:\n  Run 1: {chunk_ids_run_1}\n  Run 2: {chunk_ids_run_2}"

    # Verify: format of chunk_id (sha256_hash#sequence)
    for i, chunk_id in enumerate(chunk_ids_run_1):
        parts = chunk_id.split("#")
        assert len(parts) == 2, f"Invalid chunk_id format: {chunk_id}"
        hash_part, seq_part = parts
        assert len(hash_part) == 16, f"Invalid hash length: {hash_part}"
        assert seq_part.isdigit(), f"Invalid sequence number: {seq_part}"
        assert int(seq_part) == i, f"Sequence number mismatch: {seq_part} vs {i}"

    print(f"✅ T019 PASS: Chunk IDs deterministic across {len(chunks_run_1)} chunks")
    for i, chunk in enumerate(chunks_run_1):
        print(f"   Chunk {i}: {chunk['chunk_id']} ({chunk['tokens']} tokens)")

    return True


def test_chunk_stability_with_different_urls():
    """
    Verify that different URLs produce different chunk_ids (no collisions).
    """
    logger = logging.getLogger(__name__)
    token_counter = TokenCounter(logger)

    sample_text = """
    Same content repeated.
    """ * 20  # Generate multiple chunks

    url_1 = "https://example.com/page-1"
    url_2 = "https://example.com/page-2"

    chunks_1 = chunk_text(sample_text, url_1, token_counter=token_counter, logger=logger)
    chunks_2 = chunk_text(sample_text, url_2, token_counter=token_counter, logger=logger)

    ids_1 = [c["chunk_id"] for c in chunks_1]
    ids_2 = [c["chunk_id"] for c in chunks_2]

    # Different URLs should produce different chunk_ids
    assert ids_1 != ids_2, f"Same chunk_ids from different URLs:\n  {ids_1}\n  {ids_2}"

    # But same structure (same count)
    assert len(chunks_1) == len(chunks_2), \
        f"Chunk count differs for same text: {len(chunks_1)} vs {len(chunks_2)}"

    print(f"✅ URL uniqueness test PASS: Different URLs produce different chunk_ids")
    return True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("\n" + "="*80)
    print("Running T019: Chunk ID Determinism Tests")
    print("="*80)

    test_chunk_id_determinism()
    test_chunk_stability_with_different_urls()

    print("\n" + "="*80)
    print("✅ All T019 tests PASSED")
    print("="*80 + "\n")
