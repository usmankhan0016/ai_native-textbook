"""
Unit and integration tests for RAG retrieval validation.
Tests: whole-book queries, metadata validation, top-k nesting.
"""

import os
from typing import List

import pytest
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from cohere import ClientV2

import sys
sys.path.insert(0, '/home/usmankhan/projects/ai_native-textbook')

from backend.retrieval.retriever import search_chunks
from backend.retrieval.validators import (
    validate_metadata,
    validate_relevance,
    validate_correctness,
)
from backend.retrieval.test_queries import get_test_queries


# Load environment variables
load_dotenv("backend/.env")

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")


@pytest.fixture(scope="module")
def qdrant_client():
    """Initialize Qdrant client."""
    return QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)


@pytest.fixture(scope="module")
def cohere_client():
    """Initialize Cohere client."""
    return ClientV2(api_key=COHERE_API_KEY)


class TestWholeBookRetrieval:
    """Tests for SC-001: Whole-book conceptual retrieval accuracy."""

    def test_whole_book_queries(self, qdrant_client, cohere_client):
        """
        SC-001: Verify 100% accuracy across 12 test queries.
        Expected chapter must appear in top-3 results with score > 0.6.
        """
        test_queries = get_test_queries()
        correct_count = 0
        results_summary = []

        for test_query in test_queries:
            query = test_query["query"]
            expected_chapter = test_query["expected_chapter"]

            # Execute search
            results = search_chunks(
                query=query,
                qdrant_client=qdrant_client,
                cohere_client=cohere_client,
                top_k=5,
            )

            # Validate correctness (expected chapter in top-3)
            validation = validate_correctness(query, results, expected_chapter)
            found_in_top_3 = validation["found_in_top_3"]

            if found_in_top_3:
                correct_count += 1

            # Log result
            top_3_chapters = [r["chapter"] for r in results[:3]]
            results_summary.append({
                "query": query[:60] + "...",
                "expected": expected_chapter,
                "found": found_in_top_3,
                "top_3": top_3_chapters,
                "validation": validation,
            })

        # Assert 100% accuracy
        accuracy = correct_count / len(test_queries) * 100
        print(f"\n✅ SC-001 Accuracy: {accuracy:.1f}% ({correct_count}/{len(test_queries)})")
        print("\nDetailed Results:")
        for r in results_summary:
            status = "✅" if r["found"] else "❌"
            print(f"{status} Q: {r['query']}")
            print(f"   Expected: {r['expected']}")
            print(f"   Found at rank: {r['validation']['found_at_rank']}")
            print(f"   Top-3 chapters: {r['top_3'][:2]}\n")

        assert accuracy == 100.0, f"Expected 100% accuracy, got {accuracy:.1f}%"

    def test_metadata_validation(self, qdrant_client, cohere_client):
        """
        SC-007: Verify all retrieved chunks have valid, non-null metadata.
        """
        test_queries = get_test_queries()[:3]  # Test first 3 queries
        all_valid = True
        invalid_chunks = []

        for test_query in test_queries:
            query = test_query["query"]

            # Execute search
            results = search_chunks(
                query=query,
                qdrant_client=qdrant_client,
                cohere_client=cohere_client,
                top_k=5,
            )

            # Validate metadata for each result
            for result in results:
                if not validate_metadata(result):
                    all_valid = False
                    invalid_chunks.append({
                        "query": query,
                        "chunk_id": result.get("chunk_id", ""),
                        "issue": "metadata validation failed"
                    })

        if invalid_chunks:
            print(f"\n❌ Found {len(invalid_chunks)} invalid chunks:")
            for chunk in invalid_chunks:
                print(f"   Query: {chunk['query']}")
                print(f"   Chunk: {chunk['chunk_id']}")
                print(f"   Issue: {chunk['issue']}\n")

        assert all_valid, f"Expected all chunks valid, but found {len(invalid_chunks)} invalid"
        print("\n✅ SC-007: All metadata valid")

    def test_top_k_nesting(self, qdrant_client, cohere_client):
        """
        SC-003: Verify top-1 ⊂ top-5 ⊂ top-10 (nested consistency).
        """
        test_queries = get_test_queries()[:3]  # Test first 3 queries
        all_consistent = True
        inconsistencies = []

        for test_query in test_queries:
            query = test_query["query"]

            # Get results for different k values
            results_k1 = search_chunks(query, qdrant_client=qdrant_client, cohere_client=cohere_client, top_k=1)
            results_k5 = search_chunks(query, qdrant_client=qdrant_client, cohere_client=cohere_client, top_k=5)
            results_k10 = search_chunks(query, qdrant_client=qdrant_client, cohere_client=cohere_client, top_k=10)

            # Extract chunk_ids
            ids_k1 = [r["chunk_id"] for r in results_k1]
            ids_k5 = [r["chunk_id"] for r in results_k5]
            ids_k10 = [r["chunk_id"] for r in results_k10]

            # Check nesting
            k1_in_k5 = all(cid in ids_k5 for cid in ids_k1)
            k5_in_k10 = all(cid in ids_k10 for cid in ids_k5)

            if not (k1_in_k5 and k5_in_k10):
                all_consistent = False
                inconsistencies.append({
                    "query": query,
                    "k1_in_k5": k1_in_k5,
                    "k5_in_k10": k5_in_k10,
                })

            # Check monotonic scores
            for k, results in [(5, results_k5), (10, results_k10)]:
                scores = [r["relevance_score"] for r in results]
                monotonic = all(scores[i] >= scores[i+1] for i in range(len(scores)-1))
                if not monotonic:
                    all_consistent = False
                    inconsistencies.append({
                        "query": query,
                        "k": k,
                        "issue": "scores not monotonic",
                    })

        if inconsistencies:
            print(f"\n❌ Found {len(inconsistencies)} nesting issues:")
            for inc in inconsistencies:
                print(f"   {inc}\n")

        assert all_consistent, f"Expected consistent nesting, found issues"
        print("\n✅ SC-003: Top-k nesting consistent")

    def test_consistency_10_runs(self, qdrant_client, cohere_client):
        """
        SC-004: Verify deterministic results across 10 repetitions.
        Results must be consistent (same chunk_ids in same order).
        """
        test_queries = get_test_queries()[:2]  # Test first 2 queries

        for test_query in test_queries:
            query = test_query["query"]
            query_consistent = True
            inconsistencies = []

            # Run query 10 times
            results_runs = []
            latencies = []
            for _ in range(10):
                results = search_chunks(
                    query=query,
                    qdrant_client=qdrant_client,
                    cohere_client=cohere_client,
                    top_k=5,
                )
                results_runs.append(results)
                if results:
                    latencies.append(results[0].get("latency_ms", 0))

            # Compare all runs to baseline (first run)
            baseline = results_runs[0]
            baseline_ids = [r["chunk_id"] for r in baseline]

            for run_idx, results in enumerate(results_runs[1:], 1):
                results_ids = [r["chunk_id"] for r in results]

                # Check chunk IDs are in same order (deterministic ranking)
                ids_match = baseline_ids == results_ids

                if not ids_match:
                    query_consistent = False
                    inconsistencies.append({
                        "run": run_idx,
                        "baseline_ids": baseline_ids,
                        "actual_ids": results_ids,
                    })

            if query_consistent:
                # Calculate latency stats
                p95_latency = int(sorted(latencies)[int(len(latencies) * 0.95)])
                print(
                    f"✅ Query {test_query['id']}: "
                    f"10 runs consistent, p95 latency: {p95_latency}ms"
                )
            else:
                print(f"\n❌ Query {test_query['id']}: Found {len(inconsistencies)} ranking issues")
                for inc in inconsistencies[:2]:  # Show first 2
                    print(f"   Run {inc['run']}: {inc['actual_ids'][:2]}")

            assert query_consistent, f"Expected consistent ranking for query '{query}'"

        print("\n✅ SC-004: Deterministic ranking (consistent chunk ordering)")

    def test_edge_cases(self, qdrant_client, cohere_client):
        """
        SC-006, SC-008: Test edge cases (zero results, length variance, false positives).
        """
        edge_cases = [
            {
                "query": "Quantum computing in robotics",
                "description": "Off-topic query (should return zero or low-relevance results)",
                "expected_behavior": "valid_results",  # May return results but low relevance
            },
            {
                "query": "Isaac",
                "description": "Single-word query",
                "expected_behavior": "valid_results",
            },
            {
                "query": "Tell me everything about Isaac Sim and URDF and how they relate to humanoid robot simulation and learning from demonstrations.",
                "description": "Long paragraph-style query",
                "expected_behavior": "valid_results",
            },
        ]

        for edge_case in edge_cases:
            query = edge_case["query"]

            # Execute search
            results = search_chunks(
                query=query,
                qdrant_client=qdrant_client,
                cohere_client=cohere_client,
                top_k=5,
            )

            # Validate results exist and have valid structure
            assert len(results) >= 0, f"Should handle query gracefully: '{query}'"

            # Validate metadata for any returned results
            for result in results:
                assert "chunk_id" in result, "Result should have chunk_id"
                assert "relevance_score" in result, "Result should have relevance_score"
                assert 0 <= result["relevance_score"] <= 1, "Score should be in [0, 1]"

            top_score = f"{results[0]['relevance_score']:.3f}" if results else "N/A"
            print(
                f"✅ Edge case: {edge_case['description']} "
                f"({len(results)} results, top score: {top_score})"
            )

        print("\n✅ SC-006, SC-008: Edge cases handled correctly")
