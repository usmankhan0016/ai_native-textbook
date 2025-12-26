"""
CLI interface for RAG retrieval validation.
Supports interactive and batch query execution with logging.
"""

import argparse
import json
import os
import sys
from pathlib import Path

from dotenv import load_dotenv
from qdrant_client import QdrantClient
from cohere import ClientV2

from .retriever import search_chunks, batch_search
from .logger import log_query_result, export_to_csv, compute_accuracy


def setup_clients():
    """Initialize Qdrant and Cohere clients from environment."""
    load_dotenv()

    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_key = os.getenv('QDRANT_API_KEY')
    cohere_key = os.getenv('COHERE_API_KEY')

    if not all([qdrant_url, qdrant_key, cohere_key]):
        raise ValueError(
            "Missing required environment variables: "
            "QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY"
        )

    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_key)
    cohere_client = ClientV2(api_key=cohere_key)

    return qdrant_client, cohere_client


def cmd_query(args):
    """Execute single query and display results."""
    qdrant_client, cohere_client = setup_clients()

    results = search_chunks(
        query=args.query,
        qdrant_client=qdrant_client,
        cohere_client=cohere_client,
        top_k=args.top_k,
    )

    print(f"\n📊 Results for: {args.query}\n")
    print(f"Found {len(results)} results (top-{args.top_k} requested)")
    print("-" * 80)

    for result in results:
        print(f"Rank: {result['rank']}")
        print(f"Chapter: {result['chapter']}")
        print(f"Score: {result['relevance_score']:.4f}")
        print(f"Text: {result['text'][:100]}...")
        print(f"URL: {result['source_url']}")
        print(f"Latency: {result['latency_ms']}ms")
        print("-" * 80)

    # Log result if requested
    if args.log:
        for result in results:
            log_query_result(result, args.log)
        print(f"\n✅ Results logged to: {args.log}")


def cmd_batch(args):
    """Execute batch queries from file."""
    qdrant_client, cohere_client = setup_clients()

    # Load queries from file
    with open(args.input, 'r') as f:
        queries = [line.strip() for line in f if line.strip()]

    print(f"\n📊 Running {len(queries)} queries...")

    results = batch_search(
        queries=queries,
        qdrant_client=qdrant_client,
        cohere_client=cohere_client,
        top_k=args.top_k,
    )

    # Log results
    output_file = args.output or "retrieval_results.jsonl"
    for query_result in results:
        for result in query_result.get("results", []):
            log_query_result(result, output_file)

    print(f"✅ {len(results)} queries processed")
    print(f"📝 Results logged to: {output_file}")

    # Export to CSV if requested
    if args.csv:
        export_to_csv(output_file, args.csv)
        print(f"📊 Results exported to: {args.csv}")

    # Compute and display accuracy if expected chapters provided
    if args.accuracy:
        accuracy = compute_accuracy(output_file)
        print(f"\n📈 Accuracy Metrics:")
        print(f"  Overall: {accuracy.get('overall_accuracy', 0):.1f}%")


def cmd_validate(args):
    """Validate retrieval system with curated test queries."""
    from .test_queries import get_test_queries
    from .validators import validate_correctness

    qdrant_client, cohere_client = setup_clients()

    test_queries = get_test_queries()
    correct_count = 0

    print(f"\n🧪 Running validation on {len(test_queries)} test queries...\n")

    for test_query in test_queries:
        query = test_query["query"]
        expected_chapter = test_query["expected_chapter"]

        results = search_chunks(
            query=query,
            qdrant_client=qdrant_client,
            cohere_client=cohere_client,
            top_k=5,
        )

        validation = validate_correctness(query, results, expected_chapter)

        if validation["found_in_top_3"]:
            correct_count += 1
            status = "✅"
        else:
            status = "❌"

        print(f"{status} Q{test_query['id']}: {query[:50]}...")
        print(f"   Expected: {expected_chapter}")
        print(f"   Found at: Rank {validation['found_at_rank']}")
        print()

    accuracy = correct_count / len(test_queries) * 100
    print(f"\n📊 Validation Accuracy: {accuracy:.1f}% ({correct_count}/{len(test_queries)})")


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="RAG Retrieval Validation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # Query command
    query_parser = subparsers.add_parser("query", help="Execute single query")
    query_parser.add_argument("query", help="Query text")
    query_parser.add_argument("--top-k", type=int, default=5, help="Number of results")
    query_parser.add_argument("--log", help="Log results to JSONL file")
    query_parser.set_defaults(func=cmd_query)

    # Batch command
    batch_parser = subparsers.add_parser("batch", help="Execute batch queries")
    batch_parser.add_argument("input", help="Input file (one query per line)")
    batch_parser.add_argument("--output", help="Output JSONL file")
    batch_parser.add_argument("--csv", help="Export results to CSV")
    batch_parser.add_argument("--top-k", type=int, default=5, help="Number of results")
    batch_parser.add_argument("--accuracy", action="store_true", help="Compute accuracy")
    batch_parser.set_defaults(func=cmd_batch)

    # Validate command
    validate_parser = subparsers.add_parser("validate", help="Validate system")
    validate_parser.set_defaults(func=cmd_validate)

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    try:
        args.func(args)
        return 0
    except Exception as e:
        print(f"❌ Error: {str(e)}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
