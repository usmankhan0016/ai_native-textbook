"""
Structured logging for RAG retrieval validation.
Outputs JSON Lines format for queryable result analysis and CSV export.
"""

import csv
import json
from datetime import datetime
from typing import Any, Dict, List


def log_query_result(
    result: Dict[str, Any],
    output_file: str = "retrieval_results.jsonl"
) -> None:
    """
    Append a single query result to JSON Lines output file.

    Args:
        result: Result dict with query, top_k, results, validation, timestamp, duration_ms
        output_file: Path to output .jsonl file
    """

    try:
        with open(output_file, "a") as f:
            f.write(json.dumps(result) + "\n")
    except Exception as e:
        print(f"❌ Failed to log result: {str(e)}")


def export_to_csv(
    jsonl_file: str,
    csv_file: str
) -> int:
    """
    Convert JSON Lines output to CSV for spreadsheet analysis.

    Columns: timestamp, query, rank, chunk_id, score, chapter, source_url

    Args:
        jsonl_file: Path to input .jsonl file
        csv_file: Path to output .csv file

    Returns:
        Number of rows written to CSV
    """

    try:
        rows_written = 0
        with open(csv_file, "w", newline="") as csvfile:
            fieldnames = [
                "timestamp",
                "query",
                "rank",
                "chunk_id",
                "relevance_score",
                "chapter",
                "source_url"
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            with open(jsonl_file, "r") as f:
                for line in f:
                    record = json.loads(line)
                    timestamp = record.get("timestamp", "")
                    query = record.get("query", "")
                    results = record.get("results", [])

                    for result in results:
                        writer.writerow({
                            "timestamp": timestamp,
                            "query": query,
                            "rank": result.get("rank", ""),
                            "chunk_id": result.get("chunk_id", ""),
                            "relevance_score": result.get("relevance_score", ""),
                            "chapter": result.get("chapter", ""),
                            "source_url": result.get("source_url", ""),
                        })
                        rows_written += 1

        return rows_written

    except Exception as e:
        print(f"❌ Failed to export CSV: {str(e)}")
        return 0


def compute_latency_stats(jsonl_file: str) -> Dict[str, float]:
    """
    Compute latency statistics from JSON Lines output.

    Args:
        jsonl_file: Path to .jsonl file with latency_ms field

    Returns:
        Dict with min, max, mean, p95 latency in milliseconds
    """

    latencies = []

    try:
        with open(jsonl_file, "r") as f:
            for line in f:
                record = json.loads(line)
                results = record.get("results", [])
                for result in results:
                    latency = result.get("latency_ms", 0)
                    if latency:
                        latencies.append(latency)

        if not latencies:
            return {
                "min_ms": 0,
                "max_ms": 0,
                "mean_ms": 0,
                "p95_ms": 0,
                "sample_size": 0
            }

        latencies.sort()
        n = len(latencies)
        p95_index = int(n * 0.95)

        return {
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "mean_ms": sum(latencies) / n,
            "p95_ms": latencies[p95_index],
            "sample_size": n,
        }

    except Exception as e:
        print(f"❌ Failed to compute latency stats: {str(e)}")
        return {}


def compute_accuracy(jsonl_file: str) -> Dict[str, float]:
    """
    Compute accuracy metrics from validation results.

    Args:
        jsonl_file: Path to .jsonl file with validation field

    Returns:
        Dict with accuracy percentage and correct/total counts
    """

    correct = 0
    total = 0

    try:
        with open(jsonl_file, "r") as f:
            for line in f:
                record = json.loads(line)
                validation = record.get("validation", {})
                if validation.get("expected_module_found"):
                    correct += 1
                total += 1

        accuracy = (correct / total * 100) if total > 0 else 0

        return {
            "accuracy_percent": accuracy,
            "correct": correct,
            "total": total,
        }

    except Exception as e:
        print(f"❌ Failed to compute accuracy: {str(e)}")
        return {}


def get_logger_functions():
    """Return dict of all logger functions for CLI access."""
    return {
        "log_query_result": log_query_result,
        "export_to_csv": export_to_csv,
        "compute_latency_stats": compute_latency_stats,
        "compute_accuracy": compute_accuracy,
    }
