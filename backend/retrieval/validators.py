"""
Validation functions for RAG retrieval accuracy, metadata integrity, and consistency.
"""

import re
from typing import Any, Dict, List, Optional

from qdrant_client import QdrantClient
from cohere import ClientV2

from .retriever import search_chunks


def is_valid_content_chunk(chunk: Dict[str, Any], min_relevance: float = 0.15) -> bool:
    """
    Filter out junk/garbage chunks that shouldn't be displayed as sources.

    Filters out:
    - Navigation metadata (Tags: A, sidebar items, etc.)
    - Empty or very short text chunks
    - Chunks with only whitespace or special characters
    - Low relevance chunks below threshold
    - Chunks that are pure code without context
    - Malformed chapter titles

    Args:
        chunk: Chunk dict with metadata fields
        min_relevance: Minimum relevance score (default 0.15 = 15%)

    Returns:
        True if chunk is valid content, False if junk
    """
    # Must have basic fields
    if not chunk.get("text") or not chunk.get("chapter"):
        return False

    text = chunk.get("text", "").strip()
    chapter = chunk.get("chapter", "").strip()
    relevance = chunk.get("relevance_score", 0.0)

    # Filter by relevance score
    if relevance < min_relevance:
        return False

    # Filter empty or very short text (less than 20 chars is likely junk)
    if len(text) < 20:
        return False

    # Filter navigation/UI junk patterns
    junk_patterns = [
        r"^Tags:\s*[A-Z]$",  # "Tags: A"
        r"^Sidebar$",
        r"^Navigation$",
        r"^Previous\s*\|\s*Next$",
        r"^\s*\|\s*\|\s*$",  # Table borders
        r"^-+$",  # Dashes only
        r"^\*+$",  # Asterisks only
        r"^=+$",  # Equals signs only
        r"^>+$",  # Quote markers only
    ]

    for pattern in junk_patterns:
        if re.match(pattern, text, re.IGNORECASE):
            return False

    # Filter malformed/empty chapter titles
    invalid_chapters = [
        "unknown", "undefined", "null", "tags:", "sidebar", "navigation"
    ]
    if chapter.lower() in invalid_chapters or len(chapter) < 3:
        return False

    # Filter pure code chunks using multiple heuristics
    # Heuristic 1: High ratio of code characters
    code_chars = sum(1 for c in text if c in "(){}[];,._")
    if len(text) > 0 and (code_chars / len(text)) > 0.5:
        return False

    # Heuristic 2: Contains multiple consecutive function calls pattern
    # Pattern: word ( ) word ( ) or word . word ( )
    function_call_pattern = r'(\w+\s*\(\s*\)\s*){2,}|\w+\s*\.\s*\w+\s*\(\s*\)'
    if re.search(function_call_pattern, text):
        # If it has function calls, check if it has explanatory text
        # Remove code patterns and see how much is left
        cleaned = re.sub(r'\w+\s*\(\s*\)', '', text)
        cleaned = re.sub(r'\w+\s*\.\s*\w+', '', cleaned)
        cleaned = re.sub(r'[(){}[\];,._]+', '', cleaned)
        cleaned = cleaned.strip()
        # If less than 30% remains after removing code, it's mostly code
        if len(cleaned) < len(text) * 0.3:
            return False

    # Filter chunks that are just whitespace or special chars
    if not re.search(r'[a-zA-Z]{3,}', text):
        return False

    return True


def filter_valid_chunks(chunks: List[Dict[str, Any]], min_relevance: float = 0.15, max_results: int = 5) -> List[Dict[str, Any]]:
    """
    Filter chunk list to remove junk and return only high-quality sources.

    Args:
        chunks: List of chunk dicts from retrieval
        min_relevance: Minimum relevance score threshold (default 0.15 = 15%)
        max_results: Maximum number of sources to return (default 5)

    Returns:
        Filtered list of valid chunks, limited to max_results
    """
    valid_chunks = [
        chunk for chunk in chunks
        if is_valid_content_chunk(chunk, min_relevance=min_relevance)
    ]

    # Return top N results
    return valid_chunks[:max_results]


def validate_metadata(chunk: Dict[str, Any]) -> bool:
    """
    Validate chunk metadata fields for required format and non-null values.

    Checks:
    - chunk_id matches pattern: ^[a-f0-9]{32}#\\d{4}$
    - text is non-empty (length > 10 chars)
    - source_url is valid HTTPS URL pointing to ai_native-textbook domain
    - chapter is non-empty
    - book_id == "ai-native-textbook"

    Args:
        chunk: Chunk dict with metadata fields

    Returns:
        True if all validations pass, False otherwise
    """

    # Check required fields exist and are non-null
    required_fields = ["chunk_id", "text", "source_url", "chapter", "book_id"]
    for field in required_fields:
        if field not in chunk or chunk[field] is None:
            return False

    # Validate chunk_id format (16 hex digits + # + 4 decimal digits)
    chunk_id_pattern = r"^[a-f0-9]{16}#\d{4}$"
    if not re.match(chunk_id_pattern, chunk["chunk_id"]):
        return False

    # Validate text length
    if len(chunk.get("text", "")) < 10:
        return False

    # Validate source_url
    source_url = chunk.get("source_url", "")
    url_pattern = r"^https://.*ai_native-textbook.*"
    if not re.match(url_pattern, source_url):
        return False

    # Validate chapter is non-empty
    if not chunk.get("chapter", "").strip():
        return False

    # Validate book_id
    if chunk.get("book_id") != "ai-native-textbook":
        return False

    return True


def validate_relevance(results: List[Dict[str, Any]]) -> Dict[str, bool]:
    """
    Validate relevance scores are in valid range and monotonically decreasing by rank.

    Checks:
    - All scores in [0, 1]
    - Scores decrease monotonically (score[i] >= score[i+1])
    - Top score is "meaningful" (> 0.6)

    Args:
        results: List of result dicts with 'relevance_score' field

    Returns:
        Dict with keys:
        - scores_in_range: bool
        - scores_monotonic: bool
        - top_score_meaningful: bool
    """

    if not results:
        return {
            "scores_in_range": True,
            "scores_monotonic": True,
            "top_score_meaningful": True,
        }

    scores = [r.get("relevance_score", 0) for r in results]

    # Check all scores are in [0, 1]
    scores_in_range = all(0 <= s <= 1 for s in scores)

    # Check scores decrease monotonically
    scores_monotonic = all(
        scores[i] >= scores[i + 1] for i in range(len(scores) - 1)
    )

    # Check top score is meaningful
    top_score_meaningful = scores[0] > 0.6 if scores else True

    return {
        "scores_in_range": scores_in_range,
        "scores_monotonic": scores_monotonic,
        "top_score_meaningful": top_score_meaningful,
    }


def validate_consistency(
    query: str,
    num_runs: int = 10,
    qdrant_client: Optional[QdrantClient] = None,
    cohere_client: Optional[ClientV2] = None,
    top_k: int = 5,
) -> Dict[str, Any]:
    """
    Execute query N times and verify results are bitwise identical (0% variation).

    Args:
        query: Query string to test
        num_runs: Number of repetitions (default 10)
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere ClientV2
        top_k: Number of results per query

    Returns:
        Dict with keys:
        - fully_consistent: bool (all results identical)
        - num_runs: int
        - variation: float (0.0 for 100% identical)
        - latencies: List[int] (latency per run in ms)
    """

    if not qdrant_client or not cohere_client:
        return {
            "fully_consistent": False,
            "num_runs": 0,
            "error": "Missing Qdrant or Cohere client"
        }

    results_list = []
    latencies = []

    for _ in range(num_runs):
        try:
            results = search_chunks(
                query=query,
                top_k=top_k,
                qdrant_client=qdrant_client,
                cohere_client=cohere_client,
            )
            results_list.append(results)
            if results:
                latencies.append(results[0].get("latency_ms", 0))
        except Exception as e:
            return {
                "fully_consistent": False,
                "num_runs": num_runs,
                "error": f"Query execution failed: {str(e)}"
            }

    # Compare all results to first run (baseline)
    baseline = results_list[0]
    all_consistent = True

    for i, results in enumerate(results_list[1:], 1):
        # Check count matches
        if len(results) != len(baseline):
            all_consistent = False
            break

        # Check each result chunk_id and score match exactly
        for j, (baseline_r, result_r) in enumerate(zip(baseline, results)):
            if baseline_r["chunk_id"] != result_r["chunk_id"]:
                all_consistent = False
                break
            # Exact floating-point comparison (no tolerance)
            if baseline_r["relevance_score"] != result_r["relevance_score"]:
                all_consistent = False
                break
        if not all_consistent:
            break

    return {
        "fully_consistent": all_consistent,
        "num_runs": num_runs,
        "variation": 0.0 if all_consistent else 1.0,
        "latencies": latencies,
    }


def validate_correctness(
    query: str,
    results: List[Dict[str, Any]],
    expected_chapter: str,
) -> Dict[str, Any]:
    """
    Validate that expected chapter appears in top-3 results.

    Args:
        query: The query string
        results: List of result dicts
        expected_chapter: Expected chapter name (must match chunk metadata)

    Returns:
        Dict with keys:
        - expected_chapter: str
        - found_in_top_3: bool
        - found_at_rank: Optional[int] (rank where found, 1-indexed)
        - found_chapters: List[str] (chapters in top-3)
    """

    top_3_results = results[:3]
    found_chapters = [r.get("chapter", "") for r in top_3_results]

    found_in_top_3 = any(expected_chapter in r.get("chapter", "") for r in top_3_results)

    found_at_rank = None
    if found_in_top_3:
        for rank, r in enumerate(top_3_results, 1):
            if expected_chapter in r.get("chapter", ""):
                found_at_rank = rank
                break

    return {
        "expected_chapter": expected_chapter,
        "found_in_top_3": found_in_top_3,
        "found_at_rank": found_at_rank,
        "found_chapters": found_chapters,
    }
