"""
Core retrieval logic for RAG validation.
Implements cosine similarity search with Qdrant + Cohere embeddings.
"""

import time
from typing import Any, Dict, List, Optional

from cohere import ClientV2
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue


def search_chunks(
    query: str,
    qdrant_client: QdrantClient,
    cohere_client: ClientV2,
    top_k: int = 5,
    metadata_filter: Optional[Dict[str, str]] = None,
) -> List[Dict[str, Any]]:
    """
    Search for semantically similar chunks in Qdrant using cosine similarity.

    Args:
        query: User query text (1-1000 characters)
        top_k: Number of results to return (1-100, default 5)
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere ClientV2
        metadata_filter: Optional metadata filter (e.g., {"chapter": "Module 1"})

    Returns:
        List of dicts with: rank, chunk_id, text, relevance_score, source_url,
        chapter, section, book_id, latency_ms

    Raises:
        ValueError: If query is empty or top_k is invalid
        Exception: If Qdrant or Cohere API calls fail
    """

    # Validate inputs
    if not query or not query.strip():
        raise ValueError("Query cannot be empty")
    if top_k < 1 or top_k > 100:
        raise ValueError(f"top_k must be between 1 and 100, got {top_k}")

    collection_name = "ai-native-textbook"
    start_time = time.time()

    # Step 1: Embed query using Cohere
    try:
        embed_response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",
            texts=[query]
        )
        # Extract embedding from Cohere response
        # Response structure: EmbedByTypeResponse.embeddings.float_ = list of embeddings
        query_embedding = list(embed_response.embeddings.float_[0])
    except Exception as e:
        raise Exception(f"Cohere embedding failed: {str(e)}")

    # Step 2: Search Qdrant with optional metadata filtering
    try:
        # Build optional filter
        query_filter = None
        if metadata_filter:
            # Example: {"chapter": "Module 1"}
            key, value = list(metadata_filter.items())[0]
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    )
                ]
            )

        # Execute cosine similarity search using query_points
        search_results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            query_filter=query_filter,
            limit=top_k,
            with_payload=True,
        )
    except Exception as e:
        raise Exception(f"Qdrant search failed: {str(e)}")

    # Step 3: Format results with metadata
    end_time = time.time()
    latency_ms = int((end_time - start_time) * 1000)

    results = []

    # query_points returns QueryResponse with points
    scored_points = search_results.points if hasattr(search_results, 'points') else search_results

    for rank, scored_point in enumerate(scored_points, start=1):
        payload = scored_point.payload or {}

        result = {
            "rank": rank,
            "chunk_id": payload.get("chunk_id", ""),
            "text": payload.get("text", "")[:500],  # Truncate to 500 chars
            "relevance_score": float(scored_point.similarity) if hasattr(scored_point, 'similarity') else float(scored_point.score) if hasattr(scored_point, 'score') else 0.0,
            "source_url": payload.get("source_url", ""),
            "chapter": payload.get("chapter", ""),
            "section": payload.get("section", ""),
            "book_id": payload.get("book_id", "ai-native-textbook"),
            "latency_ms": latency_ms,
        }
        results.append(result)

    return results


def batch_search(
    queries: List[str],
    qdrant_client: QdrantClient,
    cohere_client: ClientV2,
    top_k: int = 5,
) -> List[Dict[str, Any]]:
    """
    Execute multiple queries and return aggregated results.

    Args:
        queries: List of query strings
        qdrant_client: Initialized Qdrant client
        cohere_client: Initialized Cohere ClientV2
        top_k: Number of results per query

    Returns:
        List of result dicts, one per query with "query" and "results" fields
    """

    batch_results = []
    for query in queries:
        try:
            results = search_chunks(
                query=query,
                qdrant_client=qdrant_client,
                cohere_client=cohere_client,
                top_k=top_k,
            )
            batch_results.append({
                "query": query,
                "results": results,
                "total_results": len(results)
            })
        except Exception as e:
            batch_results.append({
                "query": query,
                "error": str(e),
                "results": []
            })

    return batch_results
