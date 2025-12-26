"""Dual-mode routing logic for RAG agent (Feature 007).

This module implements:
- Whole-book mode: Retrieve from Qdrant + generate answer
- Selected-text mode: Answer using only user-selected text
"""

import os
import logging
import time
from typing import List, Dict, Any, Tuple
import asyncpg
from google import genai

from .prompts import format_whole_book_prompt, format_selected_text_prompt
import re

logger = logging.getLogger(__name__)


def clean_text_preview(text: str, max_length: int = 150) -> str:
    """Clean text preview by removing code snippets and formatting for display.

    Args:
        text: Raw text from chunk
        max_length: Maximum preview length (default 150)

    Returns:
        Clean preview text suitable for source display
    """
    # Remove excessive whitespace
    text = re.sub(r'\s+', ' ', text).strip()

    # Remove code patterns that shouldn't appear in source preview
    # Pattern: function_name ( ) or object . method ( )
    text = re.sub(r'\b\w+\s*\(\s*\)', '', text)
    text = re.sub(r'\b\w+\s*\.\s*\w+\s*\(\s*\)', '', text)

    # Remove common code artifacts
    text = re.sub(r'[(){}[\];]+', ' ', text)  # Remove brackets/braces
    text = re.sub(r'\s+', ' ', text).strip()  # Clean up spaces again

    # Remove markdown/code formatting
    text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
    text = re.sub(r'`[^`]+`', '', text)

    # Truncate to max length, breaking at word boundary
    if len(text) > max_length:
        text = text[:max_length]
        # Find last space to avoid cutting mid-word
        last_space = text.rfind(' ')
        if last_space > max_length * 0.7:  # Only if we're not cutting too much
            text = text[:last_space]
        text = text.strip() + '...'

    return text


def classify_query_type(query: str) -> str:
    """Classify query to determine if RAG retrieval is needed.

    Args:
        query: User's question

    Returns:
        One of: 'greeting', 'chitchat', 'needs_rag'
    """
    query_lower = query.lower().strip()

    # Greetings - no RAG needed
    greetings = ['hi', 'hello', 'hey', 'good morning', 'good afternoon', 'good evening', 'greetings', 'howdy']
    if query_lower in greetings or (len(query_lower) < 10 and any(g in query_lower for g in ['hi', 'hello', 'hey'])):
        return 'greeting'

    # Chitchat - no RAG needed
    chitchat = ['thanks', 'thank you', 'bye', 'goodbye', 'ok', 'okay', 'cool', 'nice', 'great', 'awesome', 'perfect']
    if query_lower in chitchat:
        return 'chitchat'

    # Everything else needs RAG
    return 'needs_rag'


def handle_simple_response(query: str, query_type: str) -> str:
    """Handle greetings and chitchat without RAG retrieval.

    Args:
        query: User's question
        query_type: Type of query ('greeting' or 'chitchat')

    Returns:
        Simple response message
    """
    query_lower = query.lower().strip()

    if query_type == 'greeting':
        return "Hello! I'm your AI-Native Robotics Textbook Assistant. I can help you understand concepts from the textbook, explain code examples, and answer questions about robotics and AI. What would you like to learn about?"

    if query_type == 'chitchat':
        if any(thanks in query_lower for thanks in ['thanks', 'thank you']):
            return "You're welcome! Feel free to ask any questions about the textbook content."

        if any(bye in query_lower for bye in ['bye', 'goodbye']):
            return "Goodbye! Come back anytime you have questions about robotics and AI."

        return "I'm here to help with questions about the AI-Native Robotics textbook. What would you like to know?"

    return "I'm here to help with questions about this textbook. What would you like to know?"


def classify_query_intent(query: str) -> str:
    """Classify user query intent to optimize retrieval strategy (for RAG queries only).

    Args:
        query: User's question

    Returns:
        One of: 'overview', 'specific', 'code', 'conceptual'
    """
    query_lower = query.lower()

    # Overview questions (book structure, purpose, introduction)
    overview_keywords = [
        "about", "overview", "introduction", "summary", "purpose",
        "what is this book", "textbook about", "covers what", "topics covered",
        "book covers", "main topics", "learning objectives", "target audience"
    ]

    # Code-related questions
    code_keywords = [
        "code", "example", "syntax", "function", "class", "implement",
        "program", "script", "snippet", "how to write"
    ]

    # Check for overview questions first
    if any(keyword in query_lower for keyword in overview_keywords):
        return "overview"

    # Check for code questions
    if any(keyword in query_lower for keyword in code_keywords):
        return "code"

    # Check for specific vs conceptual
    specific_indicators = ["what is", "define", "explain", "how does", "why"]
    if any(indicator in query_lower for indicator in specific_indicators):
        return "conceptual"

    # Default to specific query
    return "specific"


async def route_whole_book_query(
    query: str,
    session_id: str,
    db_pool: asyncpg.Pool,
    gemini_client: genai.Client
) -> Tuple[str, List[Dict[str, Any]], Dict[str, Any]]:
    """Route whole-book query through Qdrant retrieval + Gemini generation.

    Process:
    1. Check if query is simple greeting/chitchat (no RAG needed)
    2. For book questions: retrieve from Qdrant + generate answer
    3. Return response with sources (empty for greetings/chitchat)

    Args:
        query: User's question
        session_id: Session UUID for conversation history
        db_pool: Database connection pool
        gemini_client: Gemini API client

    Returns:
        Tuple of (response_text, sources, metadata)
        - response_text: Answer (simple for greetings, RAG-based for book questions)
        - sources: List of retrieval sources (empty for greetings/chitchat)
        - metadata: Dict with latency_ms, query_type, and other metrics

    Raises:
        Exception: If Qdrant or Gemini API fails
    """
    start_time = time.time()

    # TODO: Load chat history for multi-turn conversations (Phase 5)
    # For now, treat each query independently

    # Step 0: Classify query type (greeting/chitchat vs needs RAG)
    query_type = classify_query_type(query)
    logger.info(f"Query type classified as: {query_type}")

    # Handle greetings and chitchat without RAG
    if query_type in ['greeting', 'chitchat']:
        response_text = handle_simple_response(query, query_type)
        latency_ms = int((time.time() - start_time) * 1000)

        metadata = {
            "latency_ms": latency_ms,
            "query_type": query_type,
            "retrieval_count": 0,
            "rag_used": False
        }

        logger.info(f"Simple query handled in {latency_ms}ms without RAG")
        return response_text, [], metadata

    # Step 1: Classify query intent for RAG optimization
    intent = classify_query_intent(query)
    logger.info(f"Query intent classified as: {intent}")

    # Step 2: Determine retrieval parameters based on intent
    if intent == "overview":
        # Overview questions: fewer, higher-quality results from intro chapters
        top_k = 8  # More results for comprehensive overview
        score_threshold = 0.50  # Lower threshold to ensure results
    elif intent == "code":
        # Code questions: more results to get complete examples
        top_k = 10
        score_threshold = 0.55
    else:
        # Specific/conceptual: balanced retrieval
        top_k = 7
        score_threshold = 0.50

    logger.info(f"Retrieval params: top_k={top_k}, score_threshold={score_threshold}")

    # Step 3: Retrieve relevant chapters from Qdrant
    logger.info(f"Retrieving from Qdrant for query: {query[:50]}...")

    try:
        from retrieval.retriever import search_chunks
        from qdrant_client import QdrantClient
        import cohere

        # Initialize clients (reuse from Feature 006)
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        cohere_api_key = os.getenv("COHERE_API_KEY")
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "ai-native-textbook")

        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        cohere_client = cohere.ClientV2(api_key=cohere_api_key)

        # Search Qdrant with intent-based parameters
        all_results = search_chunks(
            query=query,
            qdrant_client=qdrant_client,
            cohere_client=cohere_client,
            top_k=top_k,
        )

        # Filter by score threshold
        filtered_results = [r for r in all_results if r.get("relevance_score", 0.0) >= score_threshold]

        # Apply content validation to remove junk chunks
        from retrieval.validators import filter_valid_chunks
        valid_results = filter_valid_chunks(
            filtered_results,
            min_relevance=score_threshold,
            max_results=5  # Limit to top 5 sources
        )

        # Safety: If filtering removes everything, try with lower threshold
        if len(valid_results) == 0 and len(all_results) > 0:
            logger.warning(f"Validation filtered out all results. Trying with lower threshold.")
            valid_results = filter_valid_chunks(
                all_results,
                min_relevance=0.10,  # Lower threshold for fallback
                max_results=3
            )

        results = valid_results
        retrieval_count = len(results)
        logger.info(f"Retrieved {retrieval_count} valid chunks from Qdrant (from {len(all_results)} total, {len(filtered_results)} after score filter)")

        # Step 2: Format retrieved content for prompt (for agent)
        retrieved_content = ""
        sources = []

        for i, result in enumerate(results, 1):
            # Extract metadata from Qdrant payload
            chapter_title = result.get("chapter", "Unknown Chapter")
            section_title = result.get("section", "")
            text = result.get("text", "")
            score = result.get("relevance_score", 0.0)
            source_url = result.get("source_url", "")

            # Format content for agent prompt (include text for synthesis)
            retrieved_content += f"\n\n--- Source {i}: {chapter_title}"
            if section_title:
                retrieved_content += f" - {section_title}"
            retrieved_content += f" ---\n{text}\n"

            # Build CLEAN sources list for frontend display
            # Clean the text preview to remove code snippets and formatting
            clean_preview = clean_text_preview(text, max_length=150)

            source_display = {
                "chapter": chapter_title,
                "relevance_score": score,
                "text_preview": clean_preview  # Clean preview without code
            }

            # Add section title if available
            if section_title:
                source_display["section"] = section_title

            # Add URL if available (for clickable links)
            if source_url:
                source_display["url"] = source_url

            sources.append(source_display)

        top_chapter = sources[0]["chapter"] if sources else None

    except Exception as e:
        logger.error(f"Qdrant retrieval failed: {e}")
        # Fallback: answer without retrieval
        retrieved_content = "No relevant content retrieved from the textbook."
        sources = []
        retrieval_count = 0
        top_chapter = None

    # Step 3: Format prompt with retrieved content
    full_prompt = format_whole_book_prompt(retrieved_content, query)

    # Step 4: Call Gemini API
    logger.info("Calling Gemini API for answer generation...")

    try:
        response = gemini_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=full_prompt
        )

        response_text = response.text
        logger.info(f"Gemini response received ({len(response_text)} chars)")

    except Exception as e:
        logger.error(f"Gemini API failed: {e}")
        raise

    # Step 5: Calculate latency and build metadata
    latency_ms = int((time.time() - start_time) * 1000)

    metadata = {
        "latency_ms": latency_ms,
        "retrieval_count": retrieval_count,
        "top_chapter": top_chapter,
        "gemini_model": "gemini-2.5-flash",
        "query_type": "needs_rag",
        "query_intent": intent,
        "score_threshold": score_threshold,
        "rag_used": True
    }

    logger.info(f"Whole-book query completed in {latency_ms}ms")

    return response_text, sources, metadata


async def route_selected_text_query(
    query: str,
    selected_text: str,
    session_id: str,
    db_pool: asyncpg.Pool,
    gemini_client: genai.Client
) -> Tuple[str, Dict[str, Any]]:
    """Route selected-text query (zero external knowledge).

    Process:
    1. Retrieve chat history for session (for multi-turn context)
    2. Format prompt with ONLY selected text (strict system prompt)
    3. Call Gemini API to generate answer
    4. Return response (no sources, since no retrieval)

    Args:
        query: User's question
        selected_text: Text selected by user
        session_id: Session UUID for conversation history
        db_pool: Database connection pool
        gemini_client: Gemini API client

    Returns:
        Tuple of (response_text, metadata)
        - response_text: Gemini's answer
        - metadata: Dict with latency_ms, selected_text_length, model

    Raises:
        Exception: If Gemini API fails
    """
    logger.info("="*80)
    logger.info("🔍 SELECTED-TEXT MODE ROUTE")
    logger.info("="*80)
    logger.info(f"Query: {query}")
    logger.info(f"Selected text length: {len(selected_text)} chars")
    logger.info(f"Selected text (first 200 chars): {selected_text[:200]}")
    logger.info(f"Session ID: {session_id}")
    logger.info("="*80)

    start_time = time.time()

    # TODO: Load chat history for multi-turn conversations (Phase 5)
    # For now, treat each query independently

    # Step 1: Format prompt with selected text ONLY
    full_prompt = format_selected_text_prompt(selected_text, query)

    logger.info(f"📝 Formatted prompt (first 300 chars):")
    logger.info(full_prompt[:300] + "...")

    # Step 2: Call Gemini API
    logger.info(f"🤖 Calling Gemini API for selected-text query...")

    try:
        response = gemini_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=full_prompt
        )

        response_text = response.text
        logger.info(f"✅ Gemini response received ({len(response_text)} chars)")
        logger.info(f"Response (first 200 chars): {response_text[:200]}")

    except Exception as e:
        logger.error(f"❌ Gemini API failed: {e}")
        raise

    # Step 3: Calculate latency and build metadata
    latency_ms = int((time.time() - start_time) * 1000)

    metadata = {
        "latency_ms": latency_ms,
        "selected_text_length": len(selected_text),
        "gemini_model": "gemini-2.5-flash"
    }

    logger.info(f"✅ Selected-text query completed in {latency_ms}ms")
    logger.info("="*80)

    return response_text, metadata
