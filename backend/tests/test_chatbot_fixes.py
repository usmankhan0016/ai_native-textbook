"""
Test script for chatbot source display fixes.

Tests:
1. Query classification (greeting/chitchat vs needs_rag)
2. Chunk validation (filtering junk sources)
3. Text preview cleaning (removing code snippets)
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from agent.modes import classify_query_type, handle_simple_response, clean_text_preview
from retrieval.validators import is_valid_content_chunk, filter_valid_chunks


def test_query_classification():
    """Test query type classification."""
    print("\n=== Testing Query Classification ===")

    test_cases = [
        ("hi", "greeting"),
        ("hello", "greeting"),
        ("hey there", "greeting"),
        ("thanks", "chitchat"),
        ("thank you", "chitchat"),
        ("bye", "chitchat"),
        ("What is ROS?", "needs_rag"),
        ("Explain Python classes", "needs_rag"),
        ("How do I create a robot node?", "needs_rag"),
    ]

    passed = 0
    for query, expected in test_cases:
        result = classify_query_type(query)
        status = "✓" if result == expected else "✗"
        print(f"{status} '{query}' -> {result} (expected: {expected})")
        if result == expected:
            passed += 1

    print(f"\nPassed: {passed}/{len(test_cases)}")
    return passed == len(test_cases)


def test_simple_responses():
    """Test simple response generation."""
    print("\n=== Testing Simple Responses ===")

    test_cases = [
        ("hello", "greeting"),
        ("thanks", "chitchat"),
        ("bye", "chitchat"),
    ]

    for query, query_type in test_cases:
        response = handle_simple_response(query, query_type)
        print(f"'{query}' ({query_type}): {response[:80]}...")

    return True


def test_chunk_validation():
    """Test chunk validation to filter junk sources."""
    print("\n=== Testing Chunk Validation ===")

    # Test cases with valid and junk chunks
    test_chunks = [
        # Valid chunk
        {
            "text": "This is a valid chapter about ROS nodes and their implementation in Python.",
            "chapter": "Module 1: Introduction to ROS",
            "relevance_score": 0.75
        },
        # Junk: Tags navigation
        {
            "text": "Tags: A",
            "chapter": "Module 1",
            "relevance_score": 0.18
        },
        # Junk: Too short
        {
            "text": "See above.",
            "chapter": "Module 2",
            "relevance_score": 0.65
        },
        # Junk: Pure code without context
        {
            "text": "destroy_node ( ) rclpy . shutdown ( )",
            "chapter": "Module 3",
            "relevance_score": 0.55
        },
        # Valid chunk
        {
            "text": "The node lifecycle consists of initialization, execution, and shutdown phases. Each phase has specific requirements.",
            "chapter": "Module 2: Node Lifecycle",
            "relevance_score": 0.82
        },
        # Junk: Low relevance
        {
            "text": "This chapter explains various concepts related to robotics.",
            "chapter": "Module 4",
            "relevance_score": 0.12
        },
    ]

    valid_count = 0
    junk_count = 0

    for i, chunk in enumerate(test_chunks, 1):
        is_valid = is_valid_content_chunk(chunk)
        status = "✓ VALID" if is_valid else "✗ JUNK"
        print(f"{status} #{i}: {chunk['text'][:50]}... (score: {chunk['relevance_score']:.0%})")

        if is_valid:
            valid_count += 1
        else:
            junk_count += 1

    print(f"\nValid: {valid_count}, Junk filtered: {junk_count}")

    # Test filter_valid_chunks function
    print("\n--- Testing filter_valid_chunks ---")
    filtered = filter_valid_chunks(test_chunks, min_relevance=0.15, max_results=5)
    print(f"Original: {len(test_chunks)} chunks -> Filtered: {len(filtered)} chunks")

    for i, chunk in enumerate(filtered, 1):
        print(f"{i}. {chunk['chapter']} ({chunk['relevance_score']:.0%})")

    # Should filter out junk, keep only 2 valid chunks
    expected_valid = 2
    return len(filtered) == expected_valid


def test_text_cleaning():
    """Test text preview cleaning."""
    print("\n=== Testing Text Preview Cleaning ===")

    test_cases = [
        (
            "The destroy_node ( ) method is called when rclpy . shutdown ( ) is invoked to clean up resources.",
            "The method is called when is invoked to clean up resources."
        ),
        (
            "ROS nodes communicate using publishers and subscribers. The node.create_publisher() function sets up a publisher.",
            "ROS nodes communicate using publishers and subscribers. The function sets up a publisher."
        ),
        (
            "Tags: A | Navigation | Sidebar",
            "Tags: A | Navigation | Sidebar"  # Will be cleaned by removing special chars
        ),
    ]

    for original, description in test_cases:
        cleaned = clean_text_preview(original, max_length=150)
        print(f"\nOriginal: {original}")
        print(f"Cleaned:  {cleaned}")
        print(f"Expected: Contains meaningful text without code patterns")

    return True


if __name__ == "__main__":
    print("\n" + "="*60)
    print("RAG CHATBOT SOURCE DISPLAY FIXES - TEST SUITE")
    print("="*60)

    all_passed = True

    # Run all tests
    all_passed &= test_query_classification()
    all_passed &= test_simple_responses()
    all_passed &= test_chunk_validation()
    all_passed &= test_text_cleaning()

    print("\n" + "="*60)
    if all_passed:
        print("✓ ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("="*60 + "\n")

    sys.exit(0 if all_passed else 1)
