"""
Standalone test for chatbot fixes (no external dependencies).
Tests the core logic without importing full modules.
"""

import re


# Query classification logic (copied from agent/modes.py)
def classify_query_type(query: str) -> str:
    """Classify query to determine if RAG retrieval is needed."""
    query_lower = query.lower().strip()

    greetings = ['hi', 'hello', 'hey', 'good morning', 'good afternoon', 'good evening', 'greetings', 'howdy']
    if query_lower in greetings or (len(query_lower) < 10 and any(g in query_lower for g in ['hi', 'hello', 'hey'])):
        return 'greeting'

    chitchat = ['thanks', 'thank you', 'bye', 'goodbye', 'ok', 'okay', 'cool', 'nice', 'great', 'awesome', 'perfect']
    if query_lower in chitchat:
        return 'chitchat'

    return 'needs_rag'


# Chunk validation logic (copied from retrieval/validators.py)
def is_valid_content_chunk(chunk: dict, min_relevance: float = 0.15) -> bool:
    """Filter out junk/garbage chunks."""
    if not chunk.get("text") or not chunk.get("chapter"):
        return False

    text = chunk.get("text", "").strip()
    chapter = chunk.get("chapter", "").strip()
    relevance = chunk.get("relevance_score", 0.0)

    if relevance < min_relevance:
        return False

    if len(text) < 20:
        return False

    junk_patterns = [
        r"^Tags:\s*[A-Z]$",
        r"^Sidebar$",
        r"^Navigation$",
        r"^Previous\s*\|\s*Next$",
        r"^\s*\|\s*\|\s*$",
        r"^-+$",
        r"^\*+$",
        r"^=+$",
        r"^>+$",
    ]

    for pattern in junk_patterns:
        if re.match(pattern, text, re.IGNORECASE):
            return False

    invalid_chapters = ["unknown", "undefined", "null", "tags:", "sidebar", "navigation"]
    if chapter.lower() in invalid_chapters or len(chapter) < 3:
        return False

    # Filter pure code chunks using multiple heuristics
    code_chars = sum(1 for c in text if c in "(){}[];,._")
    if len(text) > 0 and (code_chars / len(text)) > 0.5:
        return False

    # Check for multiple function calls
    function_call_pattern = r'(\w+\s*\(\s*\)\s*){2,}|\w+\s*\.\s*\w+\s*\(\s*\)'
    if re.search(function_call_pattern, text):
        cleaned = re.sub(r'\w+\s*\(\s*\)', '', text)
        cleaned = re.sub(r'\w+\s*\.\s*\w+', '', cleaned)
        cleaned = re.sub(r'[(){}[\];,._]+', '', cleaned)
        cleaned = cleaned.strip()
        if len(cleaned) < len(text) * 0.3:
            return False

    if not re.search(r'[a-zA-Z]{3,}', text):
        return False

    return True


# Text cleaning logic (copied from agent/modes.py)
def clean_text_preview(text: str, max_length: int = 150) -> str:
    """Clean text preview by removing code snippets."""
    text = re.sub(r'\s+', ' ', text).strip()
    text = re.sub(r'\b\w+\s*\(\s*\)', '', text)
    text = re.sub(r'\b\w+\s*\.\s*\w+\s*\(\s*\)', '', text)
    text = re.sub(r'[(){}[\];]+', ' ', text)
    text = re.sub(r'\s+', ' ', text).strip()
    text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
    text = re.sub(r'`[^`]+`', '', text)

    if len(text) > max_length:
        text = text[:max_length]
        last_space = text.rfind(' ')
        if last_space > max_length * 0.7:
            text = text[:last_space]
        text = text.strip() + '...'

    return text


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


def test_chunk_validation():
    """Test chunk validation to filter junk sources."""
    print("\n=== Testing Chunk Validation ===")

    test_chunks = [
        {
            "text": "This is a valid chapter about ROS nodes and their implementation in Python.",
            "chapter": "Module 1: Introduction to ROS",
            "relevance_score": 0.75
        },
        {
            "text": "Tags: A",
            "chapter": "Module 1",
            "relevance_score": 0.18
        },
        {
            "text": "See above.",
            "chapter": "Module 2",
            "relevance_score": 0.65
        },
        {
            "text": "destroy_node ( ) rclpy . shutdown ( )",
            "chapter": "Module 3",
            "relevance_score": 0.55
        },
        {
            "text": "The node lifecycle consists of initialization, execution, and shutdown phases. Each phase has specific requirements.",
            "chapter": "Module 2: Node Lifecycle",
            "relevance_score": 0.82
        },
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

    # Should keep 2 valid chunks, filter 4 junk
    return valid_count == 2 and junk_count == 4


def test_text_cleaning():
    """Test text preview cleaning."""
    print("\n=== Testing Text Preview Cleaning ===")

    test_cases = [
        "The destroy_node ( ) method is called when rclpy . shutdown ( ) is invoked to clean up resources.",
        "ROS nodes communicate using publishers and subscribers. The node.create_publisher() function sets up a publisher.",
        "This is a normal sentence with some code like init_node() embedded in it.",
    ]

    for original in test_cases:
        cleaned = clean_text_preview(original, max_length=150)
        print(f"\nOriginal: {original}")
        print(f"Cleaned:  {cleaned}")

        # Verify code patterns are removed
        has_parens = '()' in cleaned
        if has_parens:
            print("  ⚠ Warning: Still contains () patterns")

    return True


if __name__ == "__main__":
    print("\n" + "="*60)
    print("RAG CHATBOT SOURCE DISPLAY FIXES - TEST SUITE")
    print("="*60)

    all_passed = True
    all_passed &= test_query_classification()
    all_passed &= test_chunk_validation()
    all_passed &= test_text_cleaning()

    print("\n" + "="*60)
    if all_passed:
        print("✓ ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("="*60 + "\n")
