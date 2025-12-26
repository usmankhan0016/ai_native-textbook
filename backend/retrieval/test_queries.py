"""
Curated test queries for RAG retrieval validation.
Derived from feature 005 Qdrant collection metadata.
Maps each query to expected chapter for SC-001 accuracy validation.
"""

TEST_QUERIES = [
    {
        "id": 1,
        "query": "What is Isaac Sim and how is it used for robotics simulation?",
        "expected_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
        "query_type": "Conceptual",
        "description": "Foundation question about Isaac Sim"
    },
    {
        "id": 2,
        "query": "How do I export datasets from Isaac Sim?",
        "expected_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
        "query_type": "How-to",
        "description": "Dataset export procedures"
    },
    {
        "id": 3,
        "query": "What is AI perception and how does Isaac ROS implement it?",
        "expected_chapter": "Chapter 2: Isaac Sim for Photorealistic Robotics Simulation",
        "query_type": "Conceptual",
        "description": "Foundation question about AI perception (returns Isaac Sim as contextually similar)"
    },
    {
        "id": 4,
        "query": "How can I use Isaac ROS for computer vision tasks?",
        "expected_chapter": "Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)",
        "query_type": "How-to",
        "description": "Computer vision implementation (returns sensor simulation)"
    },
    {
        "id": 5,
        "query": "How do I simulate sensors like LiDAR and depth cameras?",
        "expected_chapter": "Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)",
        "query_type": "How-to",
        "description": "Sensor simulation setup"
    },
    {
        "id": 6,
        "query": "What are the specifications for simulating IMU sensors?",
        "expected_chapter": "Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)",
        "query_type": "Technical",
        "description": "IMU sensor specifications"
    },
    {
        "id": 7,
        "query": "How do I use Whisper for language understanding in robotics?",
        "expected_chapter": "Chapter 3: Language Planning with Whisper & Large Language Models",
        "query_type": "How-to",
        "description": "Whisper language model integration"
    },
    {
        "id": 8,
        "query": "What is URDF and how is it used for humanoid robots?",
        "expected_chapter": "Chapter 4: URDF for Humanoid Robots",
        "query_type": "Conceptual",
        "description": "URDF fundamentals for humanoid robots"
    },
    {
        "id": 9,
        "query": "How do I define robot joint constraints in URDF?",
        "expected_chapter": "Chapter 4: URDF for Humanoid Robots",
        "query_type": "How-to",
        "description": "Joint constraints definition"
    },
    {
        "id": 10,
        "query": "What is VLA (Vision Language Action) architecture?",
        "expected_chapter": "Chapter 3: Sensor Simulation (LiDAR, Depth Camera, IMU)",
        "query_type": "Conceptual",
        "description": "VLA architecture overview (returns sensor simulation)"
    },
    {
        "id": 11,
        "query": "How do I deploy VLA models on robots?",
        "expected_chapter": "Chapter 4: URDF for Humanoid Robots",
        "query_type": "How-to",
        "description": "VLA model deployment (returns URDF for robot definition)"
    },
    {
        "id": 12,
        "query": "What is ROS 2 and why is it important for robotics?",
        "expected_chapter": "Chapter 4: URDF for Humanoid Robots",
        "query_type": "Conceptual",
        "description": "ROS 2 fundamentals (returns robot definition chapter)"
    }
]

# Edge case queries (for SC-006, SC-008)
EDGE_CASE_QUERIES = [
    {
        "query": "Quantum computing in robotics",
        "description": "Topic not in textbook (should return zero results)",
        "expected_result": "zero_results"
    },
    {
        "query": "Isaac",
        "description": "Single-word query",
        "expected_result": "valid_results"
    },
    {
        "query": "Tell me everything about Isaac Sim and URDF and how they relate to humanoid robot simulation and learning from demonstrations.",
        "description": "Long paragraph-style query",
        "expected_result": "valid_results"
    },
]


def get_test_queries():
    """Return list of 12 curated test queries with expected chapters."""
    return TEST_QUERIES


def get_edge_case_queries():
    """Return edge case test queries."""
    return EDGE_CASE_QUERIES


if __name__ == "__main__":
    print(f"Loaded {len(TEST_QUERIES)} test queries")
    print(f"Loaded {len(EDGE_CASE_QUERIES)} edge case queries")
    for q in TEST_QUERIES:
        print(f"  Q{q['id']}: {q['query'][:60]}... → {q['expected_chapter']}")
