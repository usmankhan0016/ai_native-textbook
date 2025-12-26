#!/usr/bin/env python3
"""
Quick verification script to test RAG chatbot functionality
Tests backend connectivity and basic query flow
"""

import os
import sys
import requests
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

BACKEND_URL = "http://localhost:8000"

def check_env_vars():
    """Verify all required environment variables are set"""
    required_vars = [
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "COHERE_API_KEY",
        "GEMINI_API_KEY",
        "DATABASE_URL"  # Changed from NEON_DATABASE_URL
    ]

    missing = []
    for var in required_vars:
        if not os.getenv(var):
            missing.append(var)

    if missing:
        print(f"❌ Missing environment variables: {', '.join(missing)}")
        return False

    print("✅ All required environment variables are set")
    return True


def check_backend_health():
    """Check if backend is running"""
    try:
        response = requests.get(f"{BACKEND_URL}/health", timeout=5)
        if response.status_code == 200:
            print("✅ Backend is running and healthy")
            return True
        else:
            print(f"❌ Backend returned status {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("❌ Backend is not running! Start it with: uvicorn api.main:app --reload")
        return False
    except Exception as e:
        print(f"❌ Error connecting to backend: {e}")
        return False


def test_session_creation():
    """Test session creation"""
    try:
        response = requests.post(
            f"{BACKEND_URL}/api/sessions",
            json={},
            timeout=10
        )

        if response.status_code in [200, 201]:  # Accept both 200 and 201
            data = response.json()
            session_id = data.get("id")
            print(f"✅ Session created: {session_id}")
            return session_id
        else:
            print(f"❌ Session creation failed: {response.status_code}")
            print(f"   Response: {response.text}")
            return None

    except Exception as e:
        print(f"❌ Session creation error: {e}")
        return None


def test_chat_query(session_id):
    """Test a simple chat query"""
    if not session_id:
        return False

    try:
        print("\n🤖 Testing query: 'What is ROS 2?'")

        response = requests.post(
            f"{BACKEND_URL}/api/sessions/{session_id}/chat",
            json={"query": "What is ROS 2?"},
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()

            # Check response structure
            if "message" not in data:
                print("❌ Response missing 'message' field")
                print(f"   Response: {data}")
                return False

            if "sources" not in data:
                print("❌ Response missing 'sources' field")
                return False

            message_content = data["message"].get("content", "")
            sources_count = len(data.get("sources", []))

            print(f"✅ Query successful!")
            print(f"   Answer length: {len(message_content)} chars")
            print(f"   Sources: {sources_count} chunks")
            print(f"\n   Answer preview: {message_content[:200]}...")

            if sources_count > 0:
                print(f"\n   First source:")
                first_source = data["sources"][0]
                print(f"     - Chapter: {first_source.get('chapter', 'N/A')}")
                print(f"     - Relevance: {first_source.get('relevance_score', 0):.2f}")

            return True
        else:
            print(f"❌ Query failed: {response.status_code}")
            print(f"   Response: {response.text}")
            return False

    except Exception as e:
        print(f"❌ Query error: {e}")
        return False


def main():
    """Run all verification checks"""
    print("=" * 60)
    print("RAG Chatbot Verification Script")
    print("=" * 60)
    print()

    # Step 1: Check environment variables
    print("Step 1: Checking environment variables...")
    if not check_env_vars():
        sys.exit(1)
    print()

    # Step 2: Check backend health
    print("Step 2: Checking backend health...")
    if not check_backend_health():
        sys.exit(1)
    print()

    # Step 3: Test session creation
    print("Step 3: Testing session creation...")
    session_id = test_session_creation()
    if not session_id:
        sys.exit(1)
    print()

    # Step 4: Test chat query
    print("Step 4: Testing chat query...")
    if not test_chat_query(session_id):
        sys.exit(1)
    print()

    print("=" * 60)
    print("✅ All checks passed! Chatbot is working correctly.")
    print("=" * 60)
    print()
    print("Next steps:")
    print("1. Start frontend: cd docusaurus_textbook && npm start")
    print("2. Open http://localhost:3000")
    print("3. Click chatbot button (💬) in bottom-right")
    print("4. Try asking: 'What is this book about?'")
    print()


if __name__ == "__main__":
    main()
