"""Test Gemini API connectivity for Feature 007."""

import os
import sys
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    print("❌ ERROR: GEMINI_API_KEY not found in .env file")
    sys.exit(1)

print("🔧 Testing Gemini API connection...")
print(f"🔑 API Key: {GEMINI_API_KEY[:20]}...")

try:
    from google import genai
    from google.genai import types

    # Configure Gemini API
    client = genai.Client(api_key=GEMINI_API_KEY)

    print("✅ Gemini API configured successfully!")
    print("🧪 Testing simple query...")

    # Test query with gemini-2.5-flash (latest stable model)
    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents="Say 'Hello from Gemini!' in exactly 3 words."
    )

    print(f"✅ Gemini response: {response.text}")
    print("🎉 Gemini API connection verified!")

except ImportError:
    print("❌ google-generativeai not installed")
    print("   Install with: uv pip install google-generativeai")
    sys.exit(1)

except Exception as e:
    print(f"❌ ERROR: {type(e).__name__}: {e}")
    sys.exit(1)
