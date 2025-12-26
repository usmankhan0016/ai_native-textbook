"""List available Gemini models."""

import os
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

try:
    from google import genai

    client = genai.Client(api_key=GEMINI_API_KEY)

    print("📋 Available Gemini models:\n")

    models = client.models.list()

    for model in models:
        print(f"  - {model.name}")
        if hasattr(model, 'supported_generation_methods'):
            print(f"    Methods: {', '.join(model.supported_generation_methods)}")
        print()

except Exception as e:
    print(f"❌ ERROR: {e}")
