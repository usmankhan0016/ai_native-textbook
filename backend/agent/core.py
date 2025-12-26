"""Gemini API agent core for Feature 007 RAG Agent Backend.

This module handles:
- Gemini API client initialization
- Agent lifecycle management (startup/shutdown)
- Global client instance for FastAPI dependency injection
"""

import os
import logging
from typing import Optional
from google import genai

logger = logging.getLogger(__name__)


# Global Gemini client instance
_gemini_client: Optional[genai.Client] = None


def create_gemini_client() -> genai.Client:
    """Create and configure Gemini API client.

    Configuration:
    - Uses GEMINI_API_KEY from environment
    - Model: gemini-2.5-flash (latest stable)

    Returns:
        genai.Client: Configured Gemini API client

    Raises:
        ValueError: If GEMINI_API_KEY not found in environment
    """
    api_key = os.getenv("GEMINI_API_KEY")

    if not api_key:
        raise ValueError("GEMINI_API_KEY not found in environment variables")

    logger.info("Initializing Gemini API client...")
    logger.info(f"API Key: {api_key[:20]}...")

    client = genai.Client(api_key=api_key)

    logger.info("✅ Gemini API client initialized successfully")
    logger.info("Model: gemini-2.5-flash")

    return client


async def init_agent() -> None:
    """Initialize the global Gemini client.

    Should be called during FastAPI app startup (lifespan context).
    """
    global _gemini_client

    if _gemini_client is not None:
        logger.warning("Gemini client already initialized. Skipping re-initialization.")
        return

    _gemini_client = create_gemini_client()


async def shutdown_agent() -> None:
    """Shutdown the global Gemini client.

    Should be called during FastAPI app shutdown (lifespan context).
    """
    global _gemini_client

    if _gemini_client is None:
        logger.warning("Gemini client not initialized. Nothing to shutdown.")
        return

    logger.info("Shutting down Gemini API client...")
    # Note: google-genai client doesn't require explicit cleanup
    _gemini_client = None
    logger.info("✅ Gemini API client shutdown complete")


def get_gemini_client() -> genai.Client:
    """Get the global Gemini client instance.

    Returns:
        genai.Client: The active Gemini client

    Raises:
        RuntimeError: If client not initialized (call init_agent first)
    """
    if _gemini_client is None:
        raise RuntimeError("Gemini client not initialized. Call init_agent() during app startup.")

    return _gemini_client


def get_agent() -> genai.Client:
    """FastAPI dependency to inject Gemini client into route handlers.

    Usage:
        @app.post("/api/sessions/{session_id}/chat")
        async def chat(
            session_id: str,
            body: ChatRequest,
            agent: genai.Client = Depends(get_agent)
        ):
            response = agent.models.generate_content(
                model="gemini-2.5-flash",
                contents=body.query
            )
            return {"answer": response.text}

    Returns:
        genai.Client: The active Gemini client
    """
    return get_gemini_client()
