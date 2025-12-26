"""Chat endpoints for Feature 007.

Endpoints:
- POST /api/sessions/{session_id}/chat: Whole-book query (Qdrant retrieval)
- POST /api/sessions/{session_id}/selected-text-chat: Selected-text query (zero external knowledge)
"""

import logging
from fastapi import APIRouter, Depends, HTTPException
from uuid import UUID
import asyncpg
from google import genai

from ..models import (
    ChatRequest,
    SelectedTextChatRequest,
    ChatResponse,
    MessageResponse,
    RetrievalSource,
    message_to_response
)
from db.connection import get_db_pool
from db.repositories import SessionRepository, MessageRepository, SelectedTextMetadataRepository
from agent.core import get_agent
from agent.modes import route_whole_book_query, route_selected_text_query

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/api/sessions/{session_id}/chat", response_model=ChatResponse, tags=["Chat"])
async def whole_book_chat(
    session_id: str,
    request: ChatRequest,
    pool: asyncpg.Pool = Depends(get_db_pool),
    agent: genai.Client = Depends(get_agent)
):
    """Send a whole-book query (retrieves from Qdrant, generates grounded answer).

    Process:
    1. Validate session exists
    2. Store user question in database
    3. Retrieve relevant chapters from Qdrant
    4. Generate answer using Gemini API
    5. Store agent response in database
    6. Update session timestamp
    7. Return response with sources

    Args:
        session_id: Session UUID
        request: ChatRequest with query
        pool: Database connection pool (injected)
        agent: Gemini API client (injected)

    Returns:
        ChatResponse with message and sources

    Raises:
        HTTPException 400: If query validation fails
        HTTPException 404: If session not found
        HTTPException 429: If Gemini API rate limit exceeded
        HTTPException 500: If internal error occurs
        HTTPException 503: If Qdrant or Gemini API unavailable
    """
    try:
        # Validate session exists
        session_uuid = UUID(session_id)
        session = await SessionRepository.get_by_id(pool, session_uuid)

        if session is None:
            raise HTTPException(status_code=404, detail="Session not found")

        # Store user question
        user_message = await MessageRepository.create(
            pool=pool,
            session_id=session_uuid,
            role="user",
            content=request.query,
            mode="whole_book",
            metadata={}
        )

        logger.info(f"User question stored: {user_message.id}")

        # Route query through Qdrant + Gemini
        try:
            response_text, sources, metadata = await route_whole_book_query(
                query=request.query,
                session_id=session_id,
                db_pool=pool,
                gemini_client=agent
            )

        except Exception as e:
            logger.error(f"Whole-book routing failed: {e}")

            # Check for specific error types
            if "429" in str(e) or "RESOURCE_EXHAUSTED" in str(e):
                raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")
            elif "503" in str(e) or "unavailable" in str(e).lower():
                raise HTTPException(status_code=503, detail="Retrieval service temporarily unavailable")
            else:
                raise HTTPException(status_code=500, detail="Failed to process query")

        # Store assistant response
        assistant_message = await MessageRepository.create(
            pool=pool,
            session_id=session_uuid,
            role="assistant",
            content=response_text,
            mode="whole_book",
            metadata=metadata
        )

        logger.info(f"Assistant response stored: {assistant_message.id}")

        # Update session timestamp
        await SessionRepository.update_timestamp(pool, session_uuid)

        # Build response
        message_response = message_to_response(assistant_message)
        source_responses = [RetrievalSource(**source) for source in sources]

        return ChatResponse(
            message=message_response,
            sources=source_responses
        )

    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid session ID format")

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        logger.error(f"Whole-book chat failed: {e}")
        raise HTTPException(status_code=500, detail="Failed to process query")


@router.post("/api/sessions/{session_id}/selected-text-chat", response_model=ChatResponse, tags=["Chat"])
async def selected_text_chat(
    session_id: str,
    request: SelectedTextChatRequest,
    pool: asyncpg.Pool = Depends(get_db_pool),
    agent: genai.Client = Depends(get_agent)
):
    """Send a selected-text-only query (zero external knowledge).

    Process:
    1. Validate session exists
    2. Validate selected text is non-empty
    3. Store user question in database
    4. Generate answer using ONLY selected text (strict prompt)
    5. Store selected text metadata
    6. Store agent response in database
    7. Update session timestamp
    8. Return response (no sources, since no retrieval)

    Args:
        session_id: Session UUID
        request: SelectedTextChatRequest with query and selected_text
        pool: Database connection pool (injected)
        agent: Gemini API client (injected)

    Returns:
        ChatResponse with message (sources=null)

    Raises:
        HTTPException 400: If selected text empty or validation fails
        HTTPException 404: If session not found
        HTTPException 429: If Gemini API rate limit exceeded
        HTTPException 500: If internal error occurs
    """
    logger.info("="*80)
    logger.info("📝 SELECTED-TEXT-CHAT ENDPOINT CALLED")
    logger.info("="*80)
    logger.info(f"Session ID: {session_id}")
    logger.info(f"Query: {request.query}")
    logger.info(f"Selected text length: {len(request.selected_text)} chars")
    logger.info(f"Selected text (first 200 chars): {request.selected_text[:200]}")
    logger.info(f"Chapter origin: {request.chapter_origin}")
    logger.info("="*80)

    try:
        # Validate selected text
        if not request.selected_text or not request.selected_text.strip():
            logger.error("❌ Selected text is empty or None!")
            raise HTTPException(status_code=400, detail="Selected text cannot be empty")

        # Warn if selected text is very short (but still allow it)
        if len(request.selected_text.strip()) < 10:
            logger.warning(f"⚠️ Selected text is quite short ({len(request.selected_text)} chars)")

        logger.info(f"✅ Selected text validation passed ({len(request.selected_text)} chars)")

        # Validate session exists
        session_uuid = UUID(session_id)
        session = await SessionRepository.get_by_id(pool, session_uuid)

        if session is None:
            logger.error(f"❌ Session not found: {session_id}")
            raise HTTPException(status_code=404, detail="Session not found")

        logger.info(f"✅ Session found: {session_uuid}")

        # Store user question
        user_message = await MessageRepository.create(
            pool=pool,
            session_id=session_uuid,
            role="user",
            content=request.query,
            mode="selected_text",
            metadata={}
        )

        logger.info(f"✅ User question stored: {user_message.id}")

        # Route query through Gemini (selected text only)
        logger.info("🚀 Routing to selected-text mode...")
        try:
            response_text, metadata = await route_selected_text_query(
                query=request.query,
                selected_text=request.selected_text,
                session_id=session_id,
                db_pool=pool,
                gemini_client=agent
            )

        except Exception as e:
            logger.error(f"Selected-text routing failed: {e}")

            # Check for specific error types
            if "429" in str(e) or "RESOURCE_EXHAUSTED" in str(e):
                raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")
            else:
                raise HTTPException(status_code=500, detail="Failed to process query")

        # Store assistant response
        assistant_message = await MessageRepository.create(
            pool=pool,
            session_id=session_uuid,
            role="assistant",
            content=response_text,
            mode="selected_text",
            metadata=metadata
        )

        logger.info(f"Assistant response stored: {assistant_message.id}")

        # Store selected text metadata
        await SelectedTextMetadataRepository.create(
            pool=pool,
            message_id=user_message.id,
            selected_text=request.selected_text,
            chapter_origin=request.chapter_origin
        )

        # Update session timestamp
        await SessionRepository.update_timestamp(pool, session_uuid)

        # Build response (no sources for selected-text mode)
        message_response = message_to_response(assistant_message)

        return ChatResponse(
            message=message_response,
            sources=None
        )

    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid session ID format")

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        logger.error(f"Selected-text chat failed: {e}")
        raise HTTPException(status_code=500, detail="Failed to process query")
