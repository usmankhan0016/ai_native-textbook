"""Session management routes for Feature 007.

Endpoints:
- POST /api/sessions: Create new chat session
- GET /api/sessions/{session_id}/history: Retrieve chat history
"""

import logging
from fastapi import APIRouter, Depends, HTTPException
from uuid import UUID
import asyncpg

from ..models import (
    CreateSessionRequest,
    SessionResponse,
    ChatHistoryResponse,
    session_to_response,
    messages_to_history_response
)
from db.connection import get_db_pool
from db.repositories import SessionRepository

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/api/sessions", response_model=SessionResponse, status_code=201, tags=["Sessions"])
async def create_session(
    request: CreateSessionRequest,
    pool: asyncpg.Pool = Depends(get_db_pool)
):
    """Create a new chat session.

    Args:
        request: CreateSessionRequest with optional metadata
        pool: Database connection pool (injected)

    Returns:
        SessionResponse with new session ID

    Raises:
        HTTPException 500: If database operation fails
    """
    try:
        session = await SessionRepository.create(pool, metadata=request.metadata)
        logger.info(f"Created session {session.id}")
        return session_to_response(session)

    except Exception as e:
        logger.error(f"Failed to create session: {e}")
        raise HTTPException(status_code=500, detail="Failed to create session")


@router.get("/api/sessions/{session_id}/history", response_model=ChatHistoryResponse, tags=["Sessions"])
async def get_session_history(
    session_id: str,
    pool: asyncpg.Pool = Depends(get_db_pool)
):
    """Retrieve chat history for a session.

    Args:
        session_id: Session UUID
        pool: Database connection pool (injected)

    Returns:
        ChatHistoryResponse with all messages in chronological order

    Raises:
        HTTPException 404: If session not found
        HTTPException 500: If database operation fails
    """
    try:
        # Validate session exists
        session_uuid = UUID(session_id)
        session = await SessionRepository.get_by_id(pool, session_uuid)

        if session is None:
            raise HTTPException(status_code=404, detail="Session not found")

        # Retrieve all messages for session
        messages = await SessionRepository.list_messages(pool, session_uuid)

        logger.info(f"Retrieved {len(messages)} messages for session {session_id}")

        return messages_to_history_response(session_id, messages)

    except ValueError:
        # Invalid UUID format
        raise HTTPException(status_code=400, detail="Invalid session ID format")

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        logger.error(f"Failed to retrieve session history: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve chat history")
