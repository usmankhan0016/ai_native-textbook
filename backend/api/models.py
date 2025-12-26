"""Pydantic models for API request/response validation (Feature 007).

This module defines:
- Request models: ChatRequest, SelectedTextChatRequest, CreateSessionRequest
- Response models: SessionResponse, MessageResponse, ChatResponse, ChatHistoryResponse
- Supporting models: RetrievalSource
"""

from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any, Literal
from datetime import datetime


# ============================================================================
# Request Models
# ============================================================================


class CreateSessionRequest(BaseModel):
    """Request to create a new chat session."""

    metadata: Optional[Dict[str, str]] = Field(
        default={},
        description="Optional extensible metadata for the session"
    )


class ChatRequest(BaseModel):
    """Request to send a whole-book query."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=50000,
        description="User's question (1-50,000 characters)"
    )
    mode: Literal["whole_book"] = Field(
        default="whole_book",
        description="Query mode (always 'whole_book' for this endpoint)"
    )


class SelectedTextChatRequest(BaseModel):
    """Request to send a selected-text-only query."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=50000,
        description="User's question (1-50,000 characters)"
    )
    selected_text: str = Field(
        ...,
        min_length=1,
        max_length=100000,
        description="Text selected by user (1-100,000 characters)"
    )
    chapter_origin: Optional[str] = Field(
        default=None,
        description="Optional chapter name if known"
    )
    mode: Literal["selected_text"] = Field(
        default="selected_text",
        description="Query mode (always 'selected_text' for this endpoint)"
    )


# ============================================================================
# Response Models
# ============================================================================


class SessionResponse(BaseModel):
    """Response for session creation or retrieval."""

    id: str = Field(..., description="Session UUID as string")
    created_at: datetime = Field(..., description="Session creation timestamp")
    updated_at: datetime = Field(..., description="Last activity timestamp")
    metadata: Dict[str, str] = Field(..., description="Session metadata")


class MessageResponse(BaseModel):
    """Response for a single message."""

    id: str = Field(..., description="Message UUID as string")
    session_id: str = Field(..., description="Session UUID")
    role: Literal["user", "assistant"] = Field(..., description="Message author")
    content: str = Field(..., description="Message text")
    created_at: datetime = Field(..., description="Message timestamp")
    mode: Literal["whole_book", "selected_text"] = Field(..., description="Query mode")
    metadata: Dict[str, Any] = Field(..., description="Message metadata (latency, model, etc.)")


class RetrievalSource(BaseModel):
    """Source attribution for whole-book queries."""

    chapter: str = Field(..., description="Chapter name or title")
    relevance_score: float = Field(..., description="Cosine similarity score (0-1)")
    text_preview: str = Field(..., description="Short preview of content (150 chars)")
    section: Optional[str] = Field(
        default=None,
        description="Section title within chapter (if available)"
    )
    url: Optional[str] = Field(
        default=None,
        description="URL to source page in published book (if available)"
    )


class ChatResponse(BaseModel):
    """Response for chat queries (both whole-book and selected-text)."""

    message: MessageResponse = Field(..., description="The assistant's answer")
    sources: Optional[List[RetrievalSource]] = Field(
        default=None,
        description="Source attribution (only for whole_book mode, null for selected_text)"
    )


class ChatHistoryResponse(BaseModel):
    """Response for retrieving chat history."""

    session_id: str = Field(..., description="Session UUID")
    messages: List[MessageResponse] = Field(..., description="All messages in chronological order")


# ============================================================================
# Helper Functions
# ============================================================================


def session_to_response(session) -> SessionResponse:
    """Convert database Session model to SessionResponse.

    Args:
        session: Session instance from db.models

    Returns:
        SessionResponse for API
    """
    return SessionResponse(
        id=str(session.id),
        created_at=session.created_at,
        updated_at=session.updated_at,
        metadata=session.metadata
    )


def message_to_response(message) -> MessageResponse:
    """Convert database Message model to MessageResponse.

    Args:
        message: Message instance from db.models

    Returns:
        MessageResponse for API
    """
    return MessageResponse(
        id=str(message.id),
        session_id=str(message.session_id),
        role=message.role,
        content=message.content,
        created_at=message.created_at,
        mode=message.mode,
        metadata=message.metadata
    )


def messages_to_history_response(session_id: str, messages: List) -> ChatHistoryResponse:
    """Convert list of Message models to ChatHistoryResponse.

    Args:
        session_id: Session UUID
        messages: List of Message instances from db.models

    Returns:
        ChatHistoryResponse for API
    """
    return ChatHistoryResponse(
        session_id=session_id,
        messages=[message_to_response(msg) for msg in messages]
    )
