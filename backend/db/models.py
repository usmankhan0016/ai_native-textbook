"""Database models for Feature 007 RAG Agent Backend.

This module defines data structures for:
- Session: User chat sessions
- Message: Chat messages (user questions and agent responses)
- SelectedTextMetadata: Selected text used in queries

These are lightweight data classes (not ORM models) for use with asyncpg raw SQL queries.
"""

from datetime import datetime
from typing import Dict, Any, Optional
from uuid import UUID


class Session:
    """Represents a user chat session.

    Attributes:
        id: Unique session identifier (UUID)
        created_at: Session creation timestamp
        updated_at: Last activity timestamp
        metadata: Extensible JSON metadata
    """

    def __init__(
        self,
        id: UUID,
        created_at: datetime,
        updated_at: datetime,
        metadata: Dict[str, Any],
    ):
        self.id = id
        self.created_at = created_at
        self.updated_at = updated_at
        self.metadata = metadata

    @classmethod
    def from_record(cls, record: Any) -> "Session":
        """Create Session from asyncpg record.

        Args:
            record: asyncpg Record object from database query

        Returns:
            Session instance
        """
        return cls(
            id=record["id"],
            created_at=record["created_at"],
            updated_at=record["updated_at"],
            metadata=record["metadata"] or {},
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for API responses.

        Returns:
            Dictionary representation
        """
        return {
            "id": str(self.id),
            "created_at": self.created_at.isoformat(),
            "updated_at": self.updated_at.isoformat(),
            "metadata": self.metadata,
        }


class Message:
    """Represents a chat message (question or agent response).

    Attributes:
        id: Unique message identifier (UUID)
        session_id: Foreign key to session
        role: Message author ('user' or 'assistant')
        content: Message text
        created_at: Message timestamp
        mode: Query mode ('whole_book' or 'selected_text')
        metadata: Extensible JSON metadata (latency, model version, etc.)
    """

    def __init__(
        self,
        id: UUID,
        session_id: UUID,
        role: str,
        content: str,
        created_at: datetime,
        mode: str,
        metadata: Dict[str, Any],
    ):
        self.id = id
        self.session_id = session_id
        self.role = role
        self.content = content
        self.created_at = created_at
        self.mode = mode
        self.metadata = metadata

    @classmethod
    def from_record(cls, record: Any) -> "Message":
        """Create Message from asyncpg record.

        Args:
            record: asyncpg Record object from database query

        Returns:
            Message instance
        """
        return cls(
            id=record["id"],
            session_id=record["session_id"],
            role=record["role"],
            content=record["content"],
            created_at=record["created_at"],
            mode=record["mode"],
            metadata=record["metadata"] or {},
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for API responses.

        Returns:
            Dictionary representation
        """
        return {
            "id": str(self.id),
            "session_id": str(self.session_id),
            "role": self.role,
            "content": self.content,
            "created_at": self.created_at.isoformat(),
            "mode": self.mode,
            "metadata": self.metadata,
        }


class SelectedTextMetadata:
    """Represents selected text metadata for audit trail.

    Attributes:
        id: Unique record identifier (UUID)
        message_id: Foreign key to message
        selected_text: Full text selected by user
        chapter_origin: Optional chapter name
        created_at: Timestamp
    """

    def __init__(
        self,
        id: UUID,
        message_id: UUID,
        selected_text: str,
        chapter_origin: Optional[str],
        created_at: datetime,
    ):
        self.id = id
        self.message_id = message_id
        self.selected_text = selected_text
        self.chapter_origin = chapter_origin
        self.created_at = created_at

    @classmethod
    def from_record(cls, record: Any) -> "SelectedTextMetadata":
        """Create SelectedTextMetadata from asyncpg record.

        Args:
            record: asyncpg Record object from database query

        Returns:
            SelectedTextMetadata instance
        """
        return cls(
            id=record["id"],
            message_id=record["message_id"],
            selected_text=record["selected_text"],
            chapter_origin=record.get("chapter_origin"),
            created_at=record["created_at"],
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for API responses.

        Returns:
            Dictionary representation
        """
        return {
            "id": str(self.id),
            "message_id": str(self.message_id),
            "selected_text": self.selected_text,
            "chapter_origin": self.chapter_origin,
            "created_at": self.created_at.isoformat(),
        }
