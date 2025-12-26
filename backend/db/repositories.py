"""CRUD repositories for Feature 007 RAG Agent Backend.

This module provides database operations for:
- SessionRepository: Create, retrieve, delete sessions
- MessageRepository: Create, list messages by session
- SelectedTextMetadataRepository: Create, retrieve selected text metadata
"""

import logging
import json
from typing import List, Optional, Dict, Any
from uuid import UUID
import asyncpg

from .models import Session, Message, SelectedTextMetadata

logger = logging.getLogger(__name__)


class SessionRepository:
    """Repository for session CRUD operations."""

    @staticmethod
    async def create(
        pool: asyncpg.Pool,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Session:
        """Create a new session.

        Args:
            pool: Database connection pool
            metadata: Optional metadata dictionary

        Returns:
            Created Session instance

        Raises:
            asyncpg.exceptions.PostgresError: If database operation fails
        """
        metadata = metadata or {}

        async with pool.acquire() as conn:
            # Set codec for JSONB handling
            await conn.set_type_codec(
                'jsonb',
                encoder=json.dumps,
                decoder=json.loads,
                schema='pg_catalog'
            )

            record = await conn.fetchrow(
                """
                INSERT INTO sessions (metadata)
                VALUES ($1)
                RETURNING id, created_at, updated_at, metadata
                """,
                metadata
            )

        session = Session.from_record(record)
        logger.info(f"Created session: {session.id}")
        return session

    @staticmethod
    async def get_by_id(pool: asyncpg.Pool, session_id: UUID) -> Optional[Session]:
        """Retrieve session by ID.

        Args:
            pool: Database connection pool
            session_id: Session UUID

        Returns:
            Session instance if found, None otherwise
        """
        async with pool.acquire() as conn:
            record = await conn.fetchrow(
                """
                SELECT id, created_at, updated_at, metadata
                FROM sessions
                WHERE id = $1
                """,
                session_id
            )

        if record is None:
            logger.warning(f"Session not found: {session_id}")
            return None

        return Session.from_record(record)

    @staticmethod
    async def update_timestamp(pool: asyncpg.Pool, session_id: UUID) -> None:
        """Update session updated_at timestamp.

        Called when new messages are added to track session activity.

        Args:
            pool: Database connection pool
            session_id: Session UUID
        """
        async with pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE sessions
                SET updated_at = NOW()
                WHERE id = $1
                """,
                session_id
            )

    @staticmethod
    async def delete(pool: asyncpg.Pool, session_id: UUID) -> bool:
        """Delete session and all associated messages (CASCADE).

        Args:
            pool: Database connection pool
            session_id: Session UUID

        Returns:
            True if session was deleted, False if not found
        """
        async with pool.acquire() as conn:
            result = await conn.execute(
                """
                DELETE FROM sessions
                WHERE id = $1
                """,
                session_id
            )

        # result is like "DELETE 1" or "DELETE 0"
        deleted = int(result.split()[-1]) > 0

        if deleted:
            logger.info(f"Deleted session: {session_id}")
        else:
            logger.warning(f"Session not found for deletion: {session_id}")

        return deleted

    @staticmethod
    async def list_messages(pool: asyncpg.Pool, session_id: UUID) -> List[Message]:
        """List all messages for a session (ordered by created_at ASC).

        Args:
            pool: Database connection pool
            session_id: Session UUID

        Returns:
            List of Message instances (may be empty)
        """
        async with pool.acquire() as conn:
            records = await conn.fetch(
                """
                SELECT id, session_id, role, content, created_at, mode, metadata
                FROM messages
                WHERE session_id = $1
                ORDER BY created_at ASC
                """,
                session_id
            )

        messages = [Message.from_record(record) for record in records]
        logger.debug(f"Retrieved {len(messages)} messages for session {session_id}")
        return messages


class MessageRepository:
    """Repository for message CRUD operations."""

    @staticmethod
    async def create(
        pool: asyncpg.Pool,
        session_id: UUID,
        role: str,
        content: str,
        mode: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> Message:
        """Create a new message.

        Args:
            pool: Database connection pool
            session_id: Session UUID
            role: 'user' or 'assistant'
            content: Message text
            mode: 'whole_book' or 'selected_text'
            metadata: Optional metadata dictionary

        Returns:
            Created Message instance

        Raises:
            asyncpg.exceptions.PostgresError: If database operation fails
            asyncpg.exceptions.CheckViolationError: If role or mode invalid
        """
        metadata = metadata or {}

        async with pool.acquire() as conn:
            # Set codec for JSONB handling
            await conn.set_type_codec(
                'jsonb',
                encoder=json.dumps,
                decoder=json.loads,
                schema='pg_catalog'
            )

            record = await conn.fetchrow(
                """
                INSERT INTO messages (session_id, role, content, mode, metadata)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id, session_id, role, content, created_at, mode, metadata
                """,
                session_id, role, content, mode, metadata
            )

        message = Message.from_record(record)
        logger.info(f"Created message: {message.id} (session: {session_id}, role: {role}, mode: {mode})")
        return message

    @staticmethod
    async def list_by_session(pool: asyncpg.Pool, session_id: UUID) -> List[Message]:
        """List all messages for a session (ordered by created_at ASC).

        Args:
            pool: Database connection pool
            session_id: Session UUID

        Returns:
            List of Message instances (may be empty)
        """
        async with pool.acquire() as conn:
            records = await conn.fetch(
                """
                SELECT id, session_id, role, content, created_at, mode, metadata
                FROM messages
                WHERE session_id = $1
                ORDER BY created_at ASC
                """,
                session_id
            )

        messages = [Message.from_record(record) for record in records]
        return messages

    @staticmethod
    async def get_by_id(pool: asyncpg.Pool, message_id: UUID) -> Optional[Message]:
        """Retrieve message by ID.

        Args:
            pool: Database connection pool
            message_id: Message UUID

        Returns:
            Message instance if found, None otherwise
        """
        async with pool.acquire() as conn:
            record = await conn.fetchrow(
                """
                SELECT id, session_id, role, content, created_at, mode, metadata
                FROM messages
                WHERE id = $1
                """,
                message_id
            )

        if record is None:
            return None

        return Message.from_record(record)


class SelectedTextMetadataRepository:
    """Repository for selected text metadata CRUD operations."""

    @staticmethod
    async def create(
        pool: asyncpg.Pool,
        message_id: UUID,
        selected_text: str,
        chapter_origin: Optional[str] = None
    ) -> SelectedTextMetadata:
        """Create selected text metadata record.

        Args:
            pool: Database connection pool
            message_id: Message UUID (foreign key)
            selected_text: Full text selected by user
            chapter_origin: Optional chapter name

        Returns:
            Created SelectedTextMetadata instance

        Raises:
            asyncpg.exceptions.PostgresError: If database operation fails
        """
        async with pool.acquire() as conn:
            record = await conn.fetchrow(
                """
                INSERT INTO selected_text_metadata (message_id, selected_text, chapter_origin)
                VALUES ($1, $2, $3)
                RETURNING id, message_id, selected_text, chapter_origin, created_at
                """,
                message_id, selected_text, chapter_origin
            )

        metadata = SelectedTextMetadata.from_record(record)
        logger.info(f"Created selected_text_metadata: {metadata.id} (message: {message_id})")
        return metadata

    @staticmethod
    async def get_by_message_id(
        pool: asyncpg.Pool,
        message_id: UUID
    ) -> Optional[SelectedTextMetadata]:
        """Retrieve selected text metadata by message ID.

        Args:
            pool: Database connection pool
            message_id: Message UUID

        Returns:
            SelectedTextMetadata instance if found, None otherwise
        """
        async with pool.acquire() as conn:
            record = await conn.fetchrow(
                """
                SELECT id, message_id, selected_text, chapter_origin, created_at
                FROM selected_text_metadata
                WHERE message_id = $1
                """,
                message_id
            )

        if record is None:
            return None

        return SelectedTextMetadata.from_record(record)
