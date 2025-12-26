"""Database connection pooling for Neon Serverless Postgres (Feature 007).

This module provides:
- Asyncpg connection pool initialization
- Connection pool lifecycle management
- Dependency injection for FastAPI routes
"""

import os
import logging
from typing import Optional
import asyncpg

logger = logging.getLogger(__name__)


# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def create_pool() -> asyncpg.Pool:
    """Create and return an asyncpg connection pool.

    Configuration:
    - min_size=2: Keep 2 connections warm (reduces cold start latency)
    - max_size=10: Limit concurrent connections (Neon free tier max: 10)
    - timeout=30: 30-second timeout for acquiring connections

    Returns:
        asyncpg.Pool: Database connection pool

    Raises:
        ValueError: If DATABASE_URL not found in environment
        asyncpg.exceptions.PostgresError: If connection fails
    """
    database_url = os.getenv("DATABASE_URL")

    if not database_url:
        raise ValueError("DATABASE_URL not found in environment variables")

    # Remove channel_binding parameter for asyncpg compatibility
    database_url = database_url.replace("&channel_binding=require", "").replace("?channel_binding=require", "")

    logger.info("Creating database connection pool...")
    logger.info(f"Host: {database_url.split('@')[1].split('/')[0]}")

    pool = await asyncpg.create_pool(
        dsn=database_url,
        min_size=2,
        max_size=10,
        timeout=30,
        command_timeout=60,
    )

    logger.info("✅ Database connection pool created successfully")
    logger.info(f"Pool config: min_size=2, max_size=10, timeout=30s")

    return pool


async def get_pool() -> asyncpg.Pool:
    """Get the global connection pool instance.

    Returns:
        asyncpg.Pool: The active connection pool

    Raises:
        RuntimeError: If pool not initialized (call init_pool first)
    """
    if _pool is None:
        raise RuntimeError("Database pool not initialized. Call init_pool() during app startup.")

    return _pool


async def init_pool() -> None:
    """Initialize the global connection pool.

    Should be called during FastAPI app startup (lifespan context).
    """
    global _pool

    if _pool is not None:
        logger.warning("Database pool already initialized. Skipping re-initialization.")
        return

    _pool = await create_pool()


async def close_pool() -> None:
    """Close the global connection pool.

    Should be called during FastAPI app shutdown (lifespan context).
    """
    global _pool

    if _pool is None:
        logger.warning("Database pool not initialized. Nothing to close.")
        return

    logger.info("Closing database connection pool...")
    await _pool.close()
    _pool = None
    logger.info("✅ Database connection pool closed")


async def get_db_pool() -> asyncpg.Pool:
    """FastAPI dependency to inject database pool into route handlers.

    Usage:
        @app.get("/api/sessions/{session_id}/history")
        async def get_history(session_id: str, pool: asyncpg.Pool = Depends(get_db_pool)):
            async with pool.acquire() as conn:
                results = await conn.fetch("SELECT * FROM messages WHERE session_id = $1", session_id)
            return results

    Returns:
        asyncpg.Pool: The active connection pool
    """
    return await get_pool()


async def test_connection() -> bool:
    """Test database connectivity.

    Returns:
        bool: True if connection successful, False otherwise
    """
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            result = await conn.fetchval("SELECT 1")
            logger.info(f"✅ Database connection test successful (result: {result})")
            return True
    except Exception as e:
        logger.error(f"❌ Database connection test failed: {e}")
        return False
