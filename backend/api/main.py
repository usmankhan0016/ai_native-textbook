"""FastAPI application for RAG Agent Backend (Feature 007).

This module initializes the FastAPI application with:
- Health check endpoint
- API routing (sessions, chat)
- Lifespan context for database and agent initialization
- Error handling and logging
"""

import logging
import os
from pathlib import Path
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv

# Load environment variables from .env file
env_path = Path(__file__).parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

from db.connection import init_pool, close_pool
from agent.core import init_agent, shutdown_agent
from api.routes import sessions, chat

# Configure structured logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for startup and shutdown.

    Startup:
        - Initialize database connection pool
        - Initialize Gemini API client

    Shutdown:
        - Close database connection pool
        - Shutdown Gemini API client
    """
    # Startup
    logger.info("Initializing RAG Agent Backend...")

    try:
        await init_pool()
        logger.info("Database connection pool initialized")
    except Exception as e:
        logger.error(f"Failed to initialize database pool: {e}")
        raise

    try:
        await init_agent()
        logger.info("Gemini API client initialized")
    except Exception as e:
        logger.error(f"Failed to initialize Gemini agent: {e}")
        raise

    logger.info("RAG Agent Backend ready")

    yield

    # Shutdown
    logger.info("Shutting down RAG Agent Backend...")

    try:
        await close_pool()
        logger.info("Database connection pool closed")
    except Exception as e:
        logger.error(f"Error closing database pool: {e}")

    try:
        await shutdown_agent()
        logger.info("Gemini API client shutdown")
    except Exception as e:
        logger.error(f"Error shutting down Gemini agent: {e}")

    logger.info("RAG Agent Backend shutdown complete")


# Initialize FastAPI app with lifespan context
app = FastAPI(
    title="RAG Agent Backend",
    description="Production-ready RAG agent backend with dual-mode querying (whole-book and selected-text)",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS middleware for Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",              # Docusaurus local dev
        "http://localhost:8000",               # Local backend testing
        "https://usmankhan0016.github.io",    # Production GitHub Pages
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Global exception handlers
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions with structured error response.

    Args:
        request: The incoming request
        exc: The HTTPException that was raised

    Returns:
        JSONResponse with error details
    """
    logger.warning(
        f"HTTP {exc.status_code} error on {request.method} {request.url.path}: {exc.detail}"
    )

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": {
                "status_code": exc.status_code,
                "message": exc.detail,
                "path": str(request.url.path)
            }
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle unexpected exceptions with structured error response.

    Args:
        request: The incoming request
        exc: The exception that was raised

    Returns:
        JSONResponse with generic error message
    """
    logger.error(
        f"Unhandled exception on {request.method} {request.url.path}: {exc}",
        exc_info=True
    )

    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "status_code": 500,
                "message": "Internal server error",
                "path": str(request.url.path)
            }
        }
    )


# Include API routes
app.include_router(sessions.router)
app.include_router(chat.router)


@app.get("/health", tags=["Health"])
async def health_check():
    """Health check endpoint to verify API is running.

    Returns:
        dict: Status message indicating API health
    """
    return JSONResponse(
        status_code=200,
        content={"status": "healthy", "service": "rag-agent-backend"}
    )
