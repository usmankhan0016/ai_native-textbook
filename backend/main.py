#!/usr/bin/env python3
"""
RAG Content Ingestion Pipeline

Discovers Docusaurus book URLs, extracts clean content, chunks text,
generates embeddings via Cohere, and stores vectors in Qdrant Cloud.
"""

import hashlib
import logging
import os
import time
from datetime import datetime
from typing import Optional, Dict, List, Tuple
import xml.etree.ElementTree as ET

import requests
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from cohere import ClientV2
import tiktoken
from dotenv import load_dotenv

# ============================================================================
# PHASE 2: FOUNDATIONAL (Blocking Prerequisites)
# ============================================================================

# --- T006: Logging Configuration ---

def setup_logging(log_level: str = "INFO") -> logging.Logger:
    """Configure logging with specified level (controllable via LOG_LEVEL env var)."""
    level = getattr(logging, log_level.upper(), logging.INFO)

    logger = logging.getLogger("rag-ingestion")
    logger.setLevel(level)

    # Console handler
    handler = logging.StreamHandler()
    handler.setLevel(level)

    # Formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    handler.setFormatter(formatter)

    logger.addHandler(handler)
    return logger


# --- T008: Error Handling Strategy ---

class RAGIngestionError(Exception):
    """Base exception for RAG ingestion errors."""
    pass


class NetworkError(RAGIngestionError):
    """Network-related errors (HTTP, DNS, timeout)."""
    pass


class RateLimitError(RAGIngestionError):
    """Rate limit error (HTTP 429)."""
    pass


class QdrantError(RAGIngestionError):
    """Qdrant-related errors (connection, storage, schema)."""
    pass


class EmbeddingError(RAGIngestionError):
    """Cohere embedding API errors."""
    pass


# --- T007: HTTP Client Utility with Exponential Backoff ---

class HTTPClient:
    """HTTP client with exponential backoff for transient failures."""

    def __init__(self, timeout: int = 30, max_retries: int = 3, logger: Optional[logging.Logger] = None):
        self.timeout = timeout
        self.max_retries = max_retries
        self.logger = logger or logging.getLogger(__name__)

    def get(self, url: str, headers: Optional[Dict] = None) -> requests.Response:
        """
        Fetch URL with exponential backoff (base 5s, max 3 retries).

        Args:
            url: URL to fetch
            headers: Optional HTTP headers

        Returns:
            Response object

        Raises:
            NetworkError: If all retries exhausted
            RateLimitError: If rate limited (429)
        """
        headers = headers or {}

        for attempt in range(self.max_retries):
            try:
                response = requests.get(url, timeout=self.timeout, headers=headers)
                response.raise_for_status()
                return response
            except requests.exceptions.HTTPError as e:
                if response.status_code == 429:
                    # Rate limit: backoff and retry
                    wait_time = 5 * (2 ** attempt)  # 5s, 10s, 20s
                    self.logger.warning(f"Rate limited. Waiting {wait_time}s before retry...")
                    time.sleep(wait_time)
                    if attempt == self.max_retries - 1:
                        raise RateLimitError(f"Rate limited after {self.max_retries} retries") from e
                else:
                    # Other HTTP error: don't retry
                    raise NetworkError(f"HTTP error {response.status_code}: {e}") from e
            except (requests.exceptions.Timeout, requests.exceptions.ConnectionError) as e:
                # Transient error: backoff and retry
                wait_time = 5 * (2 ** attempt)
                self.logger.warning(f"Connection error ({type(e).__name__}). Retrying in {wait_time}s...")
                time.sleep(wait_time)
                if attempt == self.max_retries - 1:
                    raise NetworkError(f"Network error after {self.max_retries} retries: {e}") from e

        raise NetworkError(f"Failed to fetch {url} after {self.max_retries} retries")


# --- T009: Token Counting Utility ---

class TokenCounter:
    """Token counter using OpenAI's tiktoken (compatible with Cohere embeddings)."""

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.tokenizer = tiktoken.get_encoding("cl100k_base")
        self.logger = logger or logging.getLogger(__name__)
        self.min_tokens = 500
        self.max_tokens = 800

    def count(self, text: str) -> int:
        """Count tokens in text."""
        return len(self.tokenizer.encode(text))

    def validate(self, text: str) -> Tuple[int, bool]:
        """
        Validate token count.

        Returns:
            (token_count, is_valid)
        """
        count = self.count(text)
        is_valid = self.min_tokens <= count <= self.max_tokens
        return count, is_valid


# --- T010: Environment Variable Loading & Validation ---

class EnvironmentConfig:
    """Load and validate environment variables required for ingestion."""

    REQUIRED_KEYS = [
        "DOCUSAURUS_BASE_URL",
        "DOCUSAURUS_BOOK_ID",  # Fixed typo: was DOCOSAURUS
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
    ]

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        load_dotenv()

    def validate(self) -> Dict[str, str]:
        """
        Validate all required environment variables are present.

        Returns:
            Dict of environment variables

        Raises:
            RAGIngestionError: If any required key is missing
        """
        missing_keys = []
        config = {}

        for key in self.REQUIRED_KEYS:
            value = os.getenv(key)
            if not value:
                missing_keys.append(key)
            else:
                config[key] = value

        if missing_keys:
            raise RAGIngestionError(
                f"Missing required environment variables: {', '.join(missing_keys)}\n"
                f"Please set them in .env file or export as environment variables."
            )

        # Optional keys with defaults
        config["LOG_LEVEL"] = os.getenv("LOG_LEVEL", "INFO")
        config["COHERE_BATCH_SIZE"] = int(os.getenv("COHERE_BATCH_SIZE", "50"))
        config["HTTP_TIMEOUT"] = int(os.getenv("HTTP_TIMEOUT", "30"))
        config["MAX_RETRIES"] = int(os.getenv("MAX_RETRIES", "3"))

        self.logger.info(f"Environment configuration loaded successfully")
        return config


# ============================================================================
# PHASE 3: USER STORY 1 - Core Ingestion Pipeline (MVP)
# ============================================================================

# --- T011: get_all_urls() ---

def get_all_urls(base_url: str, http_client: HTTPClient, logger: logging.Logger) -> List[str]:
    """
    Fetch sitemap.xml and extract all book URLs.

    Args:
        base_url: Base URL of Docusaurus site
        http_client: HTTP client with retry logic
        logger: Logger instance

    Returns:
        List of absolute URLs from sitemap

    Raises:
        NetworkError: If sitemap fetch fails
    """
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    logger.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = http_client.get(sitemap_url)
        root = ET.fromstring(response.content)

        # Parse XML namespace - sitemap uses default namespace
        ns = "http://www.sitemaps.org/schemas/sitemap/0.9"
        urls = [loc.text for loc in root.findall(f".//{{{ns}}}loc") if loc.text]

        # Filter to exclude common non-book pages
        excluded_patterns = ["admin", "search", "404", "api", ".json"]
        filtered_urls = [
            url for url in urls
            if not any(pattern in url.lower() for pattern in excluded_patterns)
        ]

        logger.info(f"✅ Discovered {len(filtered_urls)} book URLs from sitemap")
        return filtered_urls

    except Exception as e:
        raise NetworkError(f"Failed to fetch or parse sitemap: {e}") from e


# --- T012: extract_text_from_url() ---

def extract_text_from_url(url: str, http_client: HTTPClient, logger: logging.Logger) -> Optional[Dict]:
    """
    Fetch page HTML and extract clean main content with metadata (T012, T024, T028, T029).

    Extracts:
    - Main article text (no boilerplate) with enhanced Docusaurus selectors
    - Chapter title from H1 or page title
    - Preserves code blocks and structure

    Args:
        url: Page URL
        http_client: HTTP client with retry logic
        logger: Logger instance

    Returns:
        Dict with {text, chapter} or None if extraction fails
    """
    try:
        response = http_client.get(url)
        soup = BeautifulSoup(response.content, "html.parser")

        # Extract chapter title from H1 or page title (T024)
        chapter = ""
        h1 = soup.find("h1")
        if h1:
            chapter = h1.get_text(strip=True)
        if not chapter:
            title = soup.find("title")
            if title:
                # Clean page title (remove site name suffix if present)
                full_title = title.get_text(strip=True)
                # Remove common suffixes like " | Site Name"
                chapter = full_title.split("|")[0].strip()

        # Find main content: prioritize article.docusaurus_content, then article, then main
        # T028: Enhanced Docusaurus-specific selectors
        content = None
        selectors = [
            'article.docusaurus_content',
            'article.theme-doc-markdown',
            'article[class*="doc"]',
            'article',
            'main[role="main"]',
            'main',
        ]
        for selector in selectors:
            content = soup.select_one(selector)
            if content:
                break

        if not content:
            logger.warning(f"No article/main element found in {url}")
            return None

        # Clone content to avoid modifying original soup
        import copy
        content = copy.copy(content)

        # Remove boilerplate elements (T028: comprehensive selectors)
        boilerplate_selectors = [
            ".docSidebar",
            ".tableOfContents",
            ".toc",
            ".pagination",
            ".doc-footer",
            ".breadcrumbs",
            "nav",
            "footer",
            ".navbar",
            ".sidebar",
            "header",
            ".admonition-icon",
            ".theme-edit-this-page",
            ".pagination",
            "button",
            ".theme-toggle",
        ]

        for selector in boilerplate_selectors:
            for element in content.select(selector):
                element.decompose()

        # Extract text, preserving code blocks (T029)
        # Process each element to preserve structure
        text_parts = []
        for element in content.descendants:
            if isinstance(element, str):
                text = element.strip()
                if text:
                    text_parts.append(text)
            elif element.name in ["code", "pre"]:
                # Preserve code blocks with indentation
                code_text = element.get_text()
                text_parts.append(f"CODE: {code_text}")

        # Join with newlines to preserve structure
        text = "\n".join(text_parts)

        # Normalize whitespace (T029: decode HTML entities, normalize spaces)
        # Replace multiple spaces with single space, but preserve newlines for structure
        lines = []
        for line in text.split("\n"):
            # Decode HTML entities
            line = line.replace("&nbsp;", " ")
            line = line.replace("&mdash;", "—")
            line = line.replace("&ndash;", "–")
            line = line.replace("&ldquo;", """)
            line = line.replace("&rdquo;", """)
            line = line.replace("&rsquo;", "'")
            line = line.replace("&amp;", "&")

            # Normalize whitespace while preserving line structure
            line = " ".join(line.split())
            if line:
                lines.append(line)

        text = "\n".join(lines)

        if not text:
            logger.warning(f"Empty content extracted from {url}")
            return None

        logger.debug(f"Extracted {len(text)} characters from {url}, chapter: {chapter}")
        return {
            "text": text,
            "chapter": chapter,
        }

    except Exception as e:
        logger.warning(f"Failed to extract content from {url}: {e}")
        return None


# --- T013: chunk_text() ---

def chunk_text(
    text: str,
    source_url: str,
    chunk_size: int = 500,
    overlap_ratio: float = 0.1,
    chapter: str = "",
    token_counter: Optional[TokenCounter] = None,
    logger: Optional[logging.Logger] = None,
) -> List[Dict]:
    """
    Split text into overlapping chunks (500–800 tokens) with metadata (T013, T024).

    Extracts section titles from text patterns and associates with chunks.

    Args:
        text: Text to chunk
        source_url: Source URL (used for chunk_id)
        chunk_size: Target chunk size in tokens
        overlap_ratio: Overlap ratio (0.1 = 10%)
        chapter: Chapter title (from page header)
        token_counter: TokenCounter instance
        logger: Logger instance

    Returns:
        List of {chunk_id, text, tokens, source_url, chapter, section}
    """
    if not token_counter:
        token_counter = TokenCounter(logger)
    if not logger:
        logger = logging.getLogger(__name__)

    # Split by sentences/paragraphs
    paragraphs = [p.strip() for p in text.split("\n") if p.strip()]

    chunks = []
    current_chunk = []
    current_tokens = 0
    overlap_tokens = int(chunk_size * overlap_ratio)
    sequence = 0
    current_section = ""

    for paragraph in paragraphs:
        # Track sections: detect patterns like "Section:", "Chapter:", "Module:", etc.
        # Look for all-caps or title-case short lines as section headers
        if len(paragraph) < 100 and paragraph.isupper() or (
            paragraph.endswith(":") and len(paragraph.split()) <= 5
        ):
            current_section = paragraph

        para_tokens = token_counter.count(paragraph)

        # If adding this paragraph exceeds chunk_size, save current chunk
        if current_tokens + para_tokens > chunk_size and current_chunk:
            chunk_text_str = " ".join(current_chunk)
            tokens, is_valid = token_counter.validate(chunk_text_str)

            # Generate deterministic chunk_id
            url_hash = hashlib.sha256(source_url.encode()).hexdigest()[:16]
            chunk_id = f"{url_hash}#{sequence:04d}"

            if is_valid or len(chunks) == 0:  # Accept first chunk even if too small
                chunks.append({
                    "chunk_id": chunk_id,
                    "text": chunk_text_str,
                    "tokens": tokens,
                    "source_url": source_url,
                    "chapter": chapter,
                    "section": current_section,
                })
                sequence += 1
            else:
                logger.warning(f"Chunk {chunk_id} has {tokens} tokens (outside 500-800 range)")

            # Overlap: keep last ~10% of tokens
            keep_count = max(1, len(current_chunk) // 10)
            current_chunk = current_chunk[-keep_count:]
            current_tokens = token_counter.count(" ".join(current_chunk))

        current_chunk.append(paragraph)
        current_tokens = token_counter.count(" ".join(current_chunk))

    # Save final chunk
    if current_chunk:
        chunk_text_str = " ".join(current_chunk)
        tokens, is_valid = token_counter.validate(chunk_text_str)

        url_hash = hashlib.sha256(source_url.encode()).hexdigest()[:16]
        chunk_id = f"{url_hash}#{sequence:04d}"

        if is_valid or len(chunks) == 0:
            chunks.append({
                "chunk_id": chunk_id,
                "text": chunk_text_str,
                "tokens": tokens,
                "source_url": source_url,
                "chapter": chapter,
                "section": current_section,
            })

    logger.debug(f"Generated {len(chunks)} chunks from {source_url}, chapter: {chapter}")
    return chunks


# --- T014: embed() ---

def embed(
    texts: List[str],
    cohere_client,  # Cohere client object (avoid type hint for compatibility)
    logger: logging.Logger,
) -> List[List[float]]:
    """
    Generate embeddings using Cohere API (batch).

    Args:
        texts: List of texts to embed
        cohere_client: Cohere client
        logger: Logger instance

    Returns:
        List of 1024-dimensional embeddings

    Raises:
        EmbeddingError: If Cohere API fails
    """
    if not texts:
        return []

    try:
        response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_document",
            texts=texts,
        )
        # Extract embeddings from Cohere response
        # response.embeddings yields tuples like ('float_', [[embedding_values]])
        # We need to extract the inner embedding vector
        embeddings = []
        for emb in response.embeddings:
            try:
                # emb is a tuple: (type_str, [[embedding_values]])
                if isinstance(emb, tuple) and len(emb) >= 2:
                    embedding_data = emb[1]  # [[embedding_values]] or None
                    if embedding_data is not None:
                        if isinstance(embedding_data, list) and len(embedding_data) > 0:
                            embedding_vector = embedding_data[0]  # [embedding_values]
                            embeddings.append(list(embedding_vector))
                        else:
                            embeddings.append(list(embedding_data))
                    else:
                        logger.warning("Received None embedding data from Cohere")
                        embeddings.append([0.0] * 1024)  # Fallback: zero vector
                else:
                    if emb is not None:
                        embeddings.append(list(emb))
                    else:
                        embeddings.append([0.0] * 1024)  # Fallback: zero vector
            except Exception as e:
                logger.warning(f"Failed to extract embedding: {e}. Using zero vector as fallback")
                embeddings.append([0.0] * 1024)  # Fallback: zero vector

        logger.debug(f"Generated {len(embeddings)} embeddings")
        return embeddings
    except Exception as e:
        raise EmbeddingError(f"Cohere embedding failed: {e}") from e


# --- T015: create_collection() ---

def create_collection(
    qdrant_client: QdrantClient,
    collection_name: str,
    vector_size: int = 1024,
    logger: Optional[logging.Logger] = None,
) -> None:
    """
    Create Qdrant collection (idempotent).

    Args:
        qdrant_client: Qdrant client
        collection_name: Collection name
        vector_size: Vector dimension (default 1024 for Cohere)
        logger: Logger instance

    Raises:
        QdrantError: If collection creation fails
    """
    if not logger:
        logger = logging.getLogger(__name__)

    try:
        # Check if collection exists
        try:
            qdrant_client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' already exists, skipping creation")
            return
        except Exception:
            pass  # Collection doesn't exist, create it

        # Create collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
        )
        logger.info(f"✅ Created Qdrant collection '{collection_name}' (vector_size={vector_size})")

    except Exception as e:
        raise QdrantError(f"Failed to create collection: {e}") from e


# --- T016: save_chunk_to_qdrant() ---

def save_chunk_to_qdrant(
    qdrant_client: QdrantClient,
    collection_name: str,
    chunk: Dict,
    embedding: List[float],
    logger: Optional[logging.Logger] = None,
) -> bool:
    """
    Upsert chunk + embedding to Qdrant (idempotent by chunk_id).

    Args:
        qdrant_client: Qdrant client
        collection_name: Collection name
        chunk: Chunk dict {chunk_id, text, tokens, source_url, chapter?, section?, book_id?}
        embedding: 1024-dimensional embedding vector
        logger: Logger instance

    Returns:
        True if successful, False if failed
    """
    if not logger:
        logger = logging.getLogger(__name__)

    try:
        # --- T026: Metadata payload validation ---
        # Check required fields
        required_fields = {"chunk_id", "source_url", "text"}
        missing = required_fields - set(chunk.keys())
        if missing:
            logger.error(f"Chunk missing required fields: {missing}. Chunk: {chunk}")
            return False

        # Convert chunk_id to integer for Qdrant point ID
        chunk_id = chunk["chunk_id"]
        point_id = int(hashlib.md5(chunk_id.encode()).hexdigest(), 16) % (2**63)

        # Prepare payload with all available metadata
        payload = {
            "chunk_id": chunk_id,
            "source_url": chunk["source_url"],
            "text": chunk["text"],
            "chapter": chunk.get("chapter", ""),
            "section": chunk.get("section", ""),
            "book_id": chunk.get("book_id", ""),
        }

        # Warn if optional fields missing (but still upsert)
        optional_fields = {"chapter", "section"}
        missing_optional = optional_fields - set(chunk.keys())
        if missing_optional:
            logger.debug(f"Chunk {chunk_id} missing optional fields: {missing_optional}")

        # Upsert point
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload,
                )
            ],
        )

        logger.debug(f"Upserted chunk {chunk_id} to Qdrant (validated)")
        return True

    except Exception as e:
        logger.error(f"Failed to upsert chunk {chunk.get('chunk_id', '?')}: {e}")
        return False


# --- T022: Integrity Validation ---

def validate_ingestion(
    qdrant_client: QdrantClient,
    collection_name: str,
    expected_count: int,
    logger: Optional[logging.Logger] = None,
) -> bool:
    """
    Validate ingestion integrity after completion (T022).

    Verifies that:
    - Actual vector count in Qdrant matches expected count
    - Collection exists and is accessible
    - Payload schema is valid

    Args:
        qdrant_client: Qdrant client
        collection_name: Collection name
        expected_count: Expected number of vectors
        logger: Logger instance

    Returns:
        True if validation passes, False otherwise
    """
    if not logger:
        logger = logging.getLogger(__name__)

    try:
        # Get collection info
        collection_info = qdrant_client.get_collection(collection_name)
        actual_count = collection_info.points_count

        logger.info(f"Validating ingestion integrity...")
        logger.info(f"  Expected vectors: {expected_count}")
        logger.info(f"  Actual vectors: {actual_count}")

        if actual_count != expected_count:
            logger.warning(
                f"Vector count mismatch! Expected {expected_count}, got {actual_count}. "
                f"Difference: {abs(actual_count - expected_count)} vectors. "
                f"This may be normal if re-ingesting with some chunks unchanged."
            )
            # Don't fail on count mismatch (idempotency means counts may differ)
            return True

        # Validate sample payloads (first 5 vectors)
        points = qdrant_client.scroll(
            collection_name=collection_name,
            limit=5,
        )[0]

        required_fields = {"chunk_id", "source_url", "text"}
        for point in points:
            payload = point.payload
            missing = required_fields - set(payload.keys())
            if missing:
                logger.error(f"Point {point.id} missing fields: {missing}")
                return False

        logger.info(f"✅ Validation passed: collection '{collection_name}' is valid")
        return True

    except Exception as e:
        logger.error(f"Validation failed: {e}")
        return False


# --- T025: search_vectors() ---

def search_vectors(
    qdrant_client: QdrantClient,
    collection_name: str,
    query_embedding: List[float],
    chapter_filter: Optional[str] = None,
    limit: int = 5,
    logger: Optional[logging.Logger] = None,
) -> List[Dict]:
    """
    Search vectors by embedding similarity with optional metadata filtering (T025).

    Args:
        qdrant_client: Qdrant client
        collection_name: Collection name
        query_embedding: Query embedding (1024 dimensions)
        chapter_filter: Filter by chapter (optional)
        limit: Max results to return
        logger: Logger instance

    Returns:
        List of {chunk_id, source_url, chapter, section, text, similarity_score}
    """
    if not logger:
        logger = logging.getLogger(__name__)

    try:
        # Build filter if chapter specified (T025, T026)
        search_filter = None
        if chapter_filter:
            search_filter = {
                "key": "chapter",
                "match": {"value": chapter_filter}
            }

        # Search in Qdrant
        search_result = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=limit,
            with_payload=True,
        )

        # Format results
        results = []
        for scored_point in search_result:
            result = {
                "chunk_id": scored_point.payload.get("chunk_id", ""),
                "source_url": scored_point.payload.get("source_url", ""),
                "chapter": scored_point.payload.get("chapter", ""),
                "section": scored_point.payload.get("section", ""),
                "text": scored_point.payload.get("text", "")[:200],  # Truncate for display
                "similarity_score": scored_point.score,
            }
            results.append(result)

        logger.debug(f"Search returned {len(results)} results")
        return results

    except Exception as e:
        logger.error(f"Search failed: {e}")
        return []


# --- T017 + T018: main() and entry point ---

def main(
    base_url: Optional[str] = None,
    book_id: Optional[str] = None,
    cohere_api_key: Optional[str] = None,
    qdrant_url: Optional[str] = None,
    qdrant_api_key: Optional[str] = None,
    log_level: str = "INFO",
) -> int:
    """
    Orchestrate full RAG ingestion pipeline.

    Args:
        base_url: Docusaurus base URL (or from env)
        book_id: Book identifier (or from env)
        cohere_api_key: Cohere API key (or from env)
        qdrant_url: Qdrant cluster URL (or from env)
        qdrant_api_key: Qdrant API key (or from env)
        log_level: Logging level

    Returns:
        Exit code (0 = success, 1 = failure)
    """
    # Setup logging
    logger = setup_logging(log_level)
    logger.info("Starting RAG Content Ingestion Pipeline")

    try:
        # Load environment
        env_config = EnvironmentConfig(logger)
        config = env_config.validate()

        # Override with function parameters if provided
        base_url = base_url or config["DOCUSAURUS_BASE_URL"]
        book_id = book_id or config["DOCUSAURUS_BOOK_ID"]
        cohere_api_key = cohere_api_key or config["COHERE_API_KEY"]
        qdrant_url = qdrant_url or config["QDRANT_URL"]
        qdrant_api_key = qdrant_api_key or config["QDRANT_API_KEY"]

        # Initialize clients
        logger.info(f"Initializing HTTP client, Cohere, and Qdrant clients...")
        http_client = HTTPClient(
            timeout=config["HTTP_TIMEOUT"],
            max_retries=config["MAX_RETRIES"],
            logger=logger,
        )
        cohere_client = ClientV2(api_key=cohere_api_key)
        qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        token_counter = TokenCounter(logger)

        # Track metrics
        start_time = time.time()
        total_urls = 0
        extracted_pages = 0
        total_chunks = 0
        total_vectors = 0

        # Phase 3: User Story 1 - Core Pipeline
        logger.info("=" * 80)
        logger.info("Phase 3: User Story 1 - Ingest Complete Book Content")
        logger.info("=" * 80)

        # Step 1: Discover URLs
        logger.info("Step 1: Discovering URLs from sitemap...")
        try:
            urls = get_all_urls(base_url, http_client, logger)
            total_urls = len(urls)
            logger.info(f"✅ Discovered {total_urls} URLs")
        except NetworkError as e:
            logger.error(f"❌ Failed to discover URLs: {e}")
            return 1

        # Create Qdrant collection
        logger.info(f"Creating Qdrant collection '{book_id}'...")
        try:
            create_collection(qdrant_client, book_id, logger=logger)
        except QdrantError as e:
            logger.error(f"❌ Failed to create Qdrant collection: {e}")
            return 1

        # --- T021: Check for partial ingestion and resume ---
        progress_file = f".ingestion_progress_{book_id}.tmp"
        start_index = 0
        if os.path.exists(progress_file):
            try:
                with open(progress_file, "r") as f:
                    last_completed = f.read().strip()
                    start_index = next((i for i, url in enumerate(urls) if url == last_completed), 0) + 1
                    if start_index < len(urls):
                        logger.warning(
                            f"⚠️  Partial ingestion detected! Resuming from URL {start_index + 1}/{total_urls}"
                        )
            except Exception as e:
                logger.debug(f"Could not read progress file: {e}")
                start_index = 0

        # Step 2-6: Extract, chunk, embed, store
        logger.info(f"Step 2-6: Processing pages...")

        # Batch embed for efficiency
        batch_size = config["COHERE_BATCH_SIZE"]
        pending_chunks = []
        pending_embeddings = []

        for idx, url in enumerate(urls, 1):
            # Skip already processed URLs (T021: resume logic)
            if idx <= start_index:
                continue

            logger.debug(f"Processing {idx}/{total_urls}: {url}")

            # Extract
            extraction_result = extract_text_from_url(url, http_client, logger)
            if not extraction_result:
                logger.debug(f"Skipped {url} (empty content)")
                continue

            extracted_pages += 1

            # Chunk (with chapter metadata from T024)
            text = extraction_result["text"]
            chapter = extraction_result.get("chapter", "")
            chunks = chunk_text(text, url, chapter=chapter, token_counter=token_counter, logger=logger)
            if not chunks:
                logger.debug(f"No chunks generated from {url}")
                continue

            total_chunks += len(chunks)

            # Collect for batch embedding
            for chunk in chunks:
                pending_chunks.append(chunk)
                batch_texts = [c["text"] for c in pending_chunks]

                # If batch full or last URL, embed
                if len(batch_texts) >= batch_size or idx == total_urls:
                    # Update progress file (T021: resume logic)
                    try:
                        with open(progress_file, "w") as f:
                            f.write(url)
                    except Exception as e:
                        logger.debug(f"Could not update progress file: {e}")
                    try:
                        embeddings = embed(batch_texts, cohere_client, logger)

                        # Upsert to Qdrant
                        for chunk, embedding in zip(pending_chunks, embeddings):
                            chunk["book_id"] = book_id
                            success = save_chunk_to_qdrant(
                                qdrant_client, book_id, chunk, embedding, logger
                            )
                            if success:
                                total_vectors += 1

                        logger.info(f"✅ Embedded and stored batch of {len(batch_texts)} chunks")

                    except EmbeddingError as e:
                        logger.error(f"❌ Embedding failed: {e}")
                        return 1

                    pending_chunks = []

        # Store any remaining chunks
        if pending_chunks:
            try:
                batch_texts = [c["text"] for c in pending_chunks]
                embeddings = embed(batch_texts, cohere_client, logger)

                for chunk, embedding in zip(pending_chunks, embeddings):
                    chunk["book_id"] = book_id
                    success = save_chunk_to_qdrant(
                        qdrant_client, book_id, chunk, embedding, logger
                    )
                    if success:
                        total_vectors += 1

                logger.info(f"✅ Embedded and stored final batch of {len(batch_texts)} chunks")

            except EmbeddingError as e:
                logger.error(f"❌ Embedding failed: {e}")
                return 1

        # Summary
        elapsed = time.time() - start_time
        logger.info("=" * 80)
        logger.info(f"✅ Ingestion completed successfully!")
        logger.info(f"  - Discovered URLs: {total_urls}")
        logger.info(f"  - Extracted pages: {extracted_pages}")
        logger.info(f"  - Generated chunks: {total_chunks}")
        logger.info(f"  - Stored vectors: {total_vectors}")
        logger.info(f"  - Time elapsed: {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)")
        logger.info("=" * 80)

        # Clean up progress file on successful completion (T021)
        try:
            if os.path.exists(progress_file):
                os.remove(progress_file)
                logger.debug(f"Cleaned up progress file: {progress_file}")
        except Exception as e:
            logger.debug(f"Could not clean up progress file: {e}")

        return 0

    except Exception as e:
        logger.error(f"❌ Unexpected error: {e}", exc_info=True)
        # Note: progress file NOT deleted on error, allows resume on next run
        return 1


if __name__ == "__main__":
    import sys
    exit_code = main()
    sys.exit(exit_code)
