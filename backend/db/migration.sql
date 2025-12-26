-- Feature 007: RAG Agent Backend - Database Schema
-- Neon Serverless Postgres Migration
-- Created: 2025-12-20

-- Sessions table: Stores user chat sessions
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX IF NOT EXISTS idx_sessions_created_at ON sessions(created_at DESC);

-- Messages table: Stores chat messages (questions and agent responses)
CREATE TABLE IF NOT EXISTS messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    mode VARCHAR(20) NOT NULL CHECK (mode IN ('whole_book', 'selected_text')),
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX IF NOT EXISTS idx_messages_session_id ON messages(session_id, created_at ASC);
CREATE INDEX IF NOT EXISTS idx_messages_created_at ON messages(created_at DESC);

-- Selected text metadata table: Captures selected text used in queries
CREATE TABLE IF NOT EXISTS selected_text_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    selected_text TEXT NOT NULL,
    chapter_origin VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_selected_text_message_id ON selected_text_metadata(message_id);

-- Verification query: Check table counts
-- SELECT
--     (SELECT COUNT(*) FROM sessions) as sessions_count,
--     (SELECT COUNT(*) FROM messages) as messages_count,
--     (SELECT COUNT(*) FROM selected_text_metadata) as selected_text_count;
