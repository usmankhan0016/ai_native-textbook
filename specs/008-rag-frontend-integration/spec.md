# Feature 008: RAG Frontend Integration
## Integrate Docusaurus Chatbot with RAG Backend

**Status**: Specification
**Feature ID**: 008-rag-frontend-integration
**Created**: 2025-12-22
**Last Updated**: 2025-12-22

---

## Clarifications

### Session 2025-12-22
- Q: Which Docusaurus chatbot component to integrate with? → A: `docusaurus_textbook/src/components/ChatbotWidget`
- Q: How should frontend discover backend URL? → A: Environment variable in `.env.local`
- Q: How should selected-text affordance appear? → A: Context menu on right-click (right-click → "Ask about this text")

---

## Overview

Integrate the existing chatbot UI in the Docusaurus textbook with the RAG backend (Feature 007) to enable intelligent question answering using either whole-book context or user-selected text. The integration reuses the existing chatbot component and connects it to FastAPI endpoints, enabling end-to-end RAG functionality without UI redesign.

---

## Problem Statement

**Current State**: The Docusaurus textbook has a chatbot widget, but it is not connected to any backend. Users cannot ask questions or receive answers.

**Desired State**: Users can ask questions about the textbook content in two modes:
1. **Whole-Book Mode**: Agent retrieves relevant chapters from Qdrant and generates grounded answers with source attribution
2. **Selected-Text Mode**: Agent answers using ONLY user-selected text, with zero external knowledge

---

## Success Criteria

- ✓ Chat UI remains functional and is not replaced (changes are additive)
- ✓ Users can ask whole-book questions and receive grounded answers with source attribution
- ✓ Users can ask selected-text questions and receive answers using only the provided text
- ✓ Session IDs persist across page refreshes and user interactions
- ✓ Chat history is visible to users (conversation persists)
- ✓ API requests complete in under 15 seconds (whole-book) and under 5 seconds (selected-text)
- ✓ UI remains responsive during API calls (no freezing or blocking)
- ✓ Error messages are user-friendly and actionable
- ✓ All chat interactions are persisted in Neon Postgres

---

## User Scenarios

### Scenario 1: Ask a Whole-Book Question
**Actor**: Student reading the textbook
**Goal**: Understand a concept across multiple chapters

**Flow**:
1. Student opens the textbook in browser
2. Sees chatbot widget in sidebar or bottom-right
3. Types question: "What is URDF and how is it used in robotics?"
4. Chatbot shows "thinking..." indicator
5. Backend retrieves relevant chapters from Qdrant
6. Backend generates answer citing specific chapters and sections
7. Student sees answer with sources listed (Chapter name, relevance score, text preview)
8. Student can ask follow-up questions in same session

**Success**: Student gets accurate, grounded answer with citations

---

### Scenario 2: Ask a Selected-Text Question
**Actor**: Student reading a specific chapter

**Goal**: Get clarification on selected passage without external information

**Flow**:
1. Student selects text from a paragraph (e.g., "URDF is an XML format...")
2. Student right-clicks on selected text
3. Context menu appears with "Ask about this text" option
4. Student clicks the option; chatbot opens with selected text pre-filled
5. Student types question: "What does this mean?"
6. Backend generates answer using ONLY the selected text
7. Student sees answer prefixed with "Based on your selected text:"
8. No external sources shown (sources=null)

**Success**: Student gets clarification grounded in the exact passage

---

### Scenario 3: Multi-Turn Conversation
**Actor**: Student exploring a topic

**Goal**: Have a natural conversation across multiple questions

**Flow**:
1. Student asks Q1: "What is Isaac Sim?"
2. Gets answer with 5 sources from Chapters 1-3
3. Student asks Q2: "What are the benefits?"
4. Agent retrieves new chunks but may reference prior context
5. Student sees chat history showing all previous exchanges
6. Session ID persists, all messages stored

**Success**: Conversation feels natural, history is preserved

---

### Scenario 4: Session Persistence
**Actor**: Student taking a break

**Goal**: Continue conversation after closing browser

**Flow**:
1. Student asks 3 questions, builds chat history
2. Closes browser/navigates away
3. Returns to page hours later
4. Session ID is restored (via URL param, localStorage, or cookie)
5. Chat history is restored from database
6. Student can see all previous messages and continue

**Success**: No chat history is lost

---

## Functional Requirements

### Chat UI Integration
- **F-001**: The existing chatbot component in `docusaurus_textbook/src/components/ChatbotWidget` is reused (no replacement)
- **F-002**: Chat UI supports two query modes:
  - **Whole-Book**: Query all textbook content via Qdrant
  - **Selected-Text**: Query only user-provided text
- **F-003**: UI provides context menu affordance: right-click on selected text → "Ask about this text" option
- **F-004**: UI shows "thinking..." / loading indicator during API calls
- **F-005**: Chat messages are displayed in chronological order (oldest first)
- **F-006**: User messages and assistant responses are visually distinguishable

### Session Management
- **F-007**: Session ID is generated on first chatbot interaction or when page loads
- **F-008**: Session ID persists across page refreshes and browser closures (localStorage / URL parameter)
- **F-009**: Session ID is sent to backend with every API request
- **F-010**: Neon Postgres stores session metadata (created_at, updated_at, user_metadata if needed)

### API Integration
- **F-011**: Chat UI calls FastAPI `/api/sessions` endpoint to create sessions
- **F-012**: Chat UI calls `/api/sessions/{session_id}/chat` for whole-book queries
- **F-013**: Chat UI calls `/api/sessions/{session_id}/selected-text-chat` for selected-text queries
- **F-014**: Chat UI calls `/api/sessions/{session_id}/history` to retrieve chat history
- **F-015**: All API calls include `Content-Type: application/json` header
- **F-016**: API responses are parsed and displayed correctly in UI

### Whole-Book Mode
- **F-017**: Query is sent to backend with `{"query": "user input", "mode": "whole_book"}`
- **F-018**: Backend returns sources list with chapter name, relevance score (0-1), and text preview
- **F-019**: Sources are displayed in UI below the answer (e.g., "Sources: Chapter 2: Isaac Sim (0.85)")
- **F-020**: Relevance scores are shown as percentages or visual indicators

### Selected-Text Mode
- **F-021**: User selects text in the document; right-click shows context menu with "Ask about this text" option
- **F-022**: Clicking context menu option opens chat with selected text pre-filled
- **F-023**: Query is sent with `{"query": "user input", "selected_text": "...", "mode": "selected_text"}`
- **F-024**: Response is prefixed with "Based on your selected text:" in UI
- **F-025**: No sources are displayed (sources field is null)
- **F-026**: Selected text origin (chapter) is optionally captured and stored

### Error Handling
- **F-027**: HTTP 400 errors (invalid input) are shown as user-friendly messages
  - Example: "Query must be 1-50,000 characters"
- **F-028**: HTTP 404 errors (session not found) are handled gracefully
  - Example: "Your session expired. Click here to start a new conversation"
- **F-029**: HTTP 429 errors (rate limit) show retry message
  - Example: "Service is busy. Please try again in 1 minute"
- **F-030**: HTTP 500 errors (internal error) show generic message
  - Example: "Something went wrong. Please try again"
- **F-031**: Network errors (no internet) are handled without crashing UI
- **F-032**: Long-running requests (>15s whole-book, >5s selected-text) show timeout warning

### Data Persistence
- **F-033**: Every user message is stored in Neon Postgres with session_id, role="user", content, mode, timestamp
- **F-034**: Every assistant response is stored with session_id, role="assistant", content, mode, timestamp, metadata
- **F-035**: Selected text metadata is stored with message_id, selected_text, chapter_origin
- **F-036**: Chat history API returns all messages for a session in chronological order

### Performance
- **F-037**: Whole-book query completes in under 15 seconds (including Qdrant retrieval + Gemini generation)
- **F-038**: Selected-text query completes in under 5 seconds (Gemini only, no retrieval)
- **F-039**: Chat history retrieval completes in under 1 second
- **F-040**: UI remains responsive during API calls (no blocking, no main thread freezing)

---

## Key Entities

### Session
- **session_id** (UUID): Unique identifier
- **created_at** (datetime): When session was created
- **updated_at** (datetime): Last activity timestamp
- **metadata** (JSON): Extensible metadata (browser type, user info, etc.)

### Message
- **message_id** (UUID): Unique identifier
- **session_id** (UUID): Foreign key to session
- **role** ("user" | "assistant"): Message author
- **content** (text): Message text
- **created_at** (datetime): Timestamp
- **mode** ("whole_book" | "selected_text"): Query mode used
- **metadata** (JSON): Latency, model, retrieval_count, etc.

### SelectedTextMetadata
- **metadata_id** (UUID): Unique identifier
- **message_id** (UUID): Foreign key to user message
- **selected_text** (text): Full selected text
- **chapter_origin** (string): Optional chapter name
- **created_at** (datetime): Timestamp

---

## Assumptions

- The Docusaurus textbook has an existing chatbot component (`docusaurus_textbook/src/components/ChatbotWidget`) that can be modified
- The RAG backend (Feature 007) is deployed and accessible via environment variable (`REACT_APP_BACKEND_URL` in `.env.local`)
- Session storage uses localStorage or URL parameters (no server-side sessions)
- CORS is configured to allow Docusaurus frontend to call FastAPI backend
- User authentication is not required (chat is public)
- Selected-text functionality is optional; whole-book queries are primary
- Selected-text affordance is implemented via right-click context menu (not inline button or floating toolbar)

---

## Constraints & Non-Functional Requirements

### Constraints
- **Must not** replace or remove existing chatbot UI
- **Must not** require authentication or user accounts
- **Must not** add major UI redesign or styling changes
- **Must not** modify backend logic (Feature 007)
- **Must** complete integration within 1 week
- **Must** be non-breaking changes (additive only)

### Non-Functional
- **Performance**: Whole-book queries <15s, selected-text <5s, history <1s
- **Availability**: Backend must be accessible 24/7 (or gracefully degrade if down)
- **Reliability**: No chat messages are lost
- **Responsiveness**: UI never blocks; loading states are shown
- **Compatibility**: Works on Chrome, Firefox, Safari, Edge (modern versions)
- **Accessibility**: Chat widget is keyboard-navigable (Tab, Enter, Escape)

---

## Out of Scope

- ❌ New chatbot UI from scratch
- ❌ Backend logic changes or new models
- ❌ User authentication or accounts
- ❌ Major UI redesign or theme changes
- ❌ Analytics, telemetry, or logging dashboards
- ❌ Mobile app (web only)
- ❌ Voice input/output
- ❌ Real-time collaboration
- ❌ Rate limiting per user

---

## Dependencies

- **Feature 007: RAG Agent Backend** (must be deployed and accessible)
- **Docusaurus Chatbot Component** (existing in the codebase)
- **Neon Serverless Postgres** (for data persistence)
- **Qdrant Vector Database** (for retrieval)
- **Gemini API** (for generation)

---

## Testing Strategy

### User Acceptance Tests
1. **Whole-Book Query Flow**
   - Open textbook → Type question → Receive answer with sources ✓

2. **Selected-Text Query Flow**
   - Select text → Click button → Ask question → Receive answer ✓

3. **Multi-Turn Conversation**
   - Ask 3+ questions → View history → Verify persistence ✓

4. **Session Persistence**
   - Close/reopen browser → Session ID restored → History visible ✓

5. **Error Handling**
   - Invalid input → Friendly error message ✓
   - Network down → UI gracefully degrades ✓
   - Long request → Timeout warning after 15s ✓

---

## Acceptance Checklist

- [ ] Existing chatbot UI is unchanged (only JS/API integration added)
- [ ] Both whole-book and selected-text modes work end-to-end
- [ ] Session IDs persist across page refreshes
- [ ] Chat history is visible and retrievable
- [ ] Sources are displayed for whole-book queries
- [ ] No sources shown for selected-text queries
- [ ] Error messages are user-friendly
- [ ] Performance targets met (<15s whole-book, <5s selected-text)
- [ ] UI remains responsive during API calls
- [ ] All chat data persisted in Neon Postgres
- [ ] No console errors or warnings
- [ ] Works on Chrome, Firefox, Safari, Edge
