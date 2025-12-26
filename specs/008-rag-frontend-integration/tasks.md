# Tasks: RAG Frontend Integration

**Input**: Design documents from `/specs/008-rag-frontend-integration/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅
**Branch**: `008-rag-frontend-integration`

**Organization**: Tasks are grouped by user story (US) to enable independent implementation and testing. Each story builds on foundational tasks but can be implemented in parallel with other stories.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story (US1, US2, US3, US4) - Setup/Foundational/Polish have no story
- All paths are absolute from repository root or relative to component

---

## Phase 1: Setup & Configuration

**Purpose**: Backend and environment setup for frontend-backend communication

### Backend Configuration

- [X] T001 Add CORS middleware to `backend/api/main.py` to allow Docusaurus origins (http://localhost:3000, https://usmankhan0016.github.io)
- [X] T002 [P] Verify FastAPI backend starts without errors and health check endpoint responds at `GET /health`

### Frontend Configuration

- [X] T003 [P] Create `.env.local` template file in `docusaurus_textbook/` with `REACT_APP_BACKEND_URL` variable and example values
- [X] T004 [P] Update `.gitignore` in `docusaurus_textbook/` to exclude `.env.local`

### Connectivity Verification

- [X] T005 Test backend is accessible from Docusaurus dev server using curl/Postman (both from browser and from server)
- [X] T006 Verify session creation endpoint (`POST /api/sessions`) returns valid UUID response

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core API client hooks and utilities that enable all user stories

### Session Management Hook

- [X] T007 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/useSession.ts` hook with:
  - localStorage-based session ID persistence
  - UUID generation for new sessions
  - Graceful fallback if localStorage unavailable
  - Export `{ sessionId, isSessionReady }`

- [X] T008 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/sessionStorage.ts` utility with:
  - `getSessionId()` - read from localStorage, URL params, or generate new
  - `saveSessionId(id: string)` - persist to localStorage
  - `clearSession()` - clear session from storage

### Chat API Hook

- [X] T009 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/useChat.ts` hook with:
  - `queryWholeBook(query: string)` - calls `POST /api/sessions/{id}/chat`
  - `querySelectedText(query: string, selectedText: string)` - calls `POST /api/sessions/{id}/selected-text-chat`
  - `getHistory()` - calls `GET /api/sessions/{id}/history`
  - Returns: `{ answer, sources, isLoading, error }`

- [X] T010 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/apiClient.ts` with:
  - `fetch` wrapper that:
    - Uses `REACT_APP_BACKEND_URL` from .env.local
    - Adds `Content-Type: application/json` headers
    - Handles response parsing
    - Catches and returns typed errors (HttpError with status code)

### Text Selection Hook

- [X] T011 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/useSelectedText.ts` hook with:
  - `getSelectedText()` - captures text selected on page
  - `getSelectionCoords()` - returns {x, y} for context menu positioning
  - `getChapterContext()` - optional: detect chapter origin from selected text location

### Error Handling Utilities

- [X] T012 [P] Create `docusaurus_textbook/src/components/ChatbotWidget/errorHandling.ts` with:
  - `getUserErrorMessage(statusCode: number, detail?: string)` - maps API errors to user-friendly messages
  - Error mappings:
    - 400 → "Your question is too long. Please try asking in 50,000 characters or fewer."
    - 404 → "Your session expired. Click here to start a new conversation."
    - 429 → "Service is busy. Please try again in 1 minute."
    - 500 → "Something went wrong. Please try again."
    - Network → "No internet connection. Please check your network and try again."
    - Timeout → "Request is taking longer than expected. You can wait or cancel."

---

## Phase 3: US1 - Whole-Book Query Mode

**User Story**: Student opens textbook → types question → receives answer with sources from Qdrant retrieval

**Priority**: P1 (Core MVP)

**Acceptance Criteria**:
- ✅ Question submitted via input box + send button
- ✅ "thinking..." indicator shown during API call
- ✅ Answer displays as bot message in chat
- ✅ Sources list displays with chapter name, relevance score, text preview
- ✅ Multiple consecutive queries work in same session
- ✅ Query completes in <15 seconds (success path)

### Implementation

- [X] T013 [US1] Modify `docusaurus_textbook/src/components/ChatbotWidget/index.tsx` to:
  - Import `useSession`, `useChat`, `useSelectedText` hooks
  - Initialize hooks in component body
  - Pass sessionId to useChat calls

- [X] T014 [US1] [P] Add input state and send handler to ChatbotWidget:
  - Add `inputValue` state for textarea
  - Add `handleSendMessage` function that:
    - Adds user message to local messages array
    - Calls `queryWholeBook(inputValue)` via useChat hook
    - Clears input field after send

- [X] T015 [US1] [P] Create `docusaurus_textbook/src/components/ChatbotWidget/SourcesList.tsx` component that displays:
  - Array of sources from API response
  - Each source shows: chapter name, relevance score (as %, e.g., "85%"), text preview (first 150 chars)
  - Styled to match existing ChatbotWidget theme (blue gradient, white text on dark bg)

- [X] T016 [US1] [P] Add "thinking..." loading indicator to ChatbotWidget:
  - Show animated three-dot indicator when `isLoading === true`
  - Hide input/send button while loading (disabled state at 0.5 opacity)
  - Use CSS animation from existing styles.module.css

- [X] T017 [US1] Update ChatbotWidget message rendering to:
  - Display bot response as chat bubble (use existing message bubble styling)
  - If response contains sources: render SourcesList below answer
  - If response contains error: handle via error handler (Phase 7)

### Testing

- [ ] T018 [US1] Manual test: Ask whole-book question "What is URDF?" → verify answer appears with 3-5 sources
- [ ] T019 [US1] Manual test: Ask another question in same session → verify both questions show in chat history
- [ ] T020 [US1] Manual test: Verify "thinking..." indicator appears during API call and disappears when answer arrives
- [ ] T021 [US1] Manual test: Verify API response time is <15 seconds (measure in browser DevTools)

---

## Phase 4: US2 - Session Persistence & History

**User Story**: Student closes browser → returns hours later → chat history restored from database

**Priority**: P1 (Core MVP)

**Acceptance Criteria**:
- ✅ Session ID persists in localStorage across page refresh
- ✅ Chat history loads on component mount
- ✅ All previous messages visible after page reload
- ✅ Session restored if localStorage lost (create new session gracefully)
- ✅ History load completes in <1 second

### Implementation

- [X] T022 [US2] Add session restoration to ChatbotWidget component:
  - Call `getSessionId()` on mount
  - If session exists: call `getHistory()` to load previous messages
  - Display loaded messages in chat interface
  - Handle case where session exists in DB but not in localStorage (restore from DB)

- [X] T023 [US2] [P] Implement localStorage persistence:
  - After each user message: call `saveSessionId(sessionId)` to ensure localStorage is up-to-date
  - After each API response: optionally cache metadata in localStorage for faster recovery

- [X] T024 [US2] [P] Add error handling for localStorage failures:
  - If localStorage unavailable (privacy mode): gracefully fall back to in-memory session
  - Session data still persists in backend Postgres
  - On next visit, no local history but fresh session created

- [X] T025 [US2] Update ChatbotWidget initialization:
  - On component mount: fetch and display chat history in chronological order
  - Don't show "welcome" message if history exists
  - Smooth scroll to latest message after history loads

### Testing

- [X] T026 [US2] Manual test: Ask 3 questions → refresh page → verify all 3 questions and answers still visible
- [X] T027 [US2] Manual test: Check browser localStorage (DevTools) → verify sessionId key exists
- [X] T028 [US2] Manual test: Close browser completely → reopen textbook → history still visible
- [X] T029 [US2] Manual test: Clear localStorage → refresh → verify new session created (old messages gone)
- [X] T030 [US2] Manual test: History load time <1 second (measure in DevTools Network tab)

---

## Phase 5: US3 - Selected-Text Query Mode

**User Story**: Student selects text → right-clicks → "Ask about this text" → gets answer from selected text only

**Priority**: P2 (Feature extension)

**Acceptance Criteria**:
- ✅ Right-click on selected text shows context menu
- ✅ Context menu shows "✨ Ask about this text" option
- ✅ Clicking opens chat with selected text pre-filled in input
- ✅ Selected-text query submitted and answer displayed
- ✅ Response shows "Based on your selected text:" prefix
- ✅ No sources displayed for selected-text queries
- ✅ Query completes in <5 seconds (no Qdrant retrieval)

### Implementation

- [X] T031 [US3] Create `docusaurus_textbook/src/components/ChatbotWidget/SelectionContextMenu.tsx` component that:
  - Positioned absolutely at mouse click coordinates
  - Shows button: "💬 Ask about this text"
  - Styled to match ChatbotWidget theme
  - Auto-hides on click or when clicking outside

- [X] T032 [US3] Implement text selection handler in ChatbotWidget:
  - Add `mouseup` and `keyup` event listeners to document
  - Get selected text: `window.getSelection().toString()`
  - If text selected (>=10 chars): show ContextMenu at selection coordinates
  - Handle menu close and selection clearing

- [X] T033 [US3] [P] Add selected-text input state to ChatbotWidget:
  - Add `selectedText` and `selectionMenuPosition` state
  - When context menu option clicked: populate with selected text
  - Pre-fill in chat input field
  - Open chatbot if not already open

- [X] T034 [US3] [P] Create selected-text query submission:
  - Modify `handleSendMessage` to detect mode (whole-book vs selected-text)
  - Call `querySelectedText(query, selectedText)` when selected text mode
  - Pass selected text metadata with message

- [X] T035 [US3] Update ChatbotWidget rendering for selected-text responses:
  - Add purple "📝 Selected Text Query" badge to messages
  - Show quoted selected text in user messages
  - Do NOT render SourcesList component (sources should be null from API)

- [X] T036 [US3] Clean up after selected-text query:
  - Clear `selectedText` after message sent
  - Reset UI to whole-book mode for next query

### Testing

- [X] T037 [US3] Manual test: Select text on page → "Ask about this" appears above selection
- [X] T038 [US3] Manual test: Click option → chat opens with text pre-filled
- [X] T039 [US3] Manual test: Ask question → response shows with purple badge and quoted text
- [X] T040 [US3] Manual test: Verify no sources shown in response
- [X] T041 [US3] Manual test: Selected-text query completes in <5 seconds
- [X] T042 [US3] Manual test: Multiple selected-text queries in same session work correctly

---

## Phase 6: US4 - Multi-Turn Conversations

**User Story**: Student asks Q1 → gets answer → asks Q2 follow-up → history shows both exchanges

**Priority**: P2 (Feature enhancement)

**Acceptance Criteria**:
- ✅ Follow-up questions work in same session
- ✅ Chat history shows all questions and answers in chronological order
- ✅ Session persists across multiple turns
- ✅ Each turn is independently queryable (can refresh and history remains)

### Implementation

- [X] T043 [US4] Verify ChatbotWidget supports multiple message submissions:
  - After US1 (whole-book) implementation: verify can ask 2+ questions
  - After US2 (history) implementation: verify history shows all exchanges
  - Input field clears after each send and accepts new input

- [X] T044 [US4] [P] Verify chat history display shows mixed message types:
  - User messages and bot responses interleaved
  - Chronological order (oldest first)
  - Timestamps on messages (optional but recommended)
  - Correct visual distinction (user = blue bubble, bot = white/dark bubble)

- [X] T045 [US4] [P] Ensure context loaded on fresh visit:
  - When user returns to textbook with session ID in localStorage
  - Full conversation history loads
  - Can continue asking follow-up questions

- [X] T046 [US4] Test session metadata tracks multi-turn:
  - Verify `updated_at` timestamp updates with each new message
  - Verify message count increases in database

### Testing

- [X] T047 [US4] Manual test: Ask 3-4 questions in sequence → all visible in chat
- [X] T048 [US4] Manual test: Close browser → reopen → full 3-4 question history still there
- [X] T049 [US4] Manual test: Add 5th question → 5 questions total visible
- [X] T050 [US4] Manual test: Verify message order is chronological (Q1, A1, Q2, A2, Q3, A3...)

---

## Phase 7: Polish - Error Handling & UX

**Purpose**: Graceful error handling, timeouts, accessibility, cross-browser compatibility

**Acceptance Criteria**:
- ✅ All 6 HTTP error codes show friendly messages
- ✅ Network errors handled without crashing
- ✅ Timeout warning at 12 seconds for whole-book queries
- ✅ Error boundary prevents full-page crashes
- ✅ UI fully responsive during loading (no freezing, no main thread blocking)
- ✅ Works on Chrome, Firefox, Safari, Edge (modern versions)

### Error Message Display

- [X] T051 Create error message handler in ChatbotWidget:
  - When API call fails: extract error code and detail from response
  - Call `getUserErrorMessage()` utility (from Phase 2 T012)
  - Add error message as bot message with error styling (red/orange background)
  - Show "Retry" button for transient errors

- [X] T052 [P] Implement HTTP 400 error handling:
  - Error message: "Your question is too long. Please try asking in 50,000 characters or fewer."
  - Show in chat as bot message
  - Focus input for retry

- [X] T053 [P] Implement HTTP 404 error handling:
  - Error message: "Your session expired. Click here to start a new conversation."
  - Include button to create new session
  - Clear localStorage and previous messages
  - Generate new session ID

- [X] T054 [P] Implement HTTP 429 error handling:
  - Error message: "Service is busy. Please try again in 1 minute."
  - Disable send button for 60 seconds
  - Show countdown timer "Retry in X seconds"
  - Re-enable after timeout

- [X] T055 [P] Implement HTTP 500 error handling:
  - Error message: "Something went wrong. Please try again."
  - Show "Retry" button
  - Log error to browser console for debugging
  - Don't crash UI

- [X] T056 [P] Implement network error handling:
  - Catch `NetworkError`, `TypeError` from fetch
  - Error message: "No internet connection. Please check your network and try again."
  - Show in chat with error styling
  - Auto-retry on network reconnect (monitor `navigator.onLine`)

- [X] T057 [P] Implement timeout warning:
  - For whole-book queries: show warning at 12s that response is slow
  - For selected-text queries: show warning at 4.5s
  - Show message: "Request is taking longer than expected. You can wait or cancel."
  - Include "Cancel" button to abort request

### Error Boundary Component

- [X] T058 Create `docusaurus_textbook/src/components/ChatbotWidget/ErrorBoundary.tsx`:
  - Wraps ChatbotWidget to catch React errors
  - Displays fallback UI: "Chat service unavailable. Please refresh the page."
  - Logs error for debugging
  - Allows page to continue functioning (doesn't crash entire site)

- [X] T059 Wrap ChatbotWidget with ErrorBoundary in index.tsx (integrated directly)

### UI Responsiveness

- [X] T060 Verify send button disabled during loading:
  - While `isLoading === true`: send button opacity 0.5, pointer-events disabled
  - Input textarea disabled (readonly)
  - User can't submit multiple queries simultaneously

- [X] T061 [P] Verify no main-thread blocking:
  - Long API responses don't freeze UI
  - Chat remains scrollable while loading
  - Typing in input field is responsive (not laggy)

- [X] T062 [P] Test loading states are visible:
  - "thinking..." indicator clearly visible
  - Loading state lasts entire duration of API call
  - Disappears only when response or error arrives

### Browser Compatibility

- [ ] T063 Test on Chrome (latest):
  - All UI renders correctly
  - Fetch API works
  - localStorage accessible
  - Right-click context menu works

- [ ] T064 [P] Test on Firefox (latest):
  - Same as Chrome checks
  - Verify all event handlers work (contextmenu event)

- [ ] T065 [P] Test on Safari (latest):
  - Verify fetch API (may need polyfill)
  - localStorage works
  - CSS animations smooth

- [ ] T066 [P] Test on Edge (latest):
  - Same checks as Chrome (Chromium-based)

- [ ] T067 [P] Test on mobile browsers:
  - Chat widget visible and usable on phones
  - Right-click context menu works or fallback UX provided
  - Touch interactions responsive

### Final Validation

- [X] T068 Verify no console errors or warnings (only expected logs)
- [X] T069 Verify all UI elements have proper aria-labels for accessibility
- [X] T070 Verify keyboard navigation works (Tab, Enter, Escape keys)
- [X] T071 Verify chat widget doesn't block page scrolling
- [ ] T072 Manual end-to-end test: Whole-book query → Selected-text query → Follow-up → Refresh → History intact (requires user testing)

---

## Implementation Strategy & Dependencies

### Phase Sequence

1. **Phase 1 (Setup)**: 1-2 hours
   - Must complete before any feature development
   - Unblocks all user stories

2. **Phase 2 (Foundational)**: 4-6 hours
   - Must complete before Phase 3, 4, 5, 6
   - All hooks and utilities needed by all stories
   - Can be parallelized (T007-T012 are independent)

3. **Phase 3 (US1 - Whole-Book)**: 4-6 hours
   - **Parallelizable**: T014-T017 can run in parallel
   - Core MVP - implement first
   - Unblocks US4 (multi-turn depends on basic queries working)

4. **Phase 4 (US2 - Session Persistence)**: 2-3 hours
   - Can run parallel to Phase 3
   - Depends only on Phase 2
   - Unblocks US4 (history display)

5. **Phase 5 (US3 - Selected-Text)**: 3-4 hours
   - Can run parallel to Phase 3 & 4
   - Depends only on Phase 2
   - Independent feature

6. **Phase 6 (US4 - Multi-Turn)**: 2-3 hours
   - Depends on Phase 3 + Phase 4
   - Tests multi-query behavior
   - Requires US1 and US2 working

7. **Phase 7 (Polish)**: 5-8 hours
   - Can start parallel to Phase 3-6 for error handling framework
   - Full polish only after all features working
   - Browser testing last

### Parallelization Opportunities

**Can run in parallel**:
- Phase 2 tasks T007-T012 (all independent hooks/utilities)
- Phase 3 tasks T014-T017 (input, display, sources, loading indicator)
- Phase 5 can start while Phase 3 is in progress (independent context menu)

**Must be sequential**:
- Phase 1 → Phase 2 (setup before hooks)
- Phase 2 → Phase 3, 4, 5 (hooks before features)
- Phase 3 + Phase 4 → Phase 6 (individual features before multi-turn)
- Phase 7 can overlap with phases 3-6 but full validation requires them complete

### MVP Scope (Minimum Viable Product)

Deliver phases **1, 2, 3, 4** to get core functionality working:
- ✅ Whole-book Q&A with sources (US1)
- ✅ Session persistence and history (US2)
- ✅ Basic error handling (subset of Phase 7)

**Estimated effort**: 12-16 hours of focused development

### Add-on Features (Phase 5-7)

After MVP working, add:
- ✅ Selected-text mode (US3) - 3-4 hours
- ✅ Multi-turn testing (US4) - 2-3 hours
- ✅ Full polish & browser testing (Phase 7) - 5-8 hours

**Total effort**: 22-32 hours

---

## Success Criteria Summary

### By End of Phase 3 (US1 Complete)
- Student can ask a whole-book question
- Answer appears with sources
- Loading indicator shows during query
- No UI freezing

### By End of Phase 4 (US2 Complete)
- Session ID persists across page refresh
- Chat history loads on return visit
- Both core features testable independently

### By End of Phase 5 (US3 Complete)
- Right-click context menu works
- Selected-text queries return answers without external sources
- All 3 query modes (whole-book, selected-text, follow-up) functional

### By End of Phase 6 (US4 Complete)
- Multi-turn conversations work
- Full chat history visible and persistent
- All 4 user stories testable

### By End of Phase 7 (Polish Complete)
- All error codes show friendly messages
- No crashes or console errors
- Works on 4+ modern browsers
- UI responsive under all conditions
- Ready for production deployment

---

**Status**: ✅ Tasks generated and ready for implementation
**Total Tasks**: 72 (T001-T072)
**Recommended Start**: Phase 1 → Phase 2 → Phase 3 + Phase 4 parallel → Phase 5 → Phase 6 → Phase 7
**Estimated Total Effort**: 24-36 hours (developer-dependent)
