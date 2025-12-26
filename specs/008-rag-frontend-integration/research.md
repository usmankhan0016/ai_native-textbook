# Phase 0 Research: RAG Frontend Integration

**Date**: 2025-12-22
**Feature**: 008-rag-frontend-integration
**Status**: Complete - All 5 unknowns resolved

---

## Unknown 1: ChatbotWidget Implementation Details

**Question**: How is the existing ChatbotWidget implemented? What API can we use to integrate backend calls?

**Research**: ✅ **RESOLVED**

**Finding**:
- **Location**: `docusaurus_textbook/src/components/ChatbotWidget/` (directory-based component)
- **Structure**:
  - `index.tsx` - Main React component (functional, TSX)
  - `styles.module.css` - Scoped styles

**Component Signature**:
```tsx
export default function ChatbotWidget(): JSX.Element
```

**Current Implementation**:
- **Props**: NONE - It's a pure singleton component with no configuration interface
- **State Management**:
  - `useState<Message[]>` for message history
  - `useState<boolean>` for open/closed state
  - `useState<string>` for user input
  - `useState<boolean>` for loading state
  - `useRef<HTMLDivElement>` for scroll container
  - `useEffect` for auto-scroll behavior
- **Message Structure**:
  ```tsx
  interface Message {
    id: string;
    type: "user" | "bot";
    content: string;
    timestamp: Date;
  }
  ```

**Global Integration**:
- Loaded via `src/theme/Root.js` (Docusaurus theme override)
- Automatically rendered on all pages as a sibling to page content
- Floating chat button (60px circle) at bottom-right, z-index: 10000

**Styling**:
- Responsive: 420x650px (desktop), adapts to tablets/mobile (85vw, min 280px)
- Dark/light theme support via `[data-theme='dark']` selector
- Animations: slideUp (0.3s), fadeIn (0.3s), typing indicator

**Current Limitations**:
- Mock responses only (hardcoded `SAMPLE_RESPONSES` array)
- No backend integration
- No source attribution
- No message persistence
- No markdown rendering
- No props for customization

**Integration Strategy**:
✅ **Extend existing component** - Modify `index.tsx` to:
1. Add props interface to accept configuration:
   ```tsx
   interface ChatbotWidgetProps {
     backendUrl?: string;
     sessionId?: string;
   }
   ```
2. Add custom hooks for API calls:
   - `useChat()` - Handle API requests to `/api/sessions/{id}/chat`
   - `useSession()` - Manage session ID creation/persistence
   - `useSelectedText()` - Handle right-click context menu
3. Replace mock response logic with real API calls
4. Add error boundary for graceful error handling
5. Keep existing UI/animations intact (non-breaking change)

**Decision**: **EXTEND, DO NOT RECREATE**
- Preserves existing UI polish and animations
- Maintains Docusaurus theme integration
- Non-breaking change (additive only)
- Can still use `Root.js` for global loading

---

## Unknown 2: Frontend-Backend CORS Configuration

**Question**: Is CORS already configured in FastAPI backend? What origins need to be allowed?

**Research**: ✅ **RESOLVED**

**Finding**:
- **Current Status**: ❌ **NO CORS middleware configured** in `backend/api/main.py`
- **Locations Checked**:
  - `backend/api/main.py` - No CORSMiddleware import or setup
  - `backend/api/routes/*` - No CORS decorators
  - Grep search for "CORS", "CORSMiddleware", "allow_origins" - No results

**Required Origins**:

| Environment | Origin | Protocol |
|------------|--------|----------|
| Local Dev | `http://localhost:3000` | HTTP |
| Production | `https://usmankhan0016.github.io` | HTTPS |
| GitHub Pages | `https://usmankhan0016.github.io/ai_native-textbook/` | HTTPS |

**Frontend Config**:
- Docusaurus dev command: `docusaurus start` (runs on localhost:3000)
- Production URL: https://usmankhan0016.github.io/ai_native-textbook/
- Base URL: /ai_native-textbook/ (deployed to GitHub Pages)

**Implementation Required**:

Add to `backend/api/main.py` (after FastAPI initialization):

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",              # Local dev
        "http://localhost:8000",               # Local backend testing
        "https://usmankhan0016.github.io",    # Production domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Decision**: **ADD CORS MIDDLEWARE**
- Required for cross-origin requests from Docusaurus → FastAPI
- Must support both local dev and production origins
- Allow credentials for session cookies (if needed later)

---

## Unknown 3: Session ID Persistence Strategy

**Question**: localStorage vs URL params - which approach is better for Docusaurus?

**Research**: ✅ **RESOLVED**

**Analysis**:

| Strategy | Pros | Cons | Best For |
|----------|------|------|----------|
| **localStorage** | Persists across tabs, works offline, survives hard refresh, simple API | Some browsers have limits (~5-10MB), cleared on cache clear, privacy mode issues | Primary storage, typical use case |
| **URL Params** | Shareable sessions, no privacy concerns, explicit state | Visible in browser history, refresh loses params if not saved, long URLs | Session sharing, debugging, "continue conversation" links |
| **Cookies** | Server-side validation possible, automatic in requests, httpOnly security | Requires CORS credentials=true, potential security complexity, less control | Backend sessions (not needed here) |

**Spec Requirement**:
> Session ID persists across page refreshes and browser closures (localStorage / URL parameter)

**Recommended Strategy**: **HYBRID APPROACH**

1. **Primary**: localStorage for session ID
   - Persists across page refreshes and browser closures
   - Works seamlessly with static Docusaurus deployment
   - No visible URL clutter
   - Simple implementation with `localStorage.getItem('sessionId')`

2. **Secondary**: Optional URL parameter for:
   - Sharing sessions (e.g., "continue this conversation" link)
   - Debugging and testing
   - Session restoration if localStorage fails

**Implementation**:

```typescript
// useSession.ts
export const useSession = () => {
  const [sessionId, setSessionId] = useState<string>(() => {
    // 1. Check URL params first
    const params = new URLSearchParams(window.location.search);
    const urlSessionId = params.get('session_id');
    if (urlSessionId) {
      localStorage.setItem('sessionId', urlSessionId);
      return urlSessionId;
    }

    // 2. Check localStorage
    const storedSessionId = localStorage.getItem('sessionId');
    if (storedSessionId) {
      return storedSessionId;
    }

    // 3. Create new session
    const newSessionId = generateUUID();
    localStorage.setItem('sessionId', newSessionId);
    return newSessionId;
  });

  return sessionId;
};
```

**Decision**: **HYBRID (localStorage + optional URL params)**
- localStorage is primary (persistent, non-intrusive)
- URL params as secondary (shareable, debugging)
- Graceful fallback to new session if both missing

---

## Unknown 4: Right-Click Context Menu Implementation

**Question**: Standard browser API vs library like `react-contextmenu`?

**Research**: ✅ **RESOLVED**

**Analysis**:

| Approach | Library | Pros | Cons | Effort |
|----------|---------|------|------|--------|
| **Native API** | None (mousedown/mouseup events) | No dependencies, lightweight, full control | More code, styling from scratch, accessibility work | Medium |
| **react-contextmenu** | Community library (~3.5k downloads/week) | Drop-in component, good styling, accessibility built-in | Extra dependency, bundle size (+40KB), learning curve | Low |
| **Headless UI/Radix** | @radix-ui/dropdown-menu | Headless, fully accessible, composable | Requires more setup, unstyled base | Medium |
| **HTML5 contextmenu** | Native `<menu>` element | Standard API, no JS required | Limited browser support (deprecated), poor UX | Low-Medium |

**Spec Requirement**:
> Context menu appears with "Ask about this text" option

**Context**: Students right-click on selected text → menu appears → click option → chatbot opens with text pre-filled

**Recommended**: **NATIVE API + Custom Component**

**Why**:
1. No extra dependencies (keep bundle size minimal for Docusaurus)
2. Full control over styling (matches existing ChatbotWidget theme)
3. React pattern (hooks-based) fits existing component
4. Accessibility can be added incrementally

**Implementation Pattern**:

```typescript
// useContextMenu.ts
export const useContextMenu = () => {
  const [contextMenu, setContextMenu] = useState<{
    x: number;
    y: number;
    selectedText: string;
    visible: boolean;
  }>({ x: 0, y: 0, selectedText: '', visible: false });

  const handleContextMenu = (e: React.MouseEvent) => {
    e.preventDefault();
    const selectedText = window.getSelection()?.toString() || '';

    if (selectedText.trim()) {
      setContextMenu({
        x: e.clientX,
        y: e.clientY,
        selectedText,
        visible: true,
      });
    }
  };

  const handleClick = (action: 'ask') => {
    if (action === 'ask' && contextMenu.selectedText) {
      // Trigger chat with selected text
      onAskAboutText(contextMenu.selectedText);
    }
    setContextMenu(prev => ({ ...prev, visible: false }));
  };

  return { contextMenu, handleContextMenu, handleClick };
};

// ContextMenu.tsx (custom component)
<div
  style={{
    position: 'fixed',
    top: contextMenu.y,
    left: contextMenu.x,
    zIndex: 10001,
  }}
  className="custom-context-menu"
>
  <button onClick={() => handleClick('ask')}>
    ✨ Ask about this text
  </button>
</div>
```

**Decision**: **NATIVE API + Custom React Component**
- No dependencies
- Lightweight (~150 lines of code)
- Matches existing ChatbotWidget styling
- Easy to extend later

---

## Unknown 5: Error Handling & User Feedback Patterns

**Question**: What UI patterns does Docusaurus use for notifications and error messages?

**Research**: ✅ **RESOLVED**

**Finding**:
- **Current ChatbotWidget**: Uses inline messages in chat bubble interface
- **Docusaurus defaults**: Minimal error handling (relies on Algolia for errors)
- **No toast/notification library** in docusaurus.config.ts or package.json
- **No error boundary** in existing components

**Recommended Patterns** (based on ChatbotWidget existing style):

### 1. **Loading State** ✅ (Already implemented)
```tsx
// Show "thinking..." indicator during API calls
{isLoading && <div className="typing-indicator">●●●</div>}
```

### 2. **Error Messages in Chat** ✅ (New)
Display errors as bot messages with error styling:
```tsx
// Error message as chat bubble
{
  id: Date.now().toString(),
  type: "bot",
  content: "❌ Error: Could not connect to the service. Please try again.",
  timestamp: new Date(),
}
```

### 3. **API Error Mapping** ✅ (New)
Per spec (F-027 to F-032):

| HTTP Code | User Message | Recovery |
|-----------|--------------|----------|
| 400 | "Your message is too long. Try asking in <50,000 characters." | Clear input, retry |
| 404 | "Your session expired. Click here to start a new conversation." | Create new session, clear history |
| 429 | "Service is busy. Please try again in 1 minute." | Disable send for 60s, show countdown |
| 500 | "Something went wrong. Our team has been notified. Please try again." | Show retry button, log error |
| Network | "No internet connection. Please check your network." | Auto-retry on reconnect |
| Timeout (>15s) | "Request is taking longer than expected. You can cancel or wait." | Offer cancel button |

### 4. **Inline Error Boundary** ✅ (New)
Wrap ChatbotWidget in error boundary to prevent full page crash:
```tsx
// ErrorBoundary.tsx
class ChatErrorBoundary extends React.Component {
  state = { hasError: false };

  static getDerivedStateFromError(error) {
    return { hasError: true };
  }

  render() {
    if (this.state.hasError) {
      return <div>Chat service temporarily unavailable. Please refresh.</div>;
    }
    return this.props.children;
  }
}
```

### 5. **Disabled State During Errors**
- Disable send button when network is down
- Disable input when session invalid (show recovery button)
- Show "Retry" button for transient errors

### 6. **Success Feedback**
- Green indicator or "✓" next to message when successfully sent
- "Saved to database" quiet feedback (optional)

**Decision**: **CHAT-BASED ERROR MESSAGES** (Inline in ChatbotWidget)
- Matches existing UI paradigm (errors as bot messages)
- No extra notification library required
- User stays focused in chat context
- Error message can include retry buttons or links
- Follows chatbot UX conventions (Intercom, Drift, etc.)

**Implementation**: Add error messages to `messages` state array like regular bot messages, with error styling via CSS class.

---

## Summary: Phase 0 Unknowns Resolution

| Unknown | Decision | Effort | Dependencies |
|---------|----------|--------|--------------|
| 1. ChatbotWidget API | Extend component with custom hooks | Medium | None (use existing React) |
| 2. CORS Configuration | Add CORSMiddleware to FastAPI main.py | Low | None (built into FastAPI) |
| 3. Session Persistence | Hybrid (localStorage primary, URL params secondary) | Low | None (native browser API) |
| 4. Context Menu | Native API + custom React component | Medium | None (custom code) |
| 5. Error Handling | Chat-based error messages (inline in component) | Low | None (use existing message structure) |

**Total New Dependencies**: 0 (None! All standard libraries or native APIs)

**Total Effort Estimate**: Medium (6-8 medium tasks)

**Risk Level**: Low (well-understood technologies, proven patterns)

---

## Next Steps: Phase 1 Design

After Phase 0 research complete:

1. ✅ **data-model.md** - Define Session, Message, SelectedTextMetadata entities with relationships
2. ✅ **contracts/** - Document API contract specs (OpenAPI format)
3. ✅ **quickstart.md** - Local dev setup guide
4. ✅ **Update agent context** - Add tech stack info for task generation

Then: **Run `/sp.tasks`** to generate implementation task breakdown

---

**Approved By**: [User approval pending]
**Status**: Ready for Phase 1 design and /sp.tasks
