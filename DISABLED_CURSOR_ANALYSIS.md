# Disabled Cursor Issue - Root Cause Analysis

## Problem
Users see a `disabled` cursor (⊗ not-allowed) when hovering over the chat input field, even when the chatbot is idle.

## Root Cause

The textarea is disabled by this condition in `index.tsx:756`:

```javascript
disabled={isLoading || !isSessionReady}
```

The input becomes disabled when **either**:
1. **`isLoading` = true** → While waiting for backend response (this is correct behavior)
2. **`!isSessionReady` = true** → While session is initializing **OR when session initialization fails** (THIS IS THE ISSUE)

### Session Initialization Flow

1. **`useSession.ts:24-55`** runs on component mount
2. It attempts to:
   - Check if there's a stored session ID
   - Verify that session exists on backend by calling `GET /api/sessions/{sessionId}/history`
   - If that fails (404 or other error), it creates a new session with `POST /api/sessions`
   - Sets `isSessionReady = true` only after success

3. **The problem:** If `POST /api/sessions` fails (e.g., backend connection error), the exception is caught but `isSessionReady` remains `false`, keeping the input permanently disabled.

### Why This Happens

From `useSession.ts:47-50`:
```javascript
catch (err: any) {
  console.error('Failed to initialize session:', err);
  setError(err.message || 'Failed to initialize session');
  setIsSessionReady(false);  // ← Input stays disabled indefinitely
}
```

When the backend is unreachable or returns an error, the session initialization fails silently and the input never becomes enabled.

## Evidence from Code

### Frontend (`ChatbotWidget/index.tsx`)
- **Line 756**: `disabled={isLoading || !isSessionReady}`
- `isSessionReady` is set by `useSession` hook (line 69)

### Session Initialization (`useSession.ts`)
- **Line 20**: `const [isSessionReady, setIsSessionReady] = useState(false);`
- **Line 45**: `setIsSessionReady(true);` only set on success
- **Line 50**: `setIsSessionReady(false);` set on error, but never reset

### Backend (`CORS configuration`)
- Backend is deployed on HF Space: `https://Usmankhan0016-textbook-rag-backend.hf.space`
- Frontend is deployed on GitHub Pages: `https://usmankhan0016.github.io/ai_native-textbook`
- CORS is configured in `main.py:96-107` to allow the GitHub Pages origin

## Why the Input Stays Disabled

### Scenario 1: Session Creation Fails
1. Component mounts
2. `useSession` tries to get/create session
3. Backend connection fails (e.g., 500 error from Qdrant being empty, network timeout, CORS issue, backend down)
4. Error caught, `isSessionReady` stays `false`
5. Input remains disabled with "not-allowed" cursor
6. **No retry mechanism** - user must refresh page

### Scenario 2: Backend Returns 500 Error
The backend chat endpoint (`POST /api/sessions/{sessionId}/chat`) returns 500 errors because Qdrant has no data:
```
{
  "error": {
    "status_code": 500,
    "message": "Failed to process query",
    ...
  }
}
```

While this doesn't disable the input (session was created successfully), it explains why queries fail.

## Solutions

### Short-term (No code changes needed)
1. **Run the ingestion pipeline** to populate Qdrant with textbook content
   - This will allow chat queries to succeed
   - Reduces 500 errors

2. **Verify backend is running** on HF Space
   - Check: `curl https://Usmankhan0016-textbook-rag-backend.hf.space/health`
   - Should return: `{"status":"healthy","service":"rag-agent-backend"}`

3. **Check browser console** for session initialization errors
   - Open DevTools (F12)
   - Look for errors in Console tab
   - Check Network tab for failed `/api/sessions` requests

### Medium-term (Recommended code improvements)

#### 1. Add Retry Mechanism to Session Initialization
After session initialization fails, users should be able to retry without refreshing.

**File**: `ChatbotWidget/useSession.ts`

Add retry state and button:
```javascript
const [initError, setInitError] = useState<string | null>(null);
const [isRetrying, setIsRetrying] = useState(false);

const retryInitialization = useCallback(async () => {
  setIsRetrying(true);
  setInitError(null);
  try {
    // Retry logic here
  } catch (err) {
    setInitError(err.message);
  } finally {
    setIsRetrying(false);
  }
}, []);

return {
  sessionId,
  isSessionReady,
  error: initError,
  isRetrying,
  retryInitialization,
};
```

#### 2. Allow Input Even When Session Not Ready
Keep input enabled but show disabled button, letting users know the issue:

**File**: `ChatbotWidget/index.tsx`

Change line 756 from:
```javascript
disabled={isLoading || !isSessionReady}
```

To:
```javascript
disabled={isLoading}
```

And update button (line 761):
```javascript
disabled={isLoading || !isSessionReady || !inputValue.trim()}
```

Add error indicator above input:
```javascript
{!isSessionReady && (
  <div className={styles.initializationError}>
    ⚠️ Initializing session... {error && `(${error})`}
  </div>
)}
```

#### 3. Add Session Status Display
Show user what's happening:

```javascript
<p className={styles.chatSubtitle}>
  {!isSessionReady && error ? (
    <span style={{ color: '#ef4444' }}>
      ❌ Connection error - {error}
    </span>
  ) : !isSessionReady ? (
    <span style={{ color: '#f59e0b' }}>
      ⏳ Initializing...
    </span>
  ) : (
    'Powered by RAG & LLM'
  )}
</p>
```

#### 4. Add Exponential Backoff for Retries
In `useSession.ts`, add retry logic with exponential backoff:

```javascript
const MAX_RETRIES = 3;
const [retries, setRetries] = useState(0);

useEffect(() => {
  let timeout;

  const retry = async () => {
    if (retries < MAX_RETRIES) {
      const delay = Math.pow(2, retries) * 1000; // 1s, 2s, 4s
      timeout = setTimeout(initializeSession, delay);
    }
  };

  if (!isSessionReady && error && retries < MAX_RETRIES) {
    retry();
  }

  return () => clearTimeout(timeout);
}, [isSessionReady, error, retries]);
```

## Current Impact

- **User Experience**: Input shows "disabled" cursor until backend is fully operational
- **First-time users**: Confusing UX - they can't tell if app is broken or loading
- **Workaround**: Refresh page, wait for Qdrant ingestion to complete, restart backend
- **Severity**: Medium - affects usability but not critical for core functionality

## Testing

To verify the fix:
1. Trigger a session initialization error (e.g., stop backend, open chatbot)
2. Input should show error message
3. User should be able to see input or retry button
4. Once backend recovers, retry works without page refresh
