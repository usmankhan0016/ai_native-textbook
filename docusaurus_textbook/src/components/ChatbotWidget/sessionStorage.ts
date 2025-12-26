/**
 * Session Storage Utilities
 *
 * Handles session ID persistence across page refreshes using:
 * - localStorage (primary)
 * - URL parameters (secondary, for shareable sessions)
 */

const SESSION_ID_KEY = 'rag_chatbot_session_id';

/**
 * Generate a UUID v4
 */
function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Check if localStorage is available
 */
function isLocalStorageAvailable(): boolean {
  try {
    const test = '__localStorage_test__';
    localStorage.setItem(test, test);
    localStorage.removeItem(test);
    return true;
  } catch {
    return false;
  }
}

/**
 * Get session ID from URL parameters
 */
function getSessionIdFromURL(): string | null {
  if (typeof window === 'undefined') return null;

  const params = new URLSearchParams(window.location.search);
  return params.get('session_id');
}

/**
 * Get session ID from localStorage, URL params, or generate new one
 *
 * Priority:
 * 1. URL parameter (allows sharing sessions)
 * 2. localStorage (persists across refreshes)
 * 3. Generate new UUID
 */
export function getSessionId(): string {
  // Check URL params first (highest priority)
  const urlSessionId = getSessionIdFromURL();
  if (urlSessionId) {
    // Save to localStorage for future use
    saveSessionId(urlSessionId);
    return urlSessionId;
  }

  // Check localStorage
  if (isLocalStorageAvailable()) {
    const storedSessionId = localStorage.getItem(SESSION_ID_KEY);
    if (storedSessionId) {
      return storedSessionId;
    }
  }

  // Generate new session ID
  const newSessionId = generateUUID();
  saveSessionId(newSessionId);
  return newSessionId;
}

/**
 * Save session ID to localStorage
 */
export function saveSessionId(sessionId: string): void {
  if (!isLocalStorageAvailable()) {
    console.warn('localStorage not available, session will not persist');
    return;
  }

  try {
    localStorage.setItem(SESSION_ID_KEY, sessionId);
  } catch (error) {
    console.error('Failed to save session ID to localStorage:', error);
  }
}

/**
 * Clear session from localStorage
 */
export function clearSession(): void {
  if (!isLocalStorageAvailable()) return;

  try {
    localStorage.removeItem(SESSION_ID_KEY);
  } catch (error) {
    console.error('Failed to clear session from localStorage:', error);
  }
}

/**
 * Check if a session exists in localStorage
 */
export function hasStoredSession(): boolean {
  if (!isLocalStorageAvailable()) return false;

  return localStorage.getItem(SESSION_ID_KEY) !== null;
}
