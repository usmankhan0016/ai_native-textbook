/**
 * useSession Hook
 *
 * Manages session ID persistence and initialization
 */

import { useState, useEffect } from 'react';
import { getSessionId, saveSessionId } from './sessionStorage';
import { api } from './apiClient';

interface SessionData {
  id: string;
  created_at: string;
  updated_at: string;
  metadata: Record<string, any>;
}

export function useSession() {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [isSessionReady, setIsSessionReady] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    async function initializeSession() {
      try {
        // Get session ID from storage or generate new one
        const storedSessionId = getSessionId();

        // Verify session exists in backend (or create new one)
        try {
          // Try to fetch session from backend
          await api.get<SessionData>(`/api/sessions/${storedSessionId}/history`);

          // Session exists, use it
          setSessionId(storedSessionId);
          setIsSessionReady(true);
        } catch (err: any) {
          // Session doesn't exist (404) or other error
          // Create new session in backend
          const newSession = await api.post<SessionData>('/api/sessions', {});

          // Save new session ID
          saveSessionId(newSession.id);
          setSessionId(newSession.id);
          setIsSessionReady(true);
        }
      } catch (err: any) {
        console.error('Failed to initialize session:', err);
        setError(err.message || 'Failed to initialize session');
        setIsSessionReady(false);
      }
    }

    initializeSession();
  }, []);

  return {
    sessionId,
    isSessionReady,
    error,
  };
}
