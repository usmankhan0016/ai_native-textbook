/**
 * useChat Hook
 *
 * Handles all chat API interactions with the RAG backend
 */

import { useState, useCallback } from 'react';
import { api, HttpError } from './apiClient';
import { getUserErrorMessage } from './errorHandling';

interface Source {
  chapter: string;
  relevance_score: number;
  text_preview: string;
  section?: string;
  url?: string;
}

interface MessageResponse {
  id: string;
  session_id: string;
  role: 'user' | 'assistant';
  content: string;
  created_at: string;
  mode: 'whole_book' | 'selected_text';
  metadata?: Record<string, any>;
}

interface BackendChatResponse {
  message: MessageResponse;
  sources: Source[] | null;
}

interface ChatResponse {
  answer: string;
  sources: Source[] | null;
  metadata?: Record<string, any>;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  mode: 'whole_book' | 'selected_text';
  created_at: string;
  metadata?: Record<string, any>;
}

interface HistoryResponse {
  messages: Message[];
}

export function useChat(sessionId: string | null) {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Query the whole book (with Qdrant retrieval)
   */
  const queryWholeBook = useCallback(
    async (query: string): Promise<ChatResponse | null> => {
      if (!sessionId) {
        setError('No active session');
        return null;
      }

      setIsLoading(true);
      setError(null);

      try {
        const backendResponse = await api.post<BackendChatResponse>(
          `/api/sessions/${sessionId}/chat`,
          { query }
        );

        console.log('[useChat] Backend response:', backendResponse);

        setIsLoading(false);

        // Transform backend response to frontend format
        const chatResponse: ChatResponse = {
          answer: backendResponse.message.content,
          sources: backendResponse.sources,
          metadata: backendResponse.message.metadata
        };

        console.log('[useChat] Transformed response:', chatResponse);

        return chatResponse;
      } catch (err) {
        setIsLoading(false);

        if (err instanceof HttpError) {
          const errorMsg = getUserErrorMessage(err.statusCode, err.detail);
          setError(errorMsg);
        } else {
          setError('An unexpected error occurred');
        }

        return null;
      }
    },
    [sessionId]
  );

  /**
   * Query with selected text only (no retrieval)
   */
  const querySelectedText = useCallback(
    async (
      query: string,
      selectedText: string,
      chapterOrigin?: string
    ): Promise<ChatResponse | null> => {
      console.log('[useChat] querySelectedText called');
      console.log('[useChat] Session ID:', sessionId);
      console.log('[useChat] Query:', query);
      console.log('[useChat] Selected text (first 100 chars):', selectedText.substring(0, 100));
      console.log('[useChat] Selected text length:', selectedText.length);

      if (!sessionId) {
        setError('No active session');
        return null;
      }

      setIsLoading(true);
      setError(null);

      try {
        const requestPayload = {
          query,
          selected_text: selectedText,
          chapter_origin: chapterOrigin,
        };

        console.log('[useChat] Sending request to backend:', requestPayload);

        const backendResponse = await api.post<BackendChatResponse>(
          `/api/sessions/${sessionId}/selected-text-chat`,
          requestPayload
        );

        console.log('[useChat] Backend response:', backendResponse);

        setIsLoading(false);

        // Transform backend response to frontend format
        const chatResponse: ChatResponse = {
          answer: backendResponse.message.content,
          sources: backendResponse.sources,
          metadata: backendResponse.message.metadata
        };

        console.log('[useChat] Transformed response:', chatResponse);

        return chatResponse;
      } catch (err) {
        setIsLoading(false);

        console.error('[useChat] Error in querySelectedText:', err);

        if (err instanceof HttpError) {
          const errorMsg = getUserErrorMessage(err.statusCode, err.detail);
          setError(errorMsg);
        } else {
          setError('An unexpected error occurred');
        }

        return null;
      }
    },
    [sessionId]
  );

  /**
   * Get chat history for the current session
   */
  const getHistory = useCallback(async (): Promise<Message[]> => {
    if (!sessionId) {
      setError('No active session');
      return [];
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await api.get<HistoryResponse>(
        `/api/sessions/${sessionId}/history`
      );

      setIsLoading(false);
      return response.messages || [];
    } catch (err) {
      setIsLoading(false);

      if (err instanceof HttpError) {
        // 404 means session not found or no history - not really an error
        if (err.statusCode === 404) {
          return [];
        }

        const errorMsg = getUserErrorMessage(err.statusCode, err.detail);
        setError(errorMsg);
      } else {
        setError('Failed to load chat history');
      }

      return [];
    }
  }, [sessionId]);

  /**
   * Clear error state
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    isLoading,
    error,
    queryWholeBook,
    querySelectedText,
    getHistory,
    clearError,
  };
}
