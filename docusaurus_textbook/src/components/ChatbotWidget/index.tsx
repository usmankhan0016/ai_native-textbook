import React, { useState, useRef, useEffect, useCallback } from "react";
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from "./styles.module.css";
import { useSession } from "./useSession";
import { useChat } from "./useChat";
import SourcesList from "./SourcesList";
import SelectionContextMenu from "./SelectionContextMenu";
import ErrorMessage from "./ErrorMessage";
import ErrorBoundary from "./ErrorBoundary";
import { getUserErrorMessage, shouldRetry, getRetryDelay } from "./errorHandling";

interface Source {
  chapter: string;
  relevance_score: number;
  text_preview: string;
  section?: string;
  url?: string;
}

interface Message {
  id: string;
  type: "user" | "bot";
  content: string;
  timestamp: Date;
  sources?: Source[];
  mode?: "whole_book" | "selected_text";
  selectedText?: string;
}

export default function ChatbotWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: "welcome",
      type: "bot",
      content:
        "Hello! 👋 I'm your AI-Native Robotics Textbook Assistant. Ask me anything about the course content, or select text on the page and click 'Ask about this' to ask specifically about that text!",
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState("");
  const [selectedText, setSelectedText] = useState<string>("");
  const [capturedTextForQuery, setCapturedTextForQuery] = useState<string>(""); // Captured text that won't be cleared by selection events
  const [selectionMenuPosition, setSelectionMenuPosition] = useState<{ x: number; y: number } | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const selectionTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const selectedTextRef = useRef<string>(""); // Backup ref to prevent losing text
  const isManualClearRef = useRef<boolean>(false); // Flag to prevent auto-clear when we manually clear selection
  const lastSelectionCheckRef = useRef<number>(0); // For throttling selectionchange

  // Error handling state
  const [currentError, setCurrentError] = useState<{
    message: string;
    statusCode?: number;
    timestamp: number;
  } | null>(null);
  const [showTimeoutWarning, setShowTimeoutWarning] = useState(false);
  const [retryCountdown, setRetryCountdown] = useState<number>(0);
  const requestStartTime = useRef<number>(0);
  const timeoutWarningTimer = useRef<NodeJS.Timeout | null>(null);
  const retryTimer = useRef<NodeJS.Timeout | null>(null);
  const abortController = useRef<AbortController | null>(null);

  // Get logo path using Docusaurus baseUrl
  const logoUrl = useBaseUrl('/img/logo.svg');

  // Initialize session and chat hooks
  const { sessionId, isSessionReady, error: sessionError } = useSession();
  const { isLoading, error: chatError, queryWholeBook, querySelectedText, getHistory, clearError } = useChat(sessionId);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Load chat history when session is ready
  useEffect(() => {
    async function loadHistory() {
      if (!sessionId || !isSessionReady) return;

      try {
        const history = await getHistory();

        if (history && history.length > 0) {
          console.log('[ChatbotWidget] Loaded history:', history.length, 'messages');

          // Transform history to Message format
          const historicalMessages: Message[] = history.map((msg) => ({
            id: msg.id,
            type: msg.role === 'user' ? 'user' : 'bot',
            content: msg.content,
            timestamp: new Date(msg.created_at),
            sources: undefined, // Sources not stored in history currently
          }));

          // Replace welcome message with actual history
          setMessages(historicalMessages);
        } else {
          // No history - keep welcome message
          console.log('[ChatbotWidget] No history found, showing welcome message');
        }
      } catch (error) {
        console.error('[ChatbotWidget] Failed to load history:', error);
        // Keep welcome message on error
      }
    }

    loadHistory();
  }, [sessionId, isSessionReady, getHistory]);

  // Cleanup timers on unmount
  useEffect(() => {
    return () => {
      if (timeoutWarningTimer.current) {
        clearTimeout(timeoutWarningTimer.current);
      }
      if (retryTimer.current) {
        clearTimeout(retryTimer.current);
      }
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
      }
    };
  }, []);

  // Handle text selection on page - IMMEDIATE, NO DEBOUNCE
  const handleTextSelection = useCallback((isMouseUpEvent: boolean = false) => {
    // For mouseup/touchend: process IMMEDIATELY (no debounce)
    // For selectionchange: throttle to max once per 100ms
    const now = Date.now();
    if (!isMouseUpEvent && (now - lastSelectionCheckRef.current < 100)) {
      return; // Throttle selectionchange events
    }
    lastSelectionCheckRef.current = now;

    console.log('[ChatbotWidget] 🔍 Processing selection (mouseup:', isMouseUpEvent, ')');

    try {
      const selection = window.getSelection();

      if (!selection) {
        console.log('[ChatbotWidget] ❌ No selection API');
        return;
      }

      // Check if selection exists and has content
      if (!selection.rangeCount || selection.isCollapsed) {
        console.log('[ChatbotWidget] No active selection or collapsed');

        // CRITICAL: Only clear state if this is NOT a manual clear
        if (isManualClearRef.current) {
          console.log('[ChatbotWidget] ⚠️ Manual clear - preserving text');
          isManualClearRef.current = false;
          setSelectionMenuPosition(null);
          return;
        }

        // Normal auto-clear
        setSelectionMenuPosition(null);
        setSelectedText("");
        selectedTextRef.current = "";
        return;
      }

      const text = selection.toString().trim();
      console.log('[ChatbotWidget] 📝 Selected:', text.substring(0, 50) + (text.length > 50 ? '...' : ''));
      console.log('[ChatbotWidget] Length:', text.length, 'chars');

      // Minimum 2 characters (very lenient for acronyms like "AI", "ML")
      if (text.length < 2) {
        console.log('[ChatbotWidget] Too short (<2 chars)');
        return;
      }

      // Store IMMEDIATELY in both state and ref
      setSelectedText(text);
      selectedTextRef.current = text;
      console.log('[ChatbotWidget] ✅ Stored');

      // Calculate position
      try {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        console.log('[ChatbotWidget] Rect:', rect.width, 'x', rect.height, 'at', rect.top, ',', rect.left);

        let position;

        // Only fallback if completely invalid
        if (rect.width === 0 && rect.height === 0 && rect.top === 0 && rect.left === 0) {
          console.warn('[ChatbotWidget] ⚠️ Invalid rect - center fallback');
          position = {
            x: window.innerWidth / 2,
            y: window.scrollY + 200,
          };
        } else {
          // Use rect - constrain to viewport
          const centerX = rect.left + (rect.width / 2);
          const topY = rect.top + window.scrollY - 15;

          position = {
            x: Math.max(100, Math.min(centerX, window.innerWidth - 100)),
            y: Math.max(60, topY),
          };

          console.log('[ChatbotWidget] ✅ Position:', position.x.toFixed(0), ',', position.y.toFixed(0));
        }

        setSelectionMenuPosition(position);

      } catch (posError) {
        console.error('[ChatbotWidget] ❌ Position error:', posError);
        // Still show button even if position fails
        setSelectionMenuPosition({
          x: window.innerWidth / 2,
          y: window.scrollY + 200,
        });
      }

    } catch (error) {
      console.error('[ChatbotWidget] ❌ Critical error:', error);
      // Don't clear on error - keep button visible
    }
  }, []);

  // Add selection event listeners
  useEffect(() => {
    console.log('[ChatbotWidget] ✅ Attaching selection event listeners');

    // Handlers that mark event type
    const handleMouseUp = () => handleTextSelection(true);
    const handleTouchEnd = () => handleTextSelection(true);
    const handleSelectionChange = () => handleTextSelection(false);

    // selectionchange fires WHILE user is selecting (throttled to 100ms)
    document.addEventListener('selectionchange', handleSelectionChange);

    // mouseup/touchend fire AFTER user releases (immediate, no throttle)
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleTouchEnd);

    return () => {
      console.log('[ChatbotWidget] ❌ Removing selection event listeners');
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleTouchEnd);
    };
  }, [handleTextSelection]);

  // Handle "Ask about this" button click
  const handleAskAboutSelection = async () => {
    console.log('[ChatbotWidget] ========================');
    console.log('[ChatbotWidget] 📝 ASK ABOUT SELECTION CLICKED');
    console.log('[ChatbotWidget] ========================');

    // CRITICAL: Capture text IMMEDIATELY before any events can clear it
    const textToCapture = selectedText || selectedTextRef.current;

    console.log('[ChatbotWidget] Selected text state:', selectedText?.substring(0, 100));
    console.log('[ChatbotWidget] Selected text ref:', selectedTextRef.current?.substring(0, 100));
    console.log('[ChatbotWidget] Capturing:', textToCapture?.substring(0, 100));

    if (!textToCapture || !isSessionReady) {
      console.log('[ChatbotWidget] ❌ Cannot proceed - missing text or session not ready');
      return;
    }

    // IMMEDIATELY capture text into protected state (won't be cleared by selection events)
    setCapturedTextForQuery(textToCapture);
    console.log('[ChatbotWidget] ✅ TEXT CAPTURED INTO PROTECTED STATE');
    console.log('[ChatbotWidget] Length:', textToCapture.length, 'chars');
    console.log('[ChatbotWidget] Content:', textToCapture.substring(0, 200));

    // Open chatbot if not already open
    const wasClosedBefore = !isOpen;
    if (!isOpen) {
      console.log('[ChatbotWidget] Opening chatbot');
      setIsOpen(true);
    }

    // Hide the selection menu button
    setSelectionMenuPosition(null);

    // Set flag to prevent selection handler from clearing during cleanup
    isManualClearRef.current = true;

    // Clear browser selection visually (this will trigger selection events)
    window.getSelection()?.removeAllRanges();

    // Clear the selectedText state (but capturedTextForQuery is safe)
    setSelectedText("");
    selectedTextRef.current = "";

    console.log('[ChatbotWidget] ========================');
    console.log('[ChatbotWidget] Ready for user question');
    console.log('[ChatbotWidget] Captured text is protected and ready');
    console.log('[ChatbotWidget] ========================');

    // Focus the input field
    if (wasClosedBefore) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    } else {
      inputRef.current?.focus();
    }
  };

  // Close selection menu
  const handleCloseSelectionMenu = () => {
    console.log('[ChatbotWidget] Closing selection menu and clearing selected text');

    // Clear any pending selection timeout
    if (selectionTimeoutRef.current) {
      clearTimeout(selectionTimeoutRef.current);
    }

    // Clear everything immediately (don't wait for handler)
    setSelectionMenuPosition(null);
    setSelectedText("");
    selectedTextRef.current = "";
    isManualClearRef.current = false; // Ensure flag is reset

    // Clear browser selection
    window.getSelection()?.removeAllRanges();
  };

  const handleSendMessage = async () => {
    console.log('[ChatbotWidget] 📤 SEND MESSAGE CALLED');

    if (!inputValue.trim() || !isSessionReady) {
      console.log('[ChatbotWidget] ❌ Cannot send - empty input or session not ready');
      return;
    }

    // Clear any previous errors and warnings
    setCurrentError(null);
    setShowTimeoutWarning(false);
    clearError();

    // Clear timeout timers
    if (timeoutWarningTimer.current) {
      clearTimeout(timeoutWarningTimer.current);
    }

    const userQuery = inputValue;
    const hasSelectedText = capturedTextForQuery && capturedTextForQuery.length > 0;
    const mode = hasSelectedText ? 'selected_text' : 'whole_book';

    console.log('[ChatbotWidget] Mode:', mode.toUpperCase());

    // Add user message
    const userMessage: Message = {
      id: `user-${Date.now()}`,
      type: "user",
      content: userQuery,
      timestamp: new Date(),
      mode,
      selectedText: hasSelectedText ? capturedTextForQuery : undefined,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue("");

    // Start timeout warning timer (12s for whole-book, 4.5s for selected-text)
    const timeoutThreshold = mode === 'whole_book' ? 12000 : 4500;
    requestStartTime.current = Date.now();

    timeoutWarningTimer.current = setTimeout(() => {
      console.log('[ChatbotWidget] ⏱️ Request timeout warning triggered');
      setShowTimeoutWarning(true);
    }, timeoutThreshold);

    try {
      // Create abort controller for cancellation
      abortController.current = new AbortController();

      // Query the backend
      let response;
      if (hasSelectedText) {
        console.log('[ChatbotWidget] 🔍 SELECTED-TEXT MODE');
        response = await querySelectedText(userQuery, capturedTextForQuery);
        setCapturedTextForQuery("");
      } else {
        console.log('[ChatbotWidget] 🔍 WHOLE-BOOK MODE');
        response = await queryWholeBook(userQuery);
      }

      // Clear timeout warning if request completed
      if (timeoutWarningTimer.current) {
        clearTimeout(timeoutWarningTimer.current);
      }
      setShowTimeoutWarning(false);

      if (response) {
        const botMessage: Message = {
          id: `bot-${Date.now()}`,
          type: "bot",
          content: response.answer,
          timestamp: new Date(),
          sources: response.sources || undefined,
          mode,
        };
        setMessages((prev) => [...prev, botMessage]);
        console.log('[ChatbotWidget] ✅ Response received');
      } else if (chatError) {
        // Handle error from useChat hook
        throw new Error(chatError);
      }
    } catch (error: any) {
      // Clear timeout warning
      if (timeoutWarningTimer.current) {
        clearTimeout(timeoutWarningTimer.current);
      }
      setShowTimeoutWarning(false);

      console.error('[ChatbotWidget] ❌ Error:', error);

      // Determine error type and get user-friendly message
      const statusCode = error.statusCode || error.status || 0;
      const errorMessage = getUserErrorMessage(statusCode, error.message);

      // Set current error state
      setCurrentError({
        message: errorMessage,
        statusCode,
        timestamp: Date.now(),
      });

      // Add error message to chat
      const errorMsg: Message = {
        id: `error-${Date.now()}`,
        type: "bot",
        content: `❌ ${errorMessage}`,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMsg]);

      // If retryable error, start countdown
      if (shouldRetry(statusCode)) {
        const delay = getRetryDelay(statusCode);
        if (delay > 0) {
          setRetryCountdown(Math.floor(delay / 1000));
          const interval = setInterval(() => {
            setRetryCountdown((prev) => {
              if (prev <= 1) {
                clearInterval(interval);
                return 0;
              }
              return prev - 1;
            });
          }, 1000);
        }
      }
    } finally {
      // Cleanup
      abortController.current = null;
    }
  };

  const handleRetry = () => {
    console.log('[ChatbotWidget] 🔄 Retry button clicked');
    setCurrentError(null);
    setRetryCountdown(0);
    // Re-trigger send with last input (would need to save it)
    // For now, just clear the error and let user resend
  };

  const handleDismissError = () => {
    console.log('[ChatbotWidget] ✖️ Error dismissed');
    setCurrentError(null);
    setRetryCountdown(0);
  };

  const handleDismissTimeout = () => {
    console.log('[ChatbotWidget] ✖️ Timeout warning dismissed');
    setShowTimeoutWarning(false);
  };

  const handleCancelRequest = () => {
    console.log('[ChatbotWidget] 🛑 Request cancelled');

    // Cancel ongoing request if possible
    if (abortController.current) {
      abortController.current.abort();
    }

    // Clear timers
    if (timeoutWarningTimer.current) {
      clearTimeout(timeoutWarningTimer.current);
    }

    setShowTimeoutWarning(false);
    setCurrentError(null);

    // Add cancellation message
    const cancelMessage: Message = {
      id: `cancel-${Date.now()}`,
      type: "bot",
      content: "⏹️ Request cancelled. You can ask another question.",
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, cancelMessage]);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <ErrorBoundary>
      {/* Selection Context Menu */}
      <SelectionContextMenu
        selectedText={selectedText}
        position={selectionMenuPosition}
        onAskAboutSelection={handleAskAboutSelection}
        onClose={handleCloseSelectionMenu}
      />

      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
          title="Ask me anything about the textbook"
        >
          <span className={styles.chatIcon}>💬</span>
        </button>
      )}

      {/* Chat Widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <img
                src={logoUrl}
                alt="AI Robotics"
                className={styles.headerIcon}
              />
              <div>
                <h3 className={styles.chatTitle}>AI Robotics Assistant</h3>
                <p className={styles.chatSubtitle}>
                  {capturedTextForQuery && capturedTextForQuery.length > 0 ? (
                    <span style={{ color: '#8b5cf6', fontWeight: 600 }}>
                      📝 Selected text mode ({capturedTextForQuery.length} chars)
                    </span>
                  ) : (
                    'Powered by RAG & LLM'
                  )}
                </p>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chatbot"
            >
              ✕
            </button>
          </div>

          {/* Messages Container */}
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.messageGroup} ${
                  message.type === "user"
                    ? styles.userGroup
                    : styles.botGroup
                }`}
              >
                {/* Avatar */}
                <div className={styles.avatar}>
                  {message.type === "user" ? (
                    <span className={styles.userAvatar}>U</span>
                  ) : (
                    <span className={styles.botAvatar}>
                      <img
                        src={logoUrl}
                        alt="AI Assistant"
                        className={styles.botAvatarImg}
                      />
                    </span>
                  )}
                </div>

                {/* Message Content */}
                <div className={styles.messageContent}>
                  {/* Selected-text mode indicator */}
                  {message.mode === "selected_text" && (
                    <div className={styles.selectedTextBadge}>
                      📝 Selected Text Query
                    </div>
                  )}

                  {/* Show selected text for user messages in selected-text mode */}
                  {message.type === "user" && message.selectedText && (
                    <div style={{
                      background: '#f3f4f6',
                      padding: '0.5rem',
                      borderRadius: '6px',
                      fontSize: '0.85rem',
                      marginBottom: '0.5rem',
                      fontStyle: 'italic',
                      borderLeft: '3px solid #8b5cf6',
                      color: '#4b5563',
                    }}>
                      Selected: "{message.selectedText.substring(0, 100)}{message.selectedText.length > 100 ? '...' : ''}"
                    </div>
                  )}

                  <div className={styles.messageBubble}>
                    {message.content}
                  </div>
                  {message.type === "bot" && message.sources && (
                    <SourcesList sources={message.sources} />
                  )}
                  <span className={styles.timestamp}>
                    {message.timestamp.toLocaleTimeString([], {
                      hour: "2-digit",
                      minute: "2-digit",
                    })}
                  </span>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.messageGroup} ${styles.botGroup}`}>
                <div className={styles.avatar}>
                  <span className={styles.botAvatar}>
                    <img
                      src={logoUrl}
                      alt="AI Assistant"
                      className={styles.botAvatarImg}
                    />
                  </span>
                </div>
                <div className={styles.messageContent}>
                  <div className={styles.messageBubble}>
                    <div className={styles.typingIndicator}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {/* Timeout Warning */}
            {showTimeoutWarning && isLoading && (
              <div className={styles.messageContent} style={{ marginTop: '8px' }}>
                <ErrorMessage
                  type="timeout"
                  message="This request is taking longer than expected. You can continue waiting or cancel."
                  onCancel={handleCancelRequest}
                  onDismiss={handleDismissTimeout}
                />
              </div>
            )}

            {/* Error Message */}
            {currentError && !isLoading && (
              <div className={styles.messageContent} style={{ marginTop: '8px' }}>
                <ErrorMessage
                  type="error"
                  message={currentError.message}
                  onRetry={handleRetry}
                  onDismiss={handleDismissError}
                  showRetry={currentError.statusCode !== undefined && shouldRetry(currentError.statusCode)}
                  retryDisabled={retryCountdown > 0}
                  retryCountdown={retryCountdown}
                />
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Selected Text Indicator (above input) */}
          {capturedTextForQuery && capturedTextForQuery.length > 0 && (
            <div style={{
              background: 'linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%)',
              color: 'white',
              padding: '0.75rem 1rem',
              borderTop: '1px solid rgba(139, 92, 246, 0.3)',
              fontSize: '0.85rem',
              lineHeight: '1.4',
            }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.25rem' }}>
                <span style={{ fontWeight: 700, fontSize: '0.9rem' }}>📝 Selected Text Captured</span>
                <button
                  onClick={() => {
                    console.log('[ChatbotWidget] Clear button clicked - removing captured text');
                    setCapturedTextForQuery("");
                    setSelectedText("");
                    selectedTextRef.current = "";
                    setSelectionMenuPosition(null);
                    isManualClearRef.current = false;
                  }}
                  style={{
                    background: 'rgba(255, 255, 255, 0.2)',
                    border: 'none',
                    color: 'white',
                    padding: '0.25rem 0.5rem',
                    borderRadius: '4px',
                    cursor: 'pointer',
                    fontSize: '0.75rem',
                    fontWeight: 600,
                  }}
                  title="Clear selected text"
                >
                  Clear ✕
                </button>
              </div>
              <div style={{
                background: 'rgba(255, 255, 255, 0.15)',
                padding: '0.5rem',
                borderRadius: '4px',
                fontStyle: 'italic',
                maxHeight: '60px',
                overflow: 'auto',
              }}>
                "{capturedTextForQuery.substring(0, 200)}{capturedTextForQuery.length > 200 ? '...' : ''}"
              </div>
            </div>
          )}

          {/* Input Area */}
          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              className={styles.chatInput}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={
                !isSessionReady
                  ? "Initializing session..."
                  : capturedTextForQuery && capturedTextForQuery.length > 0
                    ? "Type your question about the selected text..."
                    : "Ask your question..."
              }
              rows={1}
              disabled={isLoading || !isSessionReady}
            />
            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={isLoading || !isSessionReady || !inputValue.trim()}
              aria-label="Send message"
            >
              📤
            </button>
          </div>
        </div>
      )}
    </ErrorBoundary>
  );
}
