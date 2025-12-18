import React, { useState, useRef, useEffect } from "react";
import styles from "./styles.module.css";

interface Message {
  id: string;
  type: "user" | "bot";
  content: string;
  timestamp: Date;
}

const SAMPLE_RESPONSES = [
  "I'm here to help you understand Physical AI and Humanoid Robotics! Ask me anything about ROS 2, Digital Twins, NVIDIA Isaac, or Vision-Language-Action models.",
  "That's a great question! Based on the textbook content, here's what you need to know...",
  "Let me break that down for you. This concept is covered in detail in Module 3: NVIDIA Isaac Platform.",
  "Good question! This relates to the core principles we discuss in the Vision-Language-Action section.",
];

export default function ChatbotWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: "welcome",
      type: "bot",
      content:
        "Hello! 👋 I'm your AI-Native Robotics Textbook Assistant. Ask me anything about the course content!",
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = () => {
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: `user-${Date.now()}`,
      type: "user",
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue("");
    setIsLoading(true);

    // Simulate bot response delay
    setTimeout(() => {
      const botMessage: Message = {
        id: `bot-${Date.now()}`,
        type: "bot",
        content:
          SAMPLE_RESPONSES[Math.floor(Math.random() * SAMPLE_RESPONSES.length)],
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, botMessage]);
      setIsLoading(false);
    }, 800);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
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
              <h3 className={styles.chatTitle}>AI Robotics Assistant</h3>
              <p className={styles.chatSubtitle}>
                Powered by RAG & LLM
              </p>
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
                className={`${styles.messageWrapper} ${
                  message.type === "user"
                    ? styles.userMessage
                    : styles.botMessage
                }`}
              >
                <div className={styles.messageBubble}>
                  {message.content}
                  {message.type === "bot" && (
                    <div className={styles.sources}>
                      <small>📚 Sources will be shown here</small>
                    </div>
                  )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={styles.messageWrapper}>
                <div className={styles.messageBubble}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className={styles.inputContainer}>
            <textarea
              className={styles.chatInput}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask your question..."
              rows={1}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              📤
            </button>
          </div>
        </div>
      )}
    </>
  );
}
