/**
 * SelectionContextMenu Component
 *
 * Displays a floating "Ask about this" button when text is selected on the page.
 * Clicking the button opens the chatbot with the selected text.
 */

import React from 'react';
import styles from './styles.module.css';

interface SelectionContextMenuProps {
  selectedText: string;
  position: { x: number; y: number } | null;
  onAskAboutSelection: () => void;
  onClose: () => void;
}

export default function SelectionContextMenu({
  selectedText,
  position,
  onAskAboutSelection,
  onClose,
}: SelectionContextMenuProps): JSX.Element | null {
  // Only check if position and selectedText exist
  // Minimum 2 chars - matches main component
  if (!position || !selectedText || selectedText.length < 2) {
    return null;
  }

  return (
    <div
      className={styles.selectionMenu}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
    >
      <button
        className={styles.selectionMenuButton}
        onClick={onAskAboutSelection}
        title="Ask a question about this selected text"
      >
        💬 Ask about this
      </button>
      <button
        className={styles.selectionMenuClose}
        onClick={onClose}
        title="Close"
      >
        ✕
      </button>
    </div>
  );
}
