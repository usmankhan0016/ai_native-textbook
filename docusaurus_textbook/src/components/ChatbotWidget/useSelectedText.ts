/**
 * useSelectedText Hook
 *
 * Handles text selection detection and coordinates for context menu
 */

import { useState, useCallback, useEffect } from 'react';

interface SelectionCoords {
  x: number;
  y: number;
}

export function useSelectedText() {
  const [selectedText, setSelectedText] = useState<string>('');
  const [selectionCoords, setSelectionCoords] = useState<SelectionCoords | null>(null);

  /**
   * Get currently selected text from the page
   */
  const getSelectedText = useCallback((): string => {
    if (typeof window === 'undefined') return '';

    const selection = window.getSelection();
    return selection?.toString().trim() || '';
  }, []);

  /**
   * Get coordinates of the current selection for context menu positioning
   */
  const getSelectionCoords = useCallback((event: MouseEvent): SelectionCoords => {
    return {
      x: event.clientX,
      y: event.clientY,
    };
  }, []);

  /**
   * Get chapter context from selected text location (optional)
   * Attempts to find the nearest heading or article element
   */
  const getChapterContext = useCallback((): string | null => {
    if (typeof window === 'undefined') return null;

    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return null;

    const range = selection.getRangeAt(0);
    let element: HTMLElement | null = range.commonAncestorContainer as HTMLElement;

    // Traverse up the DOM to find nearest article or heading
    while (element && element !== document.body) {
      // Check for article element with title
      if (element.tagName === 'ARTICLE') {
        const heading = element.querySelector('h1, h2');
        if (heading) {
          return heading.textContent?.trim() || null;
        }
      }

      // Check for nearby headings
      if (element.tagName?.match(/^H[1-6]$/)) {
        return element.textContent?.trim() || null;
      }

      element = element.parentElement;
    }

    return null;
  }, []);

  /**
   * Clear selected text
   */
  const clearSelection = useCallback(() => {
    setSelectedText('');
    setSelectionCoords(null);
  }, []);

  /**
   * Handle selection change event
   */
  const handleSelectionChange = useCallback(() => {
    const text = getSelectedText();
    setSelectedText(text);
  }, [getSelectedText]);

  // Listen for selection changes
  useEffect(() => {
    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return {
    selectedText,
    selectionCoords,
    getSelectedText,
    getSelectionCoords,
    getChapterContext,
    clearSelection,
    setSelectionCoords,
  };
}
