/**
 * SourcesList Component
 *
 * Displays retrieval sources from Qdrant with chapter names,
 * relevance scores, and optional clickable links to source pages.
 * Sources are collapsible to avoid cluttering the chat.
 */

import React, { useState } from 'react';
import styles from './styles.module.css';

interface Source {
  chapter: string;
  relevance_score: number;
  text_preview: string;
  section?: string;
  url?: string;
}

interface SourcesListProps {
  sources: Source[];
}

export default function SourcesList({ sources }: SourcesListProps): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.sourcesList}>
      <button
        className={styles.sourcesHeader}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
      >
        <span>
          📚 <strong>Sources ({sources.length})</strong>
        </span>
        <span className={styles.toggleIcon}>
          {isExpanded ? '▼' : '▶'}
        </span>
      </button>

      {isExpanded && (
        <div className={styles.sourcesContent}>
          {sources.map((source, index) => (
            <div key={index} className={styles.sourceItem}>
              <div className={styles.sourceTitle}>
                <span className={styles.sourceNumber}>{index + 1}.</span>
                {source.url ? (
                  <a
                    href={source.url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className={styles.sourceChapter}
                  >
                    {source.chapter}
                    {source.section && `: ${source.section}`}
                  </a>
                ) : (
                  <span className={styles.sourceChapter}>
                    {source.chapter}
                    {source.section && `: ${source.section}`}
                  </span>
                )}
                <span className={styles.sourceScore}>
                  {Math.round(source.relevance_score * 100)}%
                </span>
              </div>
              {source.text_preview && (
                <div className={styles.sourcePreview}>
                  {source.text_preview}
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
