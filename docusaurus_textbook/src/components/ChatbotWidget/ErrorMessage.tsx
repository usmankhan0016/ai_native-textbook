import React from 'react';
import styles from './styles.module.css';

export interface ErrorMessageProps {
  message: string;
  type?: 'error' | 'timeout' | 'warning';
  onRetry?: () => void;
  onDismiss?: () => void;
  onCancel?: () => void;
  showRetry?: boolean;
  retryDisabled?: boolean;
  retryCountdown?: number;
}

/**
 * ErrorMessage Component
 * Displays user-friendly error messages with retry/dismiss actions
 */
export default function ErrorMessage({
  message,
  type = 'error',
  onRetry,
  onDismiss,
  onCancel,
  showRetry = true,
  retryDisabled = false,
  retryCountdown,
}: ErrorMessageProps): JSX.Element {
  const isTimeout = type === 'timeout';
  const isWarning = type === 'warning';

  if (isTimeout) {
    return (
      <div className={styles.timeoutWarning}>
        <div className={styles.timeoutWarningTitle}>
          ⏱️ Taking Longer Than Expected
        </div>
        <p className={styles.timeoutWarningContent}>
          {message}
        </p>
        <div className={styles.errorMessageActions}>
          {onCancel && (
            <button
              className={styles.cancelButton}
              onClick={onCancel}
              aria-label="Cancel request"
            >
              Cancel Request
            </button>
          )}
          <button
            className={styles.waitButton}
            onClick={onDismiss}
            aria-label="Continue waiting"
          >
            Keep Waiting
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.errorMessage}>
      <div className={styles.errorMessageTitle}>
        {isWarning ? '⚠️' : '❌'} {isWarning ? 'Warning' : 'Error'}
      </div>
      <p className={styles.errorMessageContent}>
        {message}
      </p>
      <div className={styles.errorMessageActions}>
        {showRetry && onRetry && (
          <button
            className={styles.retryButton}
            onClick={onRetry}
            disabled={retryDisabled}
            aria-label={retryDisabled ? `Retry in ${retryCountdown} seconds` : 'Retry'}
          >
            {retryDisabled && retryCountdown !== undefined
              ? `Retry in ${retryCountdown}s`
              : 'Retry'}
          </button>
        )}
        {onDismiss && (
          <button
            className={styles.dismissButton}
            onClick={onDismiss}
            aria-label="Dismiss error"
          >
            Dismiss
          </button>
        )}
      </div>
    </div>
  );
}
