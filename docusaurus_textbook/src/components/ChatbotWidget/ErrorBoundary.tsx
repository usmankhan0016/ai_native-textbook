import React, { Component, ErrorInfo, ReactNode } from 'react';
import styles from './styles.module.css';

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

/**
 * Error Boundary Component for ChatbotWidget
 * Catches React errors and displays fallback UI
 * Prevents full page crashes
 */
export default class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
    };
  }

  static getDerivedStateFromError(error: Error): Partial<State> {
    // Update state so the next render will show the fallback UI
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    // Log error details for debugging
    console.error('[ErrorBoundary] Caught error:', error);
    console.error('[ErrorBoundary] Error info:', errorInfo);

    this.setState({
      error,
      errorInfo,
    });
  }

  handleReset = () => {
    // Reset error state and try again
    this.setState({
      hasError: false,
      error: null,
      errorInfo: null,
    });
  };

  render() {
    if (this.state.hasError) {
      // Fallback UI
      return (
        <div className={styles.errorBoundaryContainer}>
          <div className={styles.errorBoundaryContent}>
            <div className={styles.errorBoundaryIcon}>⚠️</div>
            <h3 className={styles.errorBoundaryTitle}>
              Chat Service Unavailable
            </h3>
            <p className={styles.errorBoundaryMessage}>
              The chatbot encountered an unexpected error. Please try refreshing the page.
            </p>
            <div className={styles.errorBoundaryActions}>
              <button
                className={styles.errorBoundaryButton}
                onClick={() => window.location.reload()}
              >
                Refresh Page
              </button>
              <button
                className={styles.errorBoundaryButtonSecondary}
                onClick={this.handleReset}
              >
                Try Again
              </button>
            </div>

            {/* Show error details in development */}
            {process.env.NODE_ENV === 'development' && this.state.error && (
              <details className={styles.errorBoundaryDetails}>
                <summary>Error Details (Development Only)</summary>
                <pre className={styles.errorBoundaryPre}>
                  <code>
                    {this.state.error.toString()}
                    {'\n\n'}
                    {this.state.errorInfo?.componentStack}
                  </code>
                </pre>
              </details>
            )}
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}
