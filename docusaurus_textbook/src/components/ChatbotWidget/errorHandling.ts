/**
 * Error Handling Utilities
 *
 * Maps API error codes to user-friendly messages
 */

/**
 * Maps HTTP status codes to user-friendly error messages
 */
export function getUserErrorMessage(
  statusCode: number,
  detail?: string
): string {
  switch (statusCode) {
    case 400:
      return detail || "Your question is too long. Please try asking in 50,000 characters or fewer.";

    case 404:
      return "Your session expired. Click here to start a new conversation.";

    case 429:
      return "Service is busy. Please try again in 1 minute.";

    case 500:
      return "Something went wrong. Please try again.";

    case 0: // Network error (fetch failed)
      return "No internet connection. Please check your network and try again.";

    default:
      if (statusCode >= 500) {
        return "Something went wrong on our end. Please try again.";
      }
      if (statusCode >= 400) {
        return detail || "There was a problem with your request. Please try again.";
      }
      return "An unexpected error occurred. Please try again.";
  }
}

/**
 * Determines if error is a timeout based on elapsed time
 */
export function isTimeoutError(elapsedMs: number, mode: 'whole_book' | 'selected_text'): boolean {
  const timeoutThreshold = mode === 'whole_book' ? 15000 : 5000; // 15s or 5s
  return elapsedMs > timeoutThreshold;
}

/**
 * Gets timeout warning message
 */
export function getTimeoutWarning(mode: 'whole_book' | 'selected_text'): string {
  return "Request is taking longer than expected. You can wait or cancel.";
}

/**
 * Checks if error should trigger a retry
 */
export function shouldRetry(statusCode: number): boolean {
  // Retry on server errors and rate limiting
  return statusCode === 429 || statusCode >= 500;
}

/**
 * Gets retry delay in milliseconds based on status code
 */
export function getRetryDelay(statusCode: number): number {
  if (statusCode === 429) {
    return 60000; // 1 minute for rate limiting
  }
  if (statusCode >= 500) {
    return 3000; // 3 seconds for server errors
  }
  return 0;
}
