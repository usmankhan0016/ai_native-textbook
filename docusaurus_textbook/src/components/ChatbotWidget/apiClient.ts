/**
 * API Client for RAG Backend Communication
 *
 * Provides a typed fetch wrapper for all backend API calls with:
 * - Environment-based backend URL
 * - Automatic JSON headers
 * - Error handling and typed responses
 */

// Get backend URL - safe for browser environment
// Docusaurus doesn't support process.env in client-side code
const getBackendUrl = (): string => {
  // Check if running in browser
  if (typeof window !== 'undefined') {
    // Use customField from Docusaurus config (accessed via window)
    // @ts-ignore - Docusaurus injects this globally
    if (window.docusaurus?.siteConfig?.customFields?.backendUrl) {
      // @ts-ignore
      return window.docusaurus.siteConfig.customFields.backendUrl as string;
    }
  }

  // Fallback to localhost for development
  return 'http://localhost:8000';
};

const BACKEND_URL = getBackendUrl();

/**
 * Custom error class for HTTP errors with status codes
 */
export class HttpError extends Error {
  constructor(
    public statusCode: number,
    public detail?: string,
    message?: string
  ) {
    super(message || `HTTP ${statusCode} Error`);
    this.name = 'HttpError';
  }
}

/**
 * Generic API response wrapper
 */
interface ApiResponse<T> {
  data?: T;
  error?: {
    status_code: number;
    message: string;
    path: string;
  };
}

/**
 * Fetch wrapper with automatic JSON handling and error parsing
 */
export async function apiRequest<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<T> {
  const url = `${BACKEND_URL}${endpoint}`;

  // Set default headers
  const headers = {
    'Content-Type': 'application/json',
    ...options.headers,
  };

  try {
    const response = await fetch(url, {
      ...options,
      headers,
      credentials: 'include', // Send cookies for CORS
    });

    // Parse JSON response
    const data: ApiResponse<T> = await response.json();

    // Handle error responses from backend
    if (!response.ok) {
      if (data.error) {
        throw new HttpError(
          data.error.status_code,
          data.error.message,
          data.error.message
        );
      }
      throw new HttpError(response.status, undefined, response.statusText);
    }

    // Return the data (unwrap from ApiResponse if needed)
    return (data as any).data || (data as T);
  } catch (error) {
    // Network errors or JSON parse errors
    if (error instanceof HttpError) {
      throw error;
    }

    // Network failure
    if (error instanceof TypeError) {
      throw new HttpError(0, 'Network error', 'Unable to connect to backend');
    }

    // Unknown error
    throw new HttpError(500, 'Unknown error', (error as Error).message);
  }
}

/**
 * Convenience methods for common HTTP verbs
 */
export const api = {
  get: <T>(endpoint: string) => apiRequest<T>(endpoint, { method: 'GET' }),

  post: <T>(endpoint: string, body?: any) =>
    apiRequest<T>(endpoint, {
      method: 'POST',
      body: body ? JSON.stringify(body) : undefined,
    }),

  put: <T>(endpoint: string, body?: any) =>
    apiRequest<T>(endpoint, {
      method: 'PUT',
      body: body ? JSON.stringify(body) : undefined,
    }),

  delete: <T>(endpoint: string) =>
    apiRequest<T>(endpoint, { method: 'DELETE' }),
};
