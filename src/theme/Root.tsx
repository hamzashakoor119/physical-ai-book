import React, { Suspense, lazy } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Lazy load ChatWidget to prevent blocking the main app
const ChatWidget = lazy(() => import('@site/src/components/ChatWidget'));

// Helper to get backend URL
function getBackendUrl(): string {
  // For production: set BACKEND_URL in docusaurus.config.ts customFields
  // For development: defaults to localhost
  if (typeof window !== 'undefined') {
    // Check for custom field from Docusaurus config
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    const customBackendUrl = docusaurusConfig?.siteConfig?.customFields?.BACKEND_URL;
    if (customBackendUrl) {
      return customBackendUrl as string;
    }
  }
  // Default to localhost for development
  return 'http://localhost:8000/api';
}

interface RootProps {
  children: React.ReactNode;
}

// Error boundary to catch ChatWidget errors without crashing the whole app
class ChatWidgetErrorBoundary extends React.Component<
  { children: React.ReactNode },
  { hasError: boolean }
> {
  constructor(props: { children: React.ReactNode }) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError() {
    return { hasError: true };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    console.error('ChatWidget Error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      return null; // Don't show anything if ChatWidget crashes
    }
    return this.props.children;
  }
}

export default function Root({ children }: RootProps): React.ReactElement {
  const backendUrl = getBackendUrl();

  return (
    <>
      {children}
      <ChatWidgetErrorBoundary>
        <Suspense fallback={null}>
          <ChatWidget apiEndpoint={backendUrl} />
        </Suspense>
      </ChatWidgetErrorBoundary>
    </>
  );
}
