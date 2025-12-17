import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

interface UserBackground {
  software_experience?: string;
  hardware_experience?: string;
  robotics_knowledge?: string;
  preferred_language?: string;
}

interface RAGQuery {
  question: string;
  user_background?: UserBackground;
  top_k?: number;
  context_window?: number;
}

interface TextSelection {
  selected_text: string;
  question: string;
  user_background?: UserBackground;
  top_k?: number;
}

interface RAGResponse {
  question: string;
  answer: string;
  context: string[];
  sources: string[];
  user_background?: UserBackground;
}

interface TranslationRequest {
  text: string;
  target_language: string;
}

interface TranslationResponse {
  original_text: string;
  translated_text: string;
  target_language: string;
}

interface ChatWidgetProps {
  apiEndpoint?: string;
}

export default function ChatWidget({ apiEndpoint }: ChatWidgetProps): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [preferredLanguage, setPreferredLanguage] = useState<'en' | 'ur'>('en');
  const [strictMode, setStrictMode] = useState(false); // Strict selection mode: answer ONLY from selected text
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Default user background (in a real app, this would come from user profile)
  const [userBackground, setUserBackground] = useState<UserBackground>({
    software_experience: 'beginner',
    hardware_experience: 'beginner',
    robotics_knowledge: 'basic',
    preferred_language: 'en'
  });

  // Update user background when language preference changes
  useEffect(() => {
    setUserBackground(prev => ({
      ...prev,
      preferred_language: preferredLanguage
    }));
  }, [preferredLanguage]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 0) {
        setSelectedText(selection.toString().trim());
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const callBackendAPI = async (query: RAGQuery) => {
    try {
      const response = await fetch(`${apiEndpoint || '/api'}/rag/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(query),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: RAGResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error calling backend API:', error);
      // Return a fallback response
      return {
        question: query.question,
        answer: `I'm having trouble connecting to the backend. Please check that the server is running. Error: ${error instanceof Error ? error.message : 'Unknown error'}`,
        context: [],
        sources: [],
        user_background: query.user_background
      };
    }
  };

  const callSelectionAPI = async (selection: TextSelection) => {
    try {
      const response = await fetch(`${apiEndpoint || '/api'}/rag/selection-query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(selection),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: RAGResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error calling selection API:', error);
      // Return a fallback response
      return {
        question: selection.question,
        answer: `I'm having trouble connecting to the backend. Please check that the server is running. Error: ${error instanceof Error ? error.message : 'Unknown error'}`,
        context: [],
        sources: [],
        user_background: selection.user_background
      };
    }
  };

  // Strict selection mode API - answers ONLY from selected text (no Qdrant search)
  const callStrictSelectionAPI = async (selection: TextSelection) => {
    try {
      const response = await fetch(`${apiEndpoint || '/api'}/rag/answer-from-selection`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          selected_text: selection.selected_text,
          question: selection.question,
          user_background: selection.user_background
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return {
        question: data.question,
        answer: data.answer,
        context: [data.selected_text],
        sources: data.sources || ['User-selected text'],
        user_background: data.user_background
      } as RAGResponse;
    } catch (error) {
      console.error('Error calling strict selection API:', error);
      return {
        question: selection.question,
        answer: `I'm having trouble connecting to the backend. Please check that the server is running. Error: ${error instanceof Error ? error.message : 'Unknown error'}`,
        context: [],
        sources: [],
        user_background: selection.user_background
      };
    }
  };

  const translateText = async (text: string, targetLanguage: string): Promise<string> => {
    try {
      const response = await fetch(`${apiEndpoint || '/api'}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: text,
          target_language: targetLanguage
        } as TranslationRequest),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: TranslationResponse = await response.json();
      return data.translated_text;
    } catch (error) {
      console.error('Error calling translation API:', error);
      return text; // Return original text if translation fails
    }
  };

  const sendMessage = async (query: string, isSelectionQuery: boolean = false) => {
    if (!query.trim() || isLoading) return;

    setIsLoading(true);

    const userMessage: Message = { role: 'user', content: query };
    setMessages(prev => [...prev, userMessage]);
    setInput('');

    try {
      let response: RAGResponse;

      if (isSelectionQuery && selectedText) {
        // Build selection query
        const selectionQuery: TextSelection = {
          selected_text: selectedText,
          question: query,
          user_background: userBackground,
          top_k: 3
        };

        // Use strict mode (answer ONLY from selection) or enhanced mode (with Qdrant search)
        if (strictMode) {
          response = await callStrictSelectionAPI(selectionQuery);
        } else {
          response = await callSelectionAPI(selectionQuery);
        }
      } else {
        // Call general RAG API
        const ragQuery: RAGQuery = {
          question: query,
          user_background: userBackground,
          top_k: 5
        };
        response = await callBackendAPI(ragQuery);
      }

      // If preferred language is Urdu, translate the response
      let finalAnswer = response.answer;
      if (preferredLanguage === 'ur') {
        finalAnswer = await translateText(response.answer, 'ur');
      }

      const assistantMessage: Message = { role: 'assistant', content: finalAnswer };
      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText('');
    } catch (error) {
      const errorMessage: Message = {
        role: 'assistant',
        content: `Error: ${error instanceof Error ? error.message : 'An unknown error occurred'}`
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(input, selectedText.length > 0);
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage(input, selectedText.length > 0);
    }
  };

  const toggleLanguage = () => {
    const newLanguage = preferredLanguage === 'en' ? 'ur' : 'en';
    setPreferredLanguage(newLanguage as 'en' | 'ur');
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Physical AI Assistant</h3>
            <span className={styles.subtitle}>Ask questions about the textbook</span>
            <div className={styles.languageToggle}>
              <button
                className={`${styles.langButton} ${preferredLanguage === 'en' ? styles.activeLang : ''}`}
                onClick={() => setPreferredLanguage('en')}
                title="Switch to English"
              >
                EN
              </button>
              <button
                className={`${styles.langButton} ${preferredLanguage === 'ur' ? styles.activeLang : ''}`}
                onClick={() => setPreferredLanguage('ur')}
                title="Switch to Urdu"
              >
                UR
              </button>
            </div>
          </div>

          {/* Selected Text Indicator */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <span>Selected: "{selectedText.slice(0, 50)}{selectedText.length > 50 ? '...' : ''}"</span>
              <div className={styles.selectionControls}>
                <label className={styles.strictModeToggle} title="Strict mode: Answer ONLY from selected text">
                  <input
                    type="checkbox"
                    checked={strictMode}
                    onChange={(e) => setStrictMode(e.target.checked)}
                  />
                  <span className={styles.toggleLabel}>Strict</span>
                </label>
                <button onClick={() => setSelectedText('')}>Clear</button>
              </div>
            </div>
          )}

          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>Welcome! Ask me anything about Physical AI and Humanoid Robotics.</p>
                <p className={styles.tip}>Tip: Select text on the page, then ask a question about it!</p>
              </div>
            )}
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.assistantMessage}`}
              >
                {msg.content}
              </div>
            ))}
            {isLoading && (
              <div className={styles.message + ' ' + styles.assistantMessage}>
                <div className={styles.typingIndicator}>
                  <div className={styles.dot}></div>
                  <div className={styles.dot}></div>
                  <div className={styles.dot}></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputContainer}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={selectedText ? "Ask about selected text..." : "Ask a question..."}
              className={styles.input}
              disabled={isLoading}
            />
            <button type="submit" disabled={!input.trim() || isLoading} className={styles.sendButton}>
              {isLoading ? (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10" strokeDasharray="15.7 15.7"></circle>
                </svg>
              ) : (
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              )}
            </button>
          </form>
        </div>
      )}
    </>
  );
}
