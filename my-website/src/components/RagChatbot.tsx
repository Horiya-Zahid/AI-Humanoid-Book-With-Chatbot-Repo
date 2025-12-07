import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

type Message = {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    module: string;
    week: string;
    section: string;
    url: string;
  }>;
  timestamp: Date;
};

const RagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [isSelecting, setIsSelecting] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Initialize with a welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        {
          id: '1',
          role: 'assistant',
          content: 'Hello! I\'m your Physical AI & Humanoid Robotics textbook assistant. I can answer questions about the content. Select any text and ask me about it!',
          sources: [],
          timestamp: new Date(),
        }
      ]);
    }
  }, [messages.length]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString();
        if (selectedText.length > 5 && selectedText.length < 500) { // Reasonable selection length
          setSelectedText(selectedText);
          setIsSelecting(true);
          // Auto-hide after 5 seconds if not clicked
          setTimeout(() => setIsSelecting(false), 5000);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Simulate API call to backend
      // In a real implementation, this would call your backend API
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Mock response - in real implementation, this would come from your RAG system
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: getMockResponse(inputValue, selectedText),
        sources: getMockSources(inputValue),
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(null);
      setIsSelecting(false);
    }
  };

  const getMockResponse = (input: string, selected?: string | null): string => {
    const lowerInput = input.toLowerCase();

    if (lowerInput.includes('ros') || lowerInput.includes('robot')) {
      return selected
        ? `Based on the selected text "${selected.substring(0, 50)}...", ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.`
        : 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify creating complex and robust robot behavior across a wide variety of robot platforms.';
    } else if (lowerInput.includes('isaac') || lowerInput.includes('nvidia')) {
      return selected
        ? `Regarding the selected text "${selected.substring(0, 50)}...", the NVIDIA Isaac™ platform is a comprehensive solution for developing, simulating, and deploying AI-powered robots. It combines hardware (Jetson platforms), software frameworks, and simulation tools to accelerate robotics development.`
        : 'The NVIDIA Isaac™ platform is a comprehensive solution for developing, simulating, and deploying AI-powered robots. It combines hardware (Jetson platforms), software frameworks, and simulation tools to accelerate robotics development.';
    } else if (lowerInput.includes('vla') || lowerInput.includes('vision-language')) {
      return selected
        ? `About the selected text "${selected.substring(0, 50)}...", Vision-Language-Action (VLA) systems represent the next generation of AI-powered robots that can perceive the world (Vision), understand and reason about it (Language), and take appropriate actions (Action) in a unified framework.`
        : 'Vision-Language-Action (VLA) systems represent the next generation of AI-powered robots that can perceive the world (Vision), understand and reason about it (Language), and take appropriate actions (Action) in a unified framework.';
    } else {
      return selected
        ? `Based on the selected text "${selected.substring(0, 50)}...", I can provide more context. This topic is covered in detail in the Physical AI & Humanoid Robotics textbook. Would you like me to elaborate on any specific aspect?`
        : 'I can help you with information from the Physical AI & Humanoid Robotics textbook. This includes topics on ROS 2, Digital Twins, NVIDIA Isaac Platform, and Vision-Language-Action systems. What would you like to know more about?';
    }
  };

  const getMockSources = (input: string): Message['sources'] => {
    const lowerInput = input.toLowerCase();

    if (lowerInput.includes('ros')) {
      return [
        {
          module: 'Module 1',
          week: 'Week 1-5',
          section: 'The Robotic Nervous System (ROS 2)',
          url: '/docs/module-1-robotic-nervous-system/week-1'
        }
      ];
    } else if (lowerInput.includes('isaac')) {
      return [
        {
          module: 'Module 3',
          week: 'Week 8-10',
          section: 'The AI-Robot Brain (NVIDIA Isaac™ Platform)',
          url: '/docs/module-3-isaac-brain/week-8'
        }
      ];
    } else if (lowerInput.includes('vla')) {
      return [
        {
          module: 'Module 4',
          week: 'Week 11-13',
          section: 'Vision-Language-Action Capstone (VLA)',
          url: '/docs/module-4-vla-capstone/week-11'
        }
      ];
    }

    return [
      {
        module: 'Full Textbook',
        week: 'All Weeks',
        section: 'Various Topics',
        url: '/docs/intro'
      }
    ];
  };

  const handleQuickQuestion = (question: string) => {
    setInputValue(question);
  };

  const clearChat = () => {
    setMessages([
      {
        id: '1',
        role: 'assistant',
        content: 'Hello! I\'m your Physical AI & Humanoid Robotics textbook assistant. I can answer questions about the content. Select any text and ask me about it!',
        sources: [],
        timestamp: new Date(),
      }
    ]);
  };

  return (
    <BrowserOnly>
      {() => (
        <>
          {/* Floating Chat Button */}
          {!isOpen && (
            <button
              onClick={() => setIsOpen(true)}
              className="fixed bottom-6 right-6 bg-[#7048e8] text-white p-4 rounded-full shadow-lg hover:bg-[#5a36d9] transition-all z-50"
              style={{ zIndex: 1000 }}
              aria-label="Open chatbot"
            >
              <svg xmlns="http://www.w3.org/2000/svg" className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
              </svg>
            </button>
          )}

          {/* Chat Container */}
          {isOpen && (
            <div
              ref={chatContainerRef}
              className="fixed bottom-6 right-6 w-full max-w-md h-[70vh] max-h-[600px] bg-white dark:bg-gray-800 rounded-xl shadow-2xl border border-gray-200 dark:border-gray-700 flex flex-col z-[1000]"
              style={{ zIndex: 1000 }}
            >
              {/* Header */}
              <div className="bg-[#7048e8] dark:bg-[#5a36d9] text-white p-4 rounded-t-xl flex justify-between items-center">
                <div>
                  <h3 className="font-bold">Physical AI Assistant</h3>
                  <p className="text-xs opacity-80">Powered by RAG from textbook content</p>
                </div>
                <div className="flex space-x-2">
                  <button
                    onClick={clearChat}
                    className="text-white hover:bg-[#5a36d9] dark:hover:bg-[#4f2cc7] p-1 rounded"
                    title="Clear chat"
                  >
                    <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" viewBox="0 0 20 20" fill="currentColor">
                      <path fillRule="evenodd" d="M9 2a1 1 0 00-.894.553L7.382 4H4a1 1 0 000 2v10a2 2 0 002 2h8a2 2 0 002-2V6a1 1 0 100-2h-3.382l-.724-1.447A1 1 0 0011 2H9zM7 8a1 1 0 012 0v6a1 1 0 11-2 0V8zm5-1a1 1 0 00-1 1v6a1 1 0 102 0V8a1 1 0 00-1-1z" clipRule="evenodd" />
                    </svg>
                  </button>
                  <button
                    onClick={() => setIsOpen(false)}
                    className="text-white hover:bg-[#5a36d9] dark:hover:bg-[#4f2cc7] p-1 rounded"
                    title="Close chat"
                  >
                    <svg xmlns="http://www.w3.org/2000/svg" className="h-5 w-5" viewBox="0 0 20 20" fill="currentColor">
                      <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                    </svg>
                  </button>
                </div>
              </div>

              {/* Messages Container */}
              <div className="flex-1 overflow-y-auto p-4 bg-gray-50 dark:bg-gray-900">
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={`mb-4 flex ${message.role === 'user' ? 'justify-end' : 'justify-start'}`}
                  >
                    <div
                      className={`max-w-[80%] rounded-xl p-3 ${
                        message.role === 'user'
                          ? 'bg-[#7048e8] text-white rounded-br-none'
                          : 'bg-white dark:bg-gray-700 text-gray-800 dark:text-gray-200 rounded-bl-none'
                      }`}
                    >
                      <div className="whitespace-pre-wrap">{message.content}</div>

                      {message.sources && message.sources.length > 0 && (
                        <div className="mt-2 pt-2 border-t border-gray-200 dark:border-gray-600 text-xs">
                          <p className="font-semibold text-[#39ff14]">Sources:</p>
                          {message.sources.map((source, idx) => (
                            <div key={idx} className="mt-1">
                              <a
                                href={source.url}
                                className="text-blue-500 hover:text-blue-700 dark:hover:text-blue-400 underline"
                                target="_blank"
                                rel="noopener noreferrer"
                              >
                                {source.module} → {source.week} → {source.section}
                              </a>
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className="mb-4 flex justify-start">
                    <div className="bg-white dark:bg-gray-700 text-gray-800 dark:text-gray-200 rounded-xl p-3 rounded-bl-none max-w-[80%]">
                      <div className="flex items-center">
                        <div className="w-2 h-2 bg-[#7048e8] rounded-full mr-1 animate-bounce"></div>
                        <div className="w-2 h-2 bg-[#7048e8] rounded-full mr-1 animate-bounce delay-100"></div>
                        <div className="w-2 h-2 bg-[#7048e8] rounded-full animate-bounce delay-200"></div>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              {/* Input Area */}
              <div className="p-4 border-t border-gray-200 dark:border-gray-700 bg-white dark:bg-gray-800">
                {selectedText && isSelecting && (
                  <div className="mb-2 p-2 bg-blue-50 dark:bg-blue-900/30 rounded text-sm text-blue-700 dark:text-blue-300">
                    Selected: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"
                  </div>
                )}

                <form onSubmit={handleSubmit} className="flex space-x-2">
                  <input
                    type="text"
                    value={inputValue}
                    onChange={(e) => setInputValue(e.target.value)}
                    placeholder={selectedText ? "Ask about selected text..." : "Ask about the textbook content..."}
                    className="flex-1 border border-gray-300 dark:border-gray-600 rounded-lg px-3 py-2 bg-white dark:bg-gray-700 text-gray-900 dark:text-white focus:outline-none focus:ring-2 focus:ring-[#7048e8]"
                    disabled={isLoading}
                  />
                  <button
                    type="submit"
                    disabled={!inputValue.trim() || isLoading}
                    className="bg-[#7048e8] text-white px-4 py-2 rounded-lg hover:bg-[#5a36d9] disabled:opacity-50 disabled:cursor-not-allowed"
                  >
                    Send
                  </button>
                </form>

                {/* Quick questions */}
                <div className="mt-2 flex flex-wrap gap-1">
                  <button
                    onClick={() => handleQuickQuestion("What is ROS 2?")}
                    className="text-xs bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600 px-2 py-1 rounded"
                  >
                    What is ROS 2?
                  </button>
                  <button
                    onClick={() => handleQuickQuestion("Explain Isaac SLAM")}
                    className="text-xs bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600 px-2 py-1 rounded"
                  >
                    Isaac SLAM
                  </button>
                  <button
                    onClick={() => handleQuickQuestion("VLA systems")}
                    className="text-xs bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600 px-2 py-1 rounded"
                  >
                    VLA Systems
                  </button>
                </div>
              </div>
            </div>
          )}
        </>
      )}
    </BrowserOnly>
  );
};

export default RagChatbot;