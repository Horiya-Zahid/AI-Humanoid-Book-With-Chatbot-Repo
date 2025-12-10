import React, { createContext, useContext, ReactNode } from 'react';
import RagChatbot from './RagChatbot';

interface ChatContextType {
  openChat: () => void;
  closeChat: () => void;
  toggleChat: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  let chatbotRef: any = null;

  const openChat = () => {
    if (chatbotRef) {
      chatbotRef.openChat();
    }
  };

  const closeChat = () => {
    if (chatbotRef) {
      chatbotRef.closeChat();
    }
  };

  const toggleChat = () => {
    if (chatbotRef) {
      chatbotRef.toggleChat();
    }
  };

  return (
    <ChatContext.Provider value={{ openChat, closeChat, toggleChat }}>
      {children}
      <RagChatbot onRef={(ref) => { chatbotRef = ref; }} />
    </ChatContext.Provider>
  );
};

export const useChat = () => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
};