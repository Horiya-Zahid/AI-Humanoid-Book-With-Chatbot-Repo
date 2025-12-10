import React from 'react';

const NavbarChat: React.FC = () => {
  const handleChatClick = () => {
    if ((window as any).chatbot && typeof (window as any).chatbot.toggleChat === 'function') {
      (window as any).chatbot.toggleChat();
    } else {
      console.error('Chatbot functions not available');
    }
  };

  return (
    <button
      onClick={handleChatClick}
      className="navbar__item navbar__link"
      aria-label="Open chat"
    >
      Chat
    </button>
  );
};

export default NavbarChat;