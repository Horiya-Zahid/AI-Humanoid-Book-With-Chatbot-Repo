
import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

type Message = {
  id: string; role: 'user' | 'assistant'; content: string; sources?: { module: string; week: string; section: string; url: string }[]; timestamp: Date };

const RagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [isSelecting, setIsSelecting] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Welcome message
 // NAVBAR SE CHAT KHOLNE KA 100% WORKING CODE
useEffect(() => {
  (window as any).openRagChat = () => setIsOpen(true);
  const btn = document.getElementById('open-rag-chat');
  if (btn) btn.onclick = () => setIsOpen(true);
}, []);

  // Auto scroll
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Text selection
  useEffect(() => {
    const handler = () => {
      const sel = window.getSelection()?.toString().trim();
      if (sel && sel.length > 5 && sel.length < 500) {
        setSelectedText(sel);
        setIsSelecting(true);
        setTimeout(() => setIsSelecting(false), 6000);
      }
    };
    document.addEventListener('mouseup', handler);
    return () => document.removeEventListener('mouseup', handler);
  }, []);

  // Open chat from navbar
  useEffect(() => {
    const btn = document.getElementById('open-rag-chat');
    if (btn) btn.onclick = () => setIsOpen(true);
  }, []);

  const sendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMsg: Message = { id: Date.now().toString(), role: 'user', content: inputValue, timestamp: new Date() };
    setMessages(prev => [...prev, userMsg]);
    setInputValue('');
    setIsLoading(true);

    try {
      const res = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: inputValue, selected_text: selectedText || undefined }),
      });
      const data = await res.json();

      setMessages(prev => [...prev, {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.answer || 'No answer',
        sources: data.sources || [],
        timestamp: new Date(),
      }]);
    } catch {
      setMessages(prev => [...prev, {
        id: 'err',
        role: 'assistant',
        content: 'Backend not running!',
        timestamp: new Date(),
      }]);
    } finally {
      setIsLoading(false);
      setSelectedText(null);
    }
  };

  return (
    <BrowserOnly>
      {() => (
        <>
          {/* MODAL CHAT BOX */}
          {isOpen && (
            <div
              className="fixed inset-0 bg-black/70 backdrop-blur-sm z-[9998] flex items-center justify-center p-4"
              onClick={() => setIsOpen(false)}
            >
              <div
                className="bg-[#0f0f1a] rounded-3xl shadow-2xl border-2 border-[#7048e8] w-full max-w-md h-[90vh] max-h-[700px] flex flex-col"
                onClick={e => e.stopPropagation()}
              >
                {/* Header */}
                <div className="bg-gradient-to-r from-[#7048e8] to-[#a78bfa] p-6 rounded-t-3xl flex justify-between items-center border-b-2 border-[#39ff14]">
                  <div>
                    <h3 className="text-white text-2xl font-bold">Physical AI Assistant</h3>
                    <p className="text-[#39ff14] text-sm">RAG-Powered · Select text to ask</p>
                  </div>
                  <button onClick={() => setIsOpen(false)} className="text-white text-4xl hover:text-[#39ff14]">
                    ×
                  </button>
                </div>

                {/* Messages */}
                <div className="flex-1 overflow-y-auto p-6 space-y-4">
                  {messages.map(msg => (
                    <div key={msg.id} className={msg.role === 'user' ? 'text-right' : 'text-left'}>
                      <div className={`inline-block max-w-md rounded-2xl px-5 py-3 ${
                        msg.role === 'user'
                          ? 'bg-gradient-to-br from-[#7048e8] to-[#5b3cd1] text-white'
                          : 'bg-[#16213e] text-gray-100 border border-[#7048e8]/40'
                      }`}>
                        <p className="whitespace-pre-wrap">{msg.content}</p>
                        {msg.sources?.length > 0 && (
                          <div className="mt-3 pt-3 border-t border-[#39ff14]/30 text-xs">
                            <p className="text-[#39ff14] font-bold mb-1">Sources:</p>
                            {msg.sources.map((s, i) => (
                              <a key={i} href={s.url} target="_blank" className="block text-cyan-400 hover:text-cyan-300 underline">
                                {s.module} ▸ {s.week} ▸ {s.section}
                              </a>
                            ))}
                          </div>
                        )}
                      </div>
                    </div>
                  ))}
                  {isLoading && (
                    <div className="text-left">
                      <div className="inline-block bg-[#16213e] rounded-2xl px-5 py-3 border border-[#7048e8]/40">
                        <div className="flex gap-2">
                          <div className="w-2 h-2 bg-[#39ff14] rounded-full animate-bounce"></div>
                          <div className="w-2 h-2 bg-[#39ff14] rounded-full animate-bounce delay-100"></div>
                          <div className="w-2 h-2 bg-[#39ff14] rounded-full animate-bounce delay-200"></div>
                        </div>
                      </div>
                    </div>
                  )}
                  <div ref={messagesEndRef} />
                </div>

                {/* Input */}
                <div className="p-6 border-t-2 border-[#7048e8]/50">
                  {selectedText && isSelecting && (
                    <div className="mb-4 p-3 bg-[#16213e]/70 rounded-xl border border-[#39ff14]/50 text-cyan-300 text-sm">
                      Selected: "{selectedText.substring(0, 80)}..."
                    </div>
                  )}
                  <form onSubmit={sendMessage} className="flex gap-3">
                    <input
                      value={inputValue}
                      onChange={e => setInputValue(e.target.value)}
                      placeholder={selectedText ? "Ask about selected text..." : "Ask anything..."}
                      className="flex-1 bg-[#16213e] border border-[#7048e8] text-white rounded-xl px-5 py-4 focus:outline-none focus:ring-2 focus:ring-[#39ff14]"
                      disabled={isLoading}
                    />
                    <button
                      type="submit"
                      disabled={isLoading || !inputValue.trim()}
                      className="bg-gradient-to-r from-[#7048e8] to-[#a78bfa] hover:from-[#5b3cd1] hover:to-[#8b68ee] text-white font-bold px-8 py-4 rounded-xl disabled:opacity-50"
                    >
                      Send
                    </button>
                  </form>
                  <div className="mt-4 flex flex-wrap gap-2">
                    {['What is ROS 2?', 'Explain Isaac Sim', 'How does VLA work?', 'What is a Digital Twin?'].map(q => (
                      <button
                        key={q}
                        onClick={() => setInputValue(q)}
                        className="text-xs bg-[#16213e] hover:bg-[#7048e8] text-[#a78bfa] hover:text-white px-4 py-2 rounded-lg border border-[#7048e8]/50 transition"
                      >
                        {q}
                      </button>
                    ))}
                  </div>
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