"""
Chat Message model for the Vision-Language-Action (VLA) & Capstone module.
This model represents a message in a chat session between user and AI.
"""
from datetime import datetime
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field, validator
from uuid import uuid4


class SourceCitation(BaseModel):
    """Model for a source citation in a chat message."""
    module: str = Field(..., description="Module identifier (e.g., 'Module 4')")
    week: str = Field(..., description="Week identifier (e.g., 'Week 11')")
    section: str = Field(..., description="Section title")
    url: str = Field(..., description="Source URL")
    text: str = Field(..., description="Relevant text snippet")


class ChatMessageBase(BaseModel):
    """Base model for chat messages with common fields."""
    session_id: str = Field(..., description="Foreign key to chat_sessions table")
    role: str = Field(..., pattern=r"^(user|assistant)$", description="Message role ('user' or 'assistant')")
    content: str = Field(..., description="Message content")
    sources: List[SourceCitation] = Field(default_factory=list, description="Array of source citations")


class ChatMessageCreate(ChatMessageBase):
    """Model for creating a new chat message."""
    pass


class ChatMessageUpdate(BaseModel):
    """Model for updating a chat message."""
    content: Optional[str] = Field(None, description="Message content")
    sources: Optional[List[SourceCitation]] = Field(None, description="Array of source citations")


class ChatMessage(ChatMessageBase):
    """Model for a complete chat message with all fields."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Primary key, auto-generated")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Message creation timestamp")
    token_count: Optional[int] = Field(None, description="Number of tokens in the message")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "msg_abc123",
                "session_id": "sess_abc123",
                "role": "assistant",
                "content": "The vision-language-action pipeline integrates perception, reasoning, and execution...",
                "sources": [
                    {
                        "module": "Module 4",
                        "week": "Week 11",
                        "section": "Introduction to VLA Systems",
                        "url": "/docs/module-4-vla/week-11/chapter-1",
                        "text": "The vision-language-action pipeline represents the integration of perception, reasoning, and execution in robotic systems..."
                    }
                ],
                "created_at": "2025-12-06T10:00:00Z",
                "token_count": 150
            }
        }