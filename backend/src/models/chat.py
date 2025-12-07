"""
Chat models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request and response models for the chat API.
"""
from datetime import datetime
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from uuid import uuid4


class SourceCitation(BaseModel):
    """Model for a source citation in a chat response."""
    module: str = Field(..., description="Module identifier (e.g., 'Module 4')")
    week: str = Field(..., description="Week identifier (e.g., 'Week 11')")
    section: str = Field(..., description="Section title")
    url: str = Field(..., description="Source URL")
    text: str = Field(..., description="Relevant text snippet")


class ChatMessage(BaseModel):
    """Model for a message in a chat response."""
    role: str = Field(..., pattern=r"^(user|assistant)$", description="Message role ('user' or 'assistant')")
    content: str = Field(..., description="The response content")
    sources: List[SourceCitation] = Field(default_factory=list, description="Array of source citations")


class ChatChoice(BaseModel):
    """Model for a choice in a chat response."""
    index: int = 0
    message: ChatMessage
    finish_reason: str = "stop"


class UsageInfo(BaseModel):
    """Model for usage information in a chat response."""
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0


class ChatResponse(BaseModel):
    """Model for the chat API response."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique response ID")
    object: str = "text_completion"
    created: int = Field(default_factory=lambda: int(datetime.utcnow().timestamp()), description="Unix timestamp")
    model: str = Field(default="gemini-pro", description="Model identifier")
    choices: List[ChatChoice] = Field(default_factory=list)
    usage: Optional[UsageInfo] = None
    session_id: Optional[str] = Field(None, description="Updated session ID")


class ChatRequest(BaseModel):
    """Model for the chat API request."""
    query: str = Field(..., min_length=1, max_length=1000, description="User's question or query")
    module_id: Optional[str] = Field(None, description="Module ID to scope search (default: all modules)")
    session_id: Optional[str] = Field(None, description="Chat session ID for conversation history")
    user_id: Optional[str] = Field(None, description="User ID for personalization")
    include_citations: bool = Field(default=True, description="Whether to include source citations (default: true)")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "query": "Explain the vision-language-action pipeline in robotics",
                "module_id": "004-module-4-vla",
                "session_id": "sess_abc123",
                "include_citations": True
            }
        }