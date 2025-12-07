"""
Chat Session model for the Vision-Language-Action (VLA) & Capstone module.
This model represents a chat session between a user and the AI system.
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from uuid import uuid4


class ChatSessionBase(BaseModel):
    """Base model for chat sessions with common fields."""
    user_id: str = Field(..., description="Foreign key to users table")
    module_id: str = Field(..., description="Module identifier for the session")


class ChatSessionCreate(ChatSessionBase):
    """Model for creating a new chat session."""
    pass


class ChatSessionUpdate(BaseModel):
    """Model for updating a chat session."""
    title: Optional[str] = Field(None, max_length=255, description="Auto-generated title from first query")


class ChatSession(ChatSessionBase):
    """Model for a complete chat session with all fields."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Primary key, auto-generated")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last interaction timestamp")
    title: Optional[str] = Field(None, max_length=255, description="Auto-generated title from first query")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "sess_abc123",
                "user_id": "user_abc123",
                "module_id": "004-module-4-vla",
                "created_at": "2025-12-06T10:00:00Z",
                "updated_at": "2025-12-06T10:00:00Z",
                "title": "VLA Pipeline Questions"
            }
        }