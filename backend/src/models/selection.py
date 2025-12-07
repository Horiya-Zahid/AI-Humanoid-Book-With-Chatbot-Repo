"""
Selection models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request model for the text selection API.
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class SelectionRequest(BaseModel):
    """Model for the text selection API request."""
    selected_text: str = Field(..., min_length=1, max_length=1000, description="The user-selected text")
    original_context: Optional[str] = Field(None, max_length=2000, description="Context around the selected text")
    module_id: Optional[str] = Field(None, description="Module ID to scope search (default: all modules)")
    page_url: Optional[str] = Field(None, description="URL of the page where text was selected")
    session_id: Optional[str] = Field(None, description="Chat session ID for conversation history")
    user_id: Optional[str] = Field(None, description="User ID for personalization")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "selected_text": "vision-language-action pipeline",
                "original_context": "In robotics, the vision-language-action pipeline integrates perception, reasoning, and execution...",
                "module_id": "004-module-4-vla",
                "page_url": "https://example.com/module-4-vla/chapter-3",
                "session_id": "sess_abc123"
            }
        }