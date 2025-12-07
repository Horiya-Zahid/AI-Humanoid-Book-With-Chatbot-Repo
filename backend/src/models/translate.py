"""
Translation models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request and response models for the translation API.
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, Field


class TranslateChunk(BaseModel):
    """Model for a single chunk to translate."""
    chunk_id: str = Field(..., description="The ID of the content chunk to translate")
    content: str = Field(..., description="The content to translate")
    target_language: str = Field(..., description="The target language code")


class TranslateBatchRequest(BaseModel):
    """Model for batch translation request."""
    chunks: List[TranslateChunk] = Field(..., description="List of chunks to translate")
    source_language: str = Field(default="en", description="Source language code (default: 'en')")
    requester_id: Optional[str] = Field(None, description="ID of the requesting user/admin")
    priority: str = Field(default="normal", pattern=r"^(low|normal|high|critical)$", description="Priority level")
    callback_url: Optional[str] = Field(None, description="URL to notify when translations are ready")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "chunks": [
                    {
                        "chunk_id": "chunk_004_11_1_001",
                        "content": "The vision-language-action pipeline represents the integration of perception, reasoning, and execution in robotic systems...",
                        "target_language": "ur"
                    }
                ],
                "source_language": "en",
                "requester_id": "admin_user_123",
                "priority": "normal",
                "callback_url": "https://myapp.com/translation-callback"
            }
        }


class TranslationResponse(BaseModel):
    """Model for a single translation response."""
    chunk_id: str = Field(..., description="Original chunk ID")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(..., description="Target language code")
    translated_content: str = Field(..., description="The translated content")
    status: str = Field(..., pattern=r"^(published|reviewed|pending)$", description="Translation status")
    quality_score: float = Field(ge=0.0, le=1.0, description="Quality score between 0 and 1")
    translator_notes: Optional[str] = Field(None, description="Any notes from the translator")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    metadata: dict = Field(default_factory=dict, description="Additional metadata")


class TranslateBatchResponse(BaseModel):
    """Model for batch translation response."""
    request_id: str = Field(..., description="Unique ID for the batch translation request")
    status: str = Field(..., description="Initial status ('received', 'processing', 'queued')")
    total_chunks: int = Field(..., description="Number of chunks in the request")
    target_language: str = Field(..., description="Target language for all translations")
    estimated_completion: datetime = Field(..., description="Estimated completion time")
    created_at: datetime = Field(..., description="Request creation timestamp")


class TranslationStatusDetail(BaseModel):
    """Model for detailed status of a single chunk."""
    chunk_id: str = Field(..., description="Failed chunk ID")
    error: Optional[str] = Field(None, description="Error message for the failure")


class TranslationStatusResponse(BaseModel):
    """Model for translation status response."""
    request: dict = Field(..., description="Request information")
    details: dict = Field(..., description="Detailed status information")


class TranslationStatusCheck(BaseModel):
    """Model for translation status check."""
    request_id: str = Field(..., description="Request ID")
    status: str = Field(..., description="Current status")
    total_chunks: int = Field(..., description="Total number of chunks")
    completed_chunks: int = Field(..., description="Number of chunks completed")
    failed_chunks: int = Field(..., description="Number of chunks that failed")
    progress_percentage: float = Field(ge=0.0, le=100.0, description="Completion percentage")
    target_language: str = Field(..., description="Target language")
    created_at: datetime = Field(..., description="Request creation timestamp")
    updated_at: datetime = Field(..., description="Last status update timestamp")
    completed_at: Optional[datetime] = Field(None, description="Completion timestamp")

    details: dict = Field(default_factory=dict, description="Additional details")