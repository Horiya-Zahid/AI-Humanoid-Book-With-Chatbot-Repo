"""
Embed models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request and response models for the embedding API.
"""
from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel, Field
from src.models.content_chunk import ContentChunkCreate


class EmbedRequest(BaseModel):
    """Model for the embedding API request."""
    chunks: List[ContentChunkCreate] = Field(..., description="List of content chunks to embed")
    batch_id: Optional[str] = Field(None, description="Batch identifier for tracking")
    overwrite: bool = Field(default=False, description="Whether to overwrite existing embeddings (default: false)")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "chunks": [
                    {
                        "id": "chunk_004_11_1_001",
                        "module_id": "004-module-4-vla",
                        "week_number": 11,
                        "section_path": "week-11/chapter-1/vla-introduction",
                        "section_title": "Introduction to Vision-Language-Action Systems",
                        "content": "The vision-language-action pipeline represents the integration of perception, reasoning, and execution in robotic systems...",
                        "overlap_start": 0,
                        "overlap_end": 50,
                        "source_url": "/docs/module-4-vla/week-11/chapter-1"
                    }
                ],
                "batch_id": "batch_20251206_001",
                "overwrite": False
            }
        }


class FailedChunk(BaseModel):
    """Model for a failed chunk in the embedding response."""
    id: str = Field(..., description="Chunk ID that failed")
    error: str = Field(..., description="Error message")


class EmbedResponse(BaseModel):
    """Model for the embedding API response."""
    status: str = Field(..., description="Status of the operation (success, partial, failed)")
    batch_id: Optional[str] = Field(None, description="Batch identifier")
    processed_count: int = Field(..., description="Number of chunks processed")
    successful_count: int = Field(..., description="Number of chunks successfully embedded")
    failed_count: int = Field(..., description="Number of chunks that failed")
    failed_chunks: List[FailedChunk] = Field(default_factory=list, description="List of failed chunks")
    collection_name: str = Field(..., description="Name of Qdrant collection used")
    processing_time_ms: int = Field(..., description="Time taken to process in milliseconds")