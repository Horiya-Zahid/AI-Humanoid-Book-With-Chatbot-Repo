"""
Content Chunk model for the Vision-Language-Action (VLA) & Capstone module.
This model represents a chunk of content that has been embedded for RAG operations.
"""
from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class ContentChunkBase(BaseModel):
    """Base model for content chunks with common fields."""
    module_id: str = Field(..., description="Module identifier (e.g., '004-module-4-vla')")
    week_number: int = Field(..., ge=1, le=13, description="Week number in the 13-week syllabus")
    section_path: str = Field(..., description="Path to the specific section")
    section_title: str = Field(..., description="Title of the section")
    content: str = Field(..., max_length=500, description="The chunked content (max 500 tokens)")
    overlap_start: Optional[int] = Field(None, description="Start position of overlap in original content")
    overlap_end: Optional[int] = Field(None, description="End position of overlap in original content")
    source_url: Optional[str] = Field(None, description="URL reference to the original content")


class ContentChunkCreate(ContentChunkBase):
    """Model for creating a new content chunk."""
    pass


class ContentChunkUpdate(BaseModel):
    """Model for updating a content chunk."""
    week_number: Optional[int] = Field(None, ge=1, le=13)
    section_title: Optional[str] = None
    content: Optional[str] = Field(None, max_length=500)
    overlap_start: Optional[int] = None
    overlap_end: Optional[int] = None
    source_url: Optional[str] = None


class ContentChunk(ContentChunkBase):
    """Model for a complete content chunk with ID and metadata."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique identifier for the chunk")
    embedding: Optional[List[float]] = Field(None, description="OpenAI embedding vector")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp of creation")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp of last update")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "chunk_004_11_1_001",
                "module_id": "004-module-4-vla",
                "week_number": 11,
                "section_path": "week-11/chapter-1/vla-introduction",
                "section_title": "Introduction to Vision-Language-Action Systems",
                "content": "The vision-language-action pipeline represents the integration of perception, reasoning, and execution in robotic systems...",
                "overlap_start": 0,
                "overlap_end": 50,
                "source_url": "/docs/module-4-vla/week-11/chapter-1",
                "created_at": "2025-12-06T10:00:00Z",
                "updated_at": "2025-12-06T10:00:00Z"
            }
        }