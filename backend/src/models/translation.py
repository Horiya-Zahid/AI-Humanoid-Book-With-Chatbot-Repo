"""
Translation model for the Vision-Language-Action (VLA) & Capstone module.
This model represents translated content chunks for multilingual support.
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field
from uuid import uuid4


class TranslationBase(BaseModel):
    """Base model for translations with common fields."""
    content_chunk_id: str = Field(..., description="Reference to content chunk ID")
    language_code: str = Field(..., description="Target language code (e.g., 'ur' for Urdu)")
    translated_content: str = Field(..., description="Translated content")
    status: str = Field(default="pending", pattern=r"^(pending|reviewed|published)$", description="Translation status")


class TranslationCreate(TranslationBase):
    """Model for creating a new translation."""
    pass


class TranslationUpdate(BaseModel):
    """Model for updating a translation."""
    translated_content: Optional[str] = Field(None, description="Translated content")
    status: Optional[str] = Field(None, pattern=r"^(pending|reviewed|published)$")
    translator_notes: Optional[str] = Field(None, description="Notes from translator")


class Translation(TranslationBase):
    """Model for a complete translation with all fields."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Primary key, auto-generated")
    translator_notes: Optional[str] = Field(None, description="Notes from translator")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "trans_abc123",
                "content_chunk_id": "chunk_004_11_1_001",
                "language_code": "ur",
                "translated_content": "وژن لینگو ایکشن پائپ لائن روبوٹک سسٹم میں ادراک، منطق اور عمل کی یکجہتی کو ظاہر کرتا ہے...",
                "status": "published",
                "translator_notes": "Technical terms kept in English",
                "created_at": "2025-12-06T10:00:00Z",
                "updated_at": "2025-12-06T10:00:00Z"
            }
        }