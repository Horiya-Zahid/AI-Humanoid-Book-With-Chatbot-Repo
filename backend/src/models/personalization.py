"""
Personalization Settings model for the Vision-Language-Action (VLA) & Capstone module.
This model represents user preferences for content complexity and accessibility.
"""
from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field, validator
from uuid import uuid4


class AccessibilityFeatures(BaseModel):
    """Model for accessibility features in personalization settings."""
    high_contrast: bool = Field(default=False, description="Whether to use high contrast mode")
    larger_text: bool = Field(default=False, description="Whether to use larger text")
    screen_reader_friendly: bool = Field(default=False, description="Whether to optimize for screen readers")


class PersonalizationSettingsBase(BaseModel):
    """Base model for personalization settings with common fields."""
    user_id: str = Field(..., description="Foreign key to users table")
    module_id: str = Field(..., description="Module identifier")
    complexity_level: int = Field(..., ge=1, le=5, description="Complexity level (1-5 scale)")
    preferred_language: str = Field(default="en", description="Preferred language code")
    accessibility_features: AccessibilityFeatures = Field(default_factory=AccessibilityFeatures, description="Accessibility preferences")


class PersonalizationSettingsCreate(PersonalizationSettingsBase):
    """Model for creating new personalization settings."""
    pass


class PersonalizationSettingsUpdate(BaseModel):
    """Model for updating personalization settings."""
    complexity_level: Optional[int] = Field(None, ge=1, le=5)
    preferred_language: Optional[str] = None
    accessibility_features: Optional[AccessibilityFeatures] = None


class PersonalizationSettings(PersonalizationSettingsBase):
    """Model for complete personalization settings with all fields."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Primary key, auto-generated")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "pers_abc123",
                "user_id": "user_abc123",
                "module_id": "004-module-4-vla",
                "complexity_level": 3,
                "preferred_language": "en",
                "accessibility_features": {
                    "high_contrast": False,
                    "larger_text": True,
                    "screen_reader_friendly": True
                },
                "created_at": "2025-12-06T10:00:00Z",
                "updated_at": "2025-12-06T10:00:00Z"
            }
        }