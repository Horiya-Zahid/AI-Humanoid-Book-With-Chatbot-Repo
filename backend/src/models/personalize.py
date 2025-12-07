"""
Personalization models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request and response models for the personalization API.
"""
from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field

from src.models.personalization import AccessibilityFeatures


class PersonalizeSettingsRequest(BaseModel):
    """Model for updating personalization settings."""
    module_id: str = Field(..., description="Module identifier")
    complexity_level: Optional[int] = Field(None, ge=1, le=5, description="Complexity level (1-5)")
    preferred_language: Optional[str] = Field(None, description="Preferred language code")
    accessibility_features: Optional[AccessibilityFeatures] = Field(None, description="Accessibility preferences")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "module_id": "004-module-4-vla",
                "complexity_level": 3,
                "preferred_language": "en",
                "accessibility_features": {
                    "high_contrast": False,
                    "larger_text": True,
                    "screen_reader_friendly": True
                }
            }
        }


class PersonalizeSettingResponse(BaseModel):
    """Model for a single personalization setting."""
    module_id: str = Field(..., description="Module identifier")
    complexity_level: int = Field(..., ge=1, le=5, description="Complexity level (1-5)")
    preferred_language: str = Field(default="en", description="Preferred language code")
    accessibility_features: AccessibilityFeatures = Field(default_factory=AccessibilityFeatures, description="Accessibility preferences")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class PersonalizeSettingsResponse(BaseModel):
    """Model for the personalization settings update response."""
    message: str = Field(default="Settings updated successfully", description="Response message")
    settings: PersonalizeSettingResponse = Field(..., description="Updated settings")


class GetPersonalizeResponse(BaseModel):
    """Model for getting personalization settings."""
    settings: List[PersonalizeSettingResponse] = Field(..., description="List of settings")


class PersonalizedContentResponse(BaseModel):
    """Model for personalized content response."""
    content: Dict[str, Any] = Field(..., description="Personalized content")
    metadata: Dict[str, Any] = Field(..., description="Metadata about the personalization")


class GetPersonalizedContentResponse(BaseModel):
    """Model for getting personalized content."""
    content: str = Field(..., description="Original content")
    personalized_content: str = Field(..., description="Personalized content based on settings")
    complexity_level: int = Field(..., description="Applied complexity level")
    language: str = Field(..., description="Applied language")
    metadata: Dict[str, Any] = Field(..., description="Additional metadata")