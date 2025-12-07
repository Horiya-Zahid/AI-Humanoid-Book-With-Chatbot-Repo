"""
User model for the Vision-Language-Action (VLA) & Capstone module.
This model represents a user in the system with authentication and preferences.
"""
from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field, EmailStr, validator
from uuid import UUID, uuid4


class UserBase(BaseModel):
    """Base model for users with common fields."""
    email: EmailStr = Field(..., description="User email address")
    name: str = Field(..., min_length=1, max_length=255, description="User's full name")


class UserCreate(UserBase):
    """Model for creating a new user."""
    password: str = Field(..., min_length=8, description="User's password (min 8 characters)")


class UserUpdate(BaseModel):
    """Model for updating a user."""
    name: Optional[str] = Field(None, min_length=1, max_length=255)
    preferences: Optional[Dict[str, Any]] = Field(None, description="User preferences including language, accessibility settings")


class UserInDBBase(UserBase):
    """Base model for user with database fields."""
    id: str = Field(default_factory=lambda: str(uuid4()), description="Primary key, auto-generated")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Account creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")
    email_verified: bool = Field(default=False, description="Whether email is verified")
    preferences: Dict[str, Any] = Field(default_factory=dict, description="User preferences including language, accessibility settings")


class User(UserInDBBase):
    """Model for a complete user with all fields."""
    class Config:
        """Pydantic model configuration."""
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "user_abc123",
                "email": "user@example.com",
                "name": "John Doe",
                "created_at": "2025-12-06T10:00:00Z",
                "updated_at": "2025-12-06T10:00:00Z",
                "email_verified": False,
                "preferences": {
                    "language": "en",
                    "accessibility": {
                        "high_contrast": False,
                        "larger_text": True
                    }
                }
            }
        }


class UserInDB(UserInDBBase):
    """Model for user in database with hashed password."""
    hashed_password: str

    class Config:
        """Pydantic model configuration."""
        from_attributes = True