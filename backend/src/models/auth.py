"""
Authentication models for the Vision-Language-Action (VLA) & Capstone module.
This module defines the request and response models for the authentication API.
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field, EmailStr, validator


class AuthRequest(BaseModel):
    """Model for authentication requests (login)."""
    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., min_length=1, description="User's password")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "securePassword123"
            }
        }


class UserCreate(BaseModel):
    """Model for user creation requests."""
    email: EmailStr = Field(..., description="User's email address")
    name: str = Field(..., min_length=1, max_length=255, description="User's full name")
    password: str = Field(..., min_length=8, description="User's password (min 8 characters)")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "name": "John Doe",
                "password": "securePassword123"
            }
        }


class UserResponse(BaseModel):
    """Model for user response."""
    id: str = Field(..., description="User ID")
    email: EmailStr = Field(..., description="User's email")
    name: str = Field(..., description="User's name")
    created_at: Optional[datetime] = Field(None, description="Creation timestamp")

    class Config:
        """Pydantic model configuration."""
        from_attributes = True


class AuthResponse(BaseModel):
    """Model for authentication responses."""
    user: UserResponse
    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "user": {
                    "id": "user_abc123",
                    "email": "user@example.com",
                    "name": "John Doe",
                    "created_at": "2025-12-06T10:00:00Z"
                },
                "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                "token_type": "bearer"
            }
        }