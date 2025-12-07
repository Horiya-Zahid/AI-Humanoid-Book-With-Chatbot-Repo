"""
Configuration settings for the Vision-Language-Action (VLA) & Capstone module.
This module contains all the configuration settings for the application.
"""
import os
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings class."""

    # API Configuration
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI & Humanoid Robotics - VLA Module"

    # Backend Configuration
    BACKEND_HOST: str = os.getenv("BACKEND_HOST", "0.0.0.0")
    BACKEND_PORT: int = int(os.getenv("BACKEND_PORT", "8000"))

    # Frontend Configuration
    FRONTEND_URL: str = os.getenv("FRONTEND_URL", "http://localhost:3000")

    # CORS Configuration
    BACKEND_CORS_ORIGINS: str = os.getenv("BACKEND_CORS_ORIGINS", "*")

    # Database Configuration
    DATABASE_URL: Optional[str] = os.getenv("DATABASE_URL")
    NEON_DB_URL: Optional[str] = os.getenv("NEON_DB_URL")

    # Qdrant Configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_HOST: str = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", "6333"))

    # OpenAI Configuration
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")

    # Google Gemini Configuration
    GEMINI_API_KEY: Optional[str] = os.getenv("GEMINI_API_KEY")

    # Authentication Configuration
    JWT_SECRET: str = os.getenv("JWT_SECRET", "dev-secret-key-change-in-production")
    JWT_ALGORITHM: str = os.getenv("JWT_ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # Application Configuration
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    # RAG Configuration
    RAG_RESPONSE_TIME_LIMIT: float = float(os.getenv("RAG_RESPONSE_TIME_LIMIT", "3.0"))  # seconds
    VECTOR_SEARCH_TIME_LIMIT: float = float(os.getenv("VECTOR_SEARCH_TIME_LIMIT", "0.5"))  # seconds
    MAX_CONCURRENT_USERS: int = int(os.getenv("MAX_CONCURRENT_USERS", "100"))
    CHUNK_SIZE_TOKENS: int = int(os.getenv("CHUNK_SIZE_TOKENS", "500"))
    CHUNK_OVERLAP_TOKENS: int = int(os.getenv("CHUNK_OVERLAP_TOKENS", "100"))

    # Embedding Configuration
    EMBEDDING_BATCH_SIZE: int = int(os.getenv("EMBEDDING_BATCH_SIZE", "100"))

    # Module Configuration
    MODULE_ID: str = "004-module-4-vla"
    SYLLABUS_WEEKS: int = 13

    # Citation Format
    CITATION_FORMAT: str = "Source: Module X → Week Y → Section Z"

    class Config:
        """Pydantic settings configuration."""
        env_file = ".env"
        case_sensitive = True


# Create a global settings instance
settings = Settings()