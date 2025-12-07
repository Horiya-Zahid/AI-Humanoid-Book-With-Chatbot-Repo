"""
Security utilities for the Vision-Language-Action (VLA) & Capstone module.
This module provides security-related functions and middleware for the application.
"""
from datetime import datetime, timedelta
from typing import Optional, Union
import jwt
import hashlib
import secrets
from passlib.context import CryptContext
from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.security.api_key import APIKeyHeader
import re

from src.config.settings import settings
from src.models.user import UserInDB


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT Bearer scheme for authentication
security = HTTPBearer(auto_error=False)

# API Key header for protected endpoints
api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.

    Args:
        plain_password: The plain text password to verify
        hashed_password: The hashed password to compare against

    Returns:
        True if the password matches, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """
    Generate a hash for the given password.

    Args:
        password: The plain text password to hash

    Returns:
        The hashed password
    """
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token with the given data and expiration.

    Args:
        data: The data to encode in the token
        expires_delta: Optional expiration time delta (defaults to settings value)

    Returns:
        The encoded JWT token
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.JWT_SECRET, algorithm=settings.JWT_ALGORITHM)
    return encoded_jwt


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decode a JWT token and return the payload.

    Args:
        token: The JWT token to decode

    Returns:
        The decoded payload if valid, None otherwise
    """
    try:
        payload = jwt.decode(token, settings.JWT_SECRET, algorithms=[settings.JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        # Token has expired
        return None
    except jwt.JWTError:
        # Invalid token
        return None


def verify_token(credentials: Optional[HTTPAuthorizationCredentials] = None) -> Optional[dict]:
    """
    Verify the JWT token from the request credentials.

    Args:
        credentials: The HTTP authorization credentials from the request

    Returns:
        The decoded token payload if valid, None otherwise
    """
    if not credentials or not credentials.credentials:
        return None

    return decode_access_token(credentials.credentials)


def get_current_user(token: str = security) -> Optional[UserInDB]:
    """
    Get the current user from the JWT token in the request.

    Args:
        token: The JWT token from the Authorization header

    Returns:
        The UserInDB object if the token is valid and user exists, None otherwise
    """
    if not token:
        return None

    payload = decode_access_token(token.credentials)
    if not payload:
        return None

    user_id: str = payload.get("sub")
    if not user_id:
        return None

    # In a real implementation, you would fetch the user from the database
    # For now, we return None to indicate that the user lookup is not implemented
    # The actual implementation would require database access which is not set up yet
    return None


def hash_data(data: str) -> str:
    """
    Create a SHA-256 hash of the given data.

    Args:
        data: The data to hash

    Returns:
        The SHA-256 hash of the data
    """
    return hashlib.sha256(data.encode()).hexdigest()


def generate_api_key() -> str:
    """
    Generate a secure API key.

    Returns:
        A randomly generated API key
    """
    return secrets.token_urlsafe(32)


def validate_email(email: str) -> bool:
    """
    Validate an email address format.

    Args:
        email: The email address to validate

    Returns:
        True if the email is valid, False otherwise
    """
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def sanitize_input(input_str: str) -> str:
    """
    Sanitize user input to prevent injection attacks.

    Args:
        input_str: The input string to sanitize

    Returns:
        The sanitized input string
    """
    # Remove potentially dangerous characters
    sanitized = input_str.replace('<', '&lt;').replace('>', '&gt;')
    sanitized = sanitized.replace('"', '&quot;').replace("'", '&#x27;')
    sanitized = sanitized.replace('/', '&#x2F;')
    return sanitized


def rate_limit_key(identifier: str, endpoint: str) -> str:
    """
    Generate a key for rate limiting based on identifier and endpoint.

    Args:
        identifier: The identifier (e.g., user ID or IP address)
        endpoint: The API endpoint

    Returns:
        A key for rate limiting
    """
    return f"rate_limit:{identifier}:{endpoint}"


def is_valid_module_id(module_id: str) -> bool:
    """
    Validate a module ID format.

    Args:
        module_id: The module ID to validate

    Returns:
        True if the module ID is valid, False otherwise
    """
    pattern = r'^\d{3}-module-\d+-[a-zA-Z-]+$'
    return re.match(pattern, module_id) is not None


def encrypt_sensitive_data(data: str) -> str:
    """
    Encrypt sensitive data (placeholder implementation).
    In a real application, you would use a proper encryption library like cryptography.

    Args:
        data: The sensitive data to encrypt

    Returns:
        The encrypted data (placeholder)
    """
    # This is a placeholder - in production, use proper encryption
    # For now, just return a hash as a simple obfuscation
    return hashlib.sha256(data.encode()).hexdigest()


def decrypt_sensitive_data(encrypted_data: str) -> str:
    """
    Decrypt sensitive data (placeholder implementation).
    In a real application, you would use a proper encryption library like cryptography.

    Args:
        encrypted_data: The encrypted data to decrypt

    Returns:
        The decrypted data (placeholder)
    """
    # This is a placeholder - in production, use proper decryption
    # For now, return the encrypted data as we don't have real encryption
    return encrypted_data