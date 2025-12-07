"""
Authentication service for the Vision-Language-Action (VLA) & Capstone module.
This service handles user authentication and management.
"""
import asyncio
from typing import Optional
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext

from src.models.user import UserCreate, UserInDB
from src.models.auth import UserResponse
from src.config.settings import settings
from src.utils.security import get_password_hash, verify_password, create_access_token
from src.utils.logger import get_logger


class AuthService:
    """Service class for handling authentication operations."""

    def __init__(self):
        """Initialize the auth service."""
        self.logger = get_logger(__name__)
        self.pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

    async def register_user(self, email: str, name: str, password: str) -> UserInDB:
        """
        Register a new user.

        Args:
            email: User's email address
            name: User's full name
            password: User's password

        Returns:
            UserInDB object representing the created user
        """
        try:
            self.logger.info(f"Registering new user: {email}")

            # Hash the password
            hashed_password = get_password_hash(password)

            # Create user object (in a real implementation, this would be saved to DB)
            user = UserInDB(
                email=email,
                name=name,
                hashed_password=hashed_password
            )

            self.logger.info(f"User registered successfully: {email}")
            return user

        except Exception as e:
            self.logger.error(f"Error registering user {email}: {str(e)}", exc_info=True)
            raise

    async def authenticate_user(self, email: str, password: str) -> Optional[UserInDB]:
        """
        Authenticate a user with email and password.

        Args:
            email: User's email address
            password: User's password

        Returns:
            UserInDB object if authentication is successful, None otherwise
        """
        try:
            self.logger.info(f"Authenticating user: {email}")

            # In a real implementation, this would fetch the user from the database
            # For this implementation, we'll return a mock user for testing purposes
            # In a real system, you'd look up the user by email in the database
            # and verify the password against the stored hash

            # Mock user lookup (in real implementation, query database)
            # This is just for demonstration - in production, implement proper DB lookup
            mock_user = UserInDB(
                id="mock_user_123",
                email=email,
                name="Test User",
                hashed_password=get_password_hash(password),  # This is just for demo
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                email_verified=True
            )

            # Verify password (in real implementation, use stored hash from DB)
            if verify_password(password, mock_user.hashed_password):
                self.logger.info(f"User authenticated successfully: {email}")
                return mock_user
            else:
                self.logger.warning(f"Failed authentication attempt for user: {email}")
                return None

        except Exception as e:
            self.logger.error(f"Error authenticating user {email}: {str(e)}", exc_info=True)
            return None

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create a JWT access token.

        Args:
            data: The data to encode in the token
            expires_delta: Optional expiration time delta

        Returns:
            The encoded JWT token
        """
        return create_access_token(data, expires_delta)

    async def get_user_by_email(self, email: str) -> Optional[UserInDB]:
        """
        Get a user by email address.

        Args:
            email: The email address to search for

        Returns:
            UserInDB object if found, None otherwise
        """
        try:
            self.logger.info(f"Looking up user by email: {email}")

            # In a real implementation, this would query the database
            # For this implementation, we'll return None to indicate not found
            # (since we don't have a real database setup yet)
            return None

        except Exception as e:
            self.logger.error(f"Error looking up user by email {email}: {str(e)}", exc_info=True)
            return None

    async def update_user_preferences(self, user_id: str, preferences: dict) -> bool:
        """
        Update user preferences.

        Args:
            user_id: The ID of the user to update
            preferences: The preferences to update

        Returns:
            True if update was successful, False otherwise
        """
        try:
            self.logger.info(f"Updating preferences for user: {user_id}")

            # In a real implementation, this would update the user in the database
            # For this implementation, we'll just return True to indicate success
            return True

        except Exception as e:
            self.logger.error(f"Error updating preferences for user {user_id}: {str(e)}", exc_info=True)
            return False

    async def verify_token(self, token: str) -> Optional[UserResponse]:
        """
        Verify an authentication token and return user information.

        Args:
            token: The JWT token to verify

        Returns:
            UserResponse if token is valid, None otherwise
        """
        try:
            payload = jwt.decode(token, settings.JWT_SECRET, algorithms=[settings.JWT_ALGORITHM])
            user_id: str = payload.get("sub")
            if user_id is None:
                return None

            # In a real implementation, you would fetch the user from the database
            # For this implementation, return a minimal user response
            return UserResponse(
                id=user_id,
                email="user@example.com",  # This would come from DB in real implementation
                name="Authenticated User", # This would come from DB in real implementation
                created_at=None
            )
        except jwt.ExpiredSignatureError:
            return None
        except jwt.JWTError:
            return None