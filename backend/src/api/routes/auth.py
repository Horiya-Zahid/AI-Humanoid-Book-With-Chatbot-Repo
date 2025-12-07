"""
Authentication API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements user authentication endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address
import re

from src.models.auth import AuthRequest, AuthResponse, UserCreate, UserResponse
from src.services.auth_service import AuthService
from src.config.settings import settings
from src.utils.security import verify_token
from src.utils.logger import get_logger

# Initialize router and rate limiter
router = APIRouter()
limiter = Limiter(key_func=get_remote_address)

# Get logger
logger = get_logger(__name__)


@router.post("/register", response_model=AuthResponse)
@limiter.limit("5/minute")
async def register(request: Request, user_create: UserCreate):
    """
    Register a new user account.
    """
    try:
        # Log the API call
        logger.info(
            f"Registration request received",
            extra={
                "email": user_create.email
            }
        )

        # Validate input
        if len(user_create.password) < 8:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Password must be at least 8 characters long"
            )

        if not re.match(r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$', user_create.email):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid email format"
            )

        # Initialize auth service
        auth_service = AuthService()

        # Create user
        user = await auth_service.register_user(
            email=user_create.email,
            name=user_create.name,
            password=user_create.password
        )

        # Generate tokens
        access_token = auth_service.create_access_token(data={"sub": user.id, "role": "user"})

        # Create response
        response = AuthResponse(
            user=UserResponse(
                id=user.id,
                email=user.email,
                name=user.name,
                created_at=user.created_at
            ),
            access_token=access_token,
            token_type="bearer"
        )

        logger.info(f"User registered successfully: {user.email}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error processing registration request: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during registration"
        )


@router.post("/login", response_model=AuthResponse)
@limiter.limit("10/minute")
async def login(request: Request, auth_request: AuthRequest):
    """
    Authenticate user with email and password.
    """
    try:
        # Log the API call
        logger.info(
            f"Login request received",
            extra={
                "email": auth_request.email
            }
        )

        # Initialize auth service
        auth_service = AuthService()

        # Authenticate user
        user = await auth_service.authenticate_user(
            email=auth_request.email,
            password=auth_request.password
        )

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Generate tokens
        access_token = auth_service.create_access_token(data={"sub": user.id, "role": "user"})

        # Create response
        response = AuthResponse(
            user=UserResponse(
                id=user.id,
                email=user.email,
                name=user.name,
                created_at=user.created_at
            ),
            access_token=access_token,
            token_type="bearer"
        )

        logger.info(f"User logged in successfully: {user.email}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error processing login request: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during login"
        )


@router.post("/logout")
async def logout(token_data: dict = Depends(verify_token)):
    """
    Logout the current user and invalidate tokens.
    """
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )

    # In a real implementation, you would add the token to a blacklist/jti store
    # For now, we'll just return a success message

    logger.info(f"User logged out: {token_data.get('sub')}")
    return {"message": "Successfully logged out"}


@router.get("/me", response_model=UserResponse)
async def get_current_user(token_data: dict = Depends(verify_token)):
    """
    Get information about the currently authenticated user.
    """
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # In a real implementation, you would fetch the user from the database
    # For now, we'll return a minimal user response based on the token
    user_id = token_data.get("sub")

    # This is a placeholder - in real implementation, fetch from DB
    # For now, return a minimal response
    return UserResponse(
        id=user_id,
        email="user@example.com",  # This would come from DB in real implementation
        name="Authenticated User", # This would come from DB in real implementation
        created_at=None
    )


@router.get("/health")
async def auth_health():
    """
    Health check endpoint for the auth service.
    """
    return {
        "status": "healthy",
        "service": "auth",
        "module": settings.MODULE_ID
    }