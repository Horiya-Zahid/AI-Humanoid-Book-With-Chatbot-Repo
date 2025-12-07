"""
Personalization API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements user personalization endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address

from src.models.personalize import PersonalizeSettingsRequest, PersonalizeSettingsResponse, GetPersonalizeResponse
from src.services.personalization_service import PersonalizationService
from src.config.settings import settings
from src.utils.security import verify_token
from src.utils.logger import get_logger

# Initialize router and rate limiter
router = APIRouter()
limiter = Limiter(key_func=get_remote_address)

# Get logger
logger = get_logger(__name__)


@router.get("/", response_model=GetPersonalizeResponse)
@limiter.limit("30/minute")
async def get_settings(
    request: Request,
    module_id: Optional[str] = None,
    token_data: dict = Depends(verify_token)
):
    """
    Get the current user's personalization settings for a specific module or all modules.
    """
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        # Log the API call
        logger.info(
            f"Get personalization settings request",
            extra={
                "user_id": token_data.get("sub"),
                "module_id": module_id
            }
        )

        # Initialize personalization service
        personalize_service = PersonalizationService()

        # Get settings
        settings_list = await personalize_service.get_settings(
            user_id=token_data.get("sub"),
            module_id=module_id
        )

        # Create response
        response = GetPersonalizeResponse(settings=settings_list)

        logger.info(f"Personalization settings retrieved for user: {token_data.get('sub')}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error getting personalization settings: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving settings"
        )


@router.post("/", response_model=PersonalizeSettingsResponse)
@limiter.limit("10/minute")
async def update_settings(
    request: Request,
    personalize_request: PersonalizeSettingsRequest,
    token_data: dict = Depends(verify_token)
):
    """
    Update the user's personalization settings for a specific module.
    """
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        # Log the API call
        logger.info(
            f"Update personalization settings request",
            extra={
                "user_id": token_data.get("sub"),
                "module_id": personalize_request.module_id
            }
        )

        # Initialize personalization service
        personalize_service = PersonalizationService()

        # Update settings
        updated_setting = await personalize_service.update_settings(
            user_id=token_data.get("sub"),
            module_id=personalize_request.module_id,
            complexity_level=personalize_request.complexity_level,
            preferred_language=personalize_request.preferred_language,
            accessibility_features=personalize_request.accessibility_features
        )

        # Create response
        response = PersonalizeSettingsResponse(
            message="Settings updated successfully",
            settings=updated_setting
        )

        logger.info(f"Personalization settings updated for user: {token_data.get('sub')}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error updating personalization settings: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while updating settings"
        )


@router.get("/content")
@limiter.limit("20/minute")
async def get_personalized_content(
    request: Request,
    module_id: str,
    section_path: str,
    complexity_adjustment: Optional[int] = None,
    token_data: Optional[dict] = Depends(verify_token)
):
    """
    Get personalized content based on user's settings and preferences.
    """
    user_id = token_data.get("sub") if token_data else None

    try:
        # Log the API call
        logger.info(
            f"Get personalized content request",
            extra={
                "user_id": user_id,
                "module_id": module_id,
                "section_path": section_path
            }
        )

        # Initialize personalization service
        personalize_service = PersonalizationService()

        # Get personalized content
        personalized_content = await personalize_service.get_personalized_content(
            module_id=module_id,
            section_path=section_path,
            user_id=user_id,
            complexity_adjustment=complexity_adjustment
        )

        logger.info(f"Personalized content retrieved for module: {module_id}")
        return personalized_content

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error getting personalized content: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving personalized content"
        )


@router.get("/health")
async def personalize_health():
    """
    Health check endpoint for the personalize service.
    """
    return {
        "status": "healthy",
        "service": "personalize",
        "module": settings.MODULE_ID
    }