"""
Translation API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements content translation endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request, Path
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address

from src.models.translate import TranslateBatchRequest, TranslateBatchResponse, TranslationStatusResponse
from src.services.translation_service import TranslationService
from src.config.settings import settings
from src.utils.security import verify_token
from src.utils.logger import get_logger

# Initialize router and rate limiter
router = APIRouter()
limiter = Limiter(key_func=get_remote_address)

# Get logger
logger = get_logger(__name__)


@router.get("/{chunk_id}/{lang}")
@limiter.limit("20/minute")
async def get_translation(
    request: Request,
    chunk_id: str = Path(..., description="The ID of the content chunk to translate"),
    lang: str = Path(..., description="The target language code (e.g., 'ur' for Urdu, 'en' for English)"),
    fallback: Optional[bool] = True,
    token_data: Optional[dict] = Depends(verify_token)
):
    """
    Get a translated version of a specific content chunk in the requested language.
    """
    try:
        # Log the API call
        logger.info(
            f"Get translation request",
            extra={
                "user_id": token_data.get("sub") if token_data else "anonymous",
                "chunk_id": chunk_id,
                "target_language": lang
            }
        )

        # Initialize translation service
        translation_service = TranslationService()

        # Get translation
        translation = await translation_service.get_translation(
            chunk_id=chunk_id,
            target_language=lang,
            fallback=fallback
        )

        if not translation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Translation not available for the specified chunk and language"
            )

        logger.info(f"Translation retrieved for chunk: {chunk_id}, language: {lang}")
        return translation

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error getting translation for chunk {chunk_id}: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving the translation"
        )


@router.post("/batch", response_model=TranslateBatchResponse)
@limiter.limit("5/minute")  # Limit due to potential cost of translation
async def request_batch_translation(
    request: Request,
    batch_request: TranslateBatchRequest,
    token_data: dict = Depends(verify_token)
):
    """
    Request translation of multiple content chunks at once.
    This is primarily a build-time or admin endpoint.
    """
    if not token_data or token_data.get("role") != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin authentication required for batch translation requests"
        )

    try:
        # Log the API call
        logger.info(
            f"Batch translation request",
            extra={
                "user_id": token_data.get("sub"),
                "requester_id": batch_request.requester_id,
                "total_chunks": len(batch_request.chunks) if batch_request.chunks else 0
            }
        )

        # Initialize translation service
        translation_service = TranslationService()

        # Request batch translation
        batch_response = await translation_service.request_batch_translation(
            chunks=batch_request.chunks,
            source_language=batch_request.source_language,
            requester_id=token_data.get("sub"),  # Use authenticated user ID
            priority=batch_request.priority,
            callback_url=batch_request.callback_url
        )

        logger.info(f"Batch translation requested: {batch_response.request_id}")
        return batch_response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error requesting batch translation: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while requesting batch translation"
        )


@router.get("/status/{request_id}", response_model=TranslationStatusResponse)
async def get_translation_status(
    request_id: str = Path(..., description="The ID of the batch translation request"),
    token_data: dict = Depends(verify_token)
):
    """
    Check the status of a batch translation request.
    """
    if not token_data or token_data.get("role") != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin authentication required for translation status requests"
        )

    try:
        # Log the API call
        logger.info(
            f"Get translation status request",
            extra={
                "user_id": token_data.get("sub"),
                "request_id": request_id
            }
        )

        # Initialize translation service
        translation_service = TranslationService()

        # Get status
        status_response = await translation_service.get_translation_status(
            request_id=request_id
        )

        if not status_response:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Translation request not found"
            )

        logger.info(f"Translation status retrieved for request: {request_id}")
        return status_response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error getting translation status for request {request_id}: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving translation status"
        )


@router.get("/health")
async def translate_health():
    """
    Health check endpoint for the translate service.
    """
    return {
        "status": "healthy",
        "service": "translate",
        "module": settings.MODULE_ID
    }