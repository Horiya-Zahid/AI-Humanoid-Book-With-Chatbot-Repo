"""
Text Selection API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements the text selection query endpoint.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address

from src.models.selection import SelectionRequest, ChatResponse
from src.services.rag_service import RAGService
from src.config.settings import settings
from src.utils.security import verify_token
from src.utils.logger import get_logger

# Initialize router and rate limiter
router = APIRouter()
limiter = Limiter(key_func=get_remote_address)

# Get logger
logger = get_logger(__name__)


@router.post("/", response_model=ChatResponse)
@limiter.limit("10/minute")
async def selection_chat(
    request: Request,
    selection_request: SelectionRequest,
    token_data: Optional[dict] = Depends(verify_token)
):
    """
    Text selection chat endpoint.
    Handles queries based on user-selected text with context.
    """
    try:
        # Log the API call
        logger.info(
            f"Selection chat request received",
            extra={
                "user_id": token_data.get("sub") if token_data else "anonymous",
                "module_id": selection_request.module_id,
                "selected_text_length": len(selection_request.selected_text)
            }
        )

        # Validate selected text
        if not selection_request.selected_text or len(selection_request.selected_text.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selected text cannot be empty"
            )

        if len(selection_request.selected_text) > 1000:  # Arbitrary limit, adjust as needed
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selected text too long"
            )

        # Combine selected text with context if available
        query = selection_request.selected_text
        if selection_request.original_context:
            query = f"Regarding this text: '{selection_request.selected_text}'. {selection_request.original_context}"

        # Initialize RAG service
        rag_service = RAGService()

        # Process the query through RAG system
        response = await rag_service.process_query(
            query=query,
            module_id=selection_request.module_id,
            session_id=selection_request.session_id,
            user_id=selection_request.user_id,
            include_citations=True  # Always include citations for selection queries
        )

        # Log successful response
        logger.info(
            f"Selection chat response generated",
            extra={
                "response_length": len(response.choices[0].message.content) if response.choices else 0,
                "has_citations": len(response.choices[0].message.sources) > 0 if response.choices else False
            }
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error processing selection chat request: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your request"
        )


@router.get("/health")
async def selection_health():
    """
    Health check endpoint for the selection service.
    """
    return {
        "status": "healthy",
        "service": "selection",
        "module": settings.MODULE_ID
    }