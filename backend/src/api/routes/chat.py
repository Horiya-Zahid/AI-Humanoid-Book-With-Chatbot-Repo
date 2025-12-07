"""
Chat API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements the main chat endpoint for RAG queries.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address

from src.models.chat import ChatRequest, ChatResponse
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
async def chat(
    request: Request,
    chat_request: ChatRequest,
    token_data: Optional[dict] = Depends(verify_token)
):
    """
    Main chat endpoint for RAG queries.
    Processes user questions and returns responses based solely on embedded textbook content with proper citations.
    """
    try:
        # Log the API call
        logger.info(
            f"Chat request received",
            extra={
                "user_id": token_data.get("sub") if token_data else "anonymous",
                "module_id": chat_request.module_id,
                "query_length": len(chat_request.query)
            }
        )

        # Validate input query
        if not chat_request.query or len(chat_request.query.strip()) == 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Query cannot be empty"
            )

        if len(chat_request.query) > 1000:  # Arbitrary limit, adjust as needed
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Query too long"
            )

        # Initialize RAG service
        rag_service = RAGService()

        # Process the query through RAG system
        response = await rag_service.process_query(
            query=chat_request.query,
            module_id=chat_request.module_id,
            session_id=chat_request.session_id,
            user_id=chat_request.user_id,
            include_citations=chat_request.include_citations
        )

        # Log successful response
        logger.info(
            f"Chat response generated",
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
        logger.error(f"Error processing chat request: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your request"
        )


@router.get("/health")
async def chat_health():
    """
    Health check endpoint for the chat service.
    """
    return {
        "status": "healthy",
        "service": "chat",
        "module": settings.MODULE_ID
    }