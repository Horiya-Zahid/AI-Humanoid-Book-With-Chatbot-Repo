"""
Embedding API routes for the Vision-Language-Action (VLA) & Capstone module.
This module implements the embedding generation endpoint.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from typing import Optional
from slowapi import Limiter
from slowapi.util import get_remote_address

from src.models.embed import EmbedRequest, EmbedResponse
from src.services.embedding_service import EmbeddingService
from src.config.settings import settings
from src.utils.security import verify_token, verify
from src.utils.logger import get_logger

# Initialize router and rate limiter
router = APIRouter()
limiter = Limiter(key_func=get_remote_address)

# Get logger
logger = get_logger(__name__)


@router.post("/", response_model=EmbedResponse)
@limiter.limit("5/minute")  # Limit embedding generation due to cost/API constraints
async def generate_embeddings(
    request: Request,
    embed_request: EmbedRequest,
    token_data: Optional[dict] = Depends(verify_token)
):
    """
    Embedding generation endpoint.
    Build-time only endpoint for generating content embeddings.
    """
    try:
        # Log the API call
        logger.info(
            f"Embedding generation request received",
            extra={
                "user_id": token_data.get("sub") if token_data else "anonymous",
                "batch_id": embed_request.batch_id,
                "chunk_count": len(embed_request.chunks) if embed_request.chunks else 0
            }
        )

        # Verify this is a build-time request (in a real implementation, you'd have more robust verification)
        # For now, we'll check if the user has appropriate permissions
        if not token_data or token_data.get("role") != "build":
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="This endpoint is restricted to build-time operations only"
            )

        # Validate request
        if not embed_request.chunks:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Chunks list cannot be empty"
            )

        if len(embed_request.chunks) > settings.EMBEDDING_BATCH_SIZE:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Batch size {len(embed_request.chunks)} exceeds maximum allowed {settings.EMBEDDING_BATCH_SIZE}"
            )

        # Initialize embedding service
        embedding_service = EmbeddingService()

        # Process each chunk to generate embeddings
        processed_count = 0
        successful_count = 0
        failed_chunks = []
        collection_name = "textbook_chunks"  # Default collection

        for chunk_data in embed_request.chunks:
            try:
                # Generate embedding for the chunk content
                embedding = await embedding_service.generate_embedding(chunk_data.content)

                # In a real implementation, you would store the embedding in Qdrant
                # For now, we'll just simulate the process

                processed_count += 1
                successful_count += 1

            except Exception as e:
                failed_chunks.append({
                    "id": chunk_data.id,
                    "error": str(e)
                })
                logger.error(f"Failed to embed chunk {chunk_data.id}: {str(e)}")

        # Create response
        response = EmbedResponse(
            status="success",
            batch_id=embed_request.batch_id,
            processed_count=processed_count,
            successful_count=successful_count,
            failed_count=len(failed_chunks),
            failed_chunks=failed_chunks,
            collection_name=collection_name,
            processing_time_ms=100  # Placeholder
        )

        # Log successful response
        logger.info(
            f"Embedding generation completed",
            extra={
                "batch_id": embed_request.batch_id,
                "processed_count": processed_count,
                "successful_count": successful_count,
                "failed_count": len(failed_chunks)
            }
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        logger.error(f"Error processing embedding request: {str(e)}", exc_info=True)

        # Return appropriate error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while generating embeddings"
        )


@router.get("/health")
async def embed_health():
    """
    Health check endpoint for the embed service.
    """
    return {
        "status": "healthy",
        "service": "embed",
        "module": settings.MODULE_ID
    }