"""
Embedding service for the Vision-Language-Action (VLA) & Capstone module.
This service handles the generation and management of text embeddings.
"""
import asyncio
from typing import List, Optional, Dict, Any
from datetime import datetime
import openai
from openai import AsyncOpenAI
import logging

from src.config.settings import settings
from src.utils.logger import get_logger
from src.models.content_chunk import ContentChunk


class EmbeddingService:
    """Service class for handling text embeddings."""

    def __init__(self):
        """Initialize the embedding service with OpenAI client."""
        self.logger = get_logger(__name__)

        # Initialize OpenAI async client
        if settings.OPENAI_API_KEY:
            self.client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        else:
            self.client = None
            self.logger.warning("OpenAI API key not configured - embedding functionality will be limited")

        self.model = "text-embedding-3-small"  # Using the required model from the spec

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate an embedding for the given text using OpenAI's API.

        Args:
            text: The text to generate an embedding for

        Returns:
            List of float values representing the embedding vector
        """
        if not self.client:
            raise Exception("OpenAI client not initialized - API key required")

        try:
            # Truncate text if it's too long (OpenAI has limits)
            if len(text) > 8192:  # Common limit, adjust based on actual API constraints
                text = text[:8192]
                self.logger.warning("Text truncated for embedding generation due to length limit")

            response = await self.client.embeddings.create(
                input=text,
                model=self.model
            )

            embedding = response.data[0].embedding
            self.logger.info(f"Generated embedding for text of length {len(text)}")
            return embedding

        except Exception as e:
            self.logger.error(f"Error generating embedding: {str(e)}", exc_info=True)
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts using OpenAI's API.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors (each a list of floats)
        """
        if not self.client:
            raise Exception("OpenAI client not initialized - API key required")

        # Limit batch size according to requirements (max 100 chunks/request)
        if len(texts) > settings.EMBEDDING_BATCH_SIZE:
            raise ValueError(f"Batch size {len(texts)} exceeds maximum allowed {settings.EMBEDDING_BATCH_SIZE}")

        try:
            # Prepare the input - truncate texts if needed
            processed_texts = []
            for text in texts:
                if len(text) > 8192:
                    text = text[:8192]
                    self.logger.warning("Text truncated for embedding generation due to length limit")
                processed_texts.append(text)

            response = await self.client.embeddings.create(
                input=processed_texts,
                model=self.model
            )

            embeddings = [data.embedding for data in response.data]
            self.logger.info(f"Generated {len(embeddings)} embeddings in batch")
            return embeddings

        except Exception as e:
            self.logger.error(f"Error generating embeddings batch: {str(e)}", exc_info=True)
            raise

    async def chunk_and_embed_content(
        self,
        content: str,
        module_id: str,
        week_number: int,
        section_path: str,
        section_title: str,
        source_url: Optional[str] = None,
        chunk_size: int = 500,  # Tokens - using the requirement from the spec
        overlap: int = 100      # Tokens - using the requirement from the spec
    ) -> List[ContentChunk]:
        """
        Chunk content and generate embeddings for each chunk.

        Args:
            content: The content to chunk and embed
            module_id: The module ID for the content
            week_number: The week number for the content
            section_path: The section path for the content
            section_title: The section title for the content
            source_url: Optional source URL for the content
            chunk_size: Size of each chunk in tokens
            overlap: Overlap between chunks in tokens

        Returns:
            List of ContentChunk objects with embeddings
        """
        from src.utils.helpers import chunk_text  # Import here to avoid circular dependency

        # In a real implementation, we would use a proper tokenizer
        # For now, we'll use the helper function which approximates tokenization
        text_chunks = chunk_text(content, chunk_size, overlap)

        # Generate embeddings for all chunks
        embeddings = await self.generate_embeddings_batch(text_chunks)

        # Create ContentChunk objects
        content_chunks = []
        for i, (chunk_text, embedding) in enumerate(zip(text_chunks, embeddings)):
            chunk = ContentChunk(
                id=f"{module_id}_{week_number}_{section_path.split('/')[-1]}_{i+1:03d}",
                module_id=module_id,
                week_number=week_number,
                section_path=section_path,
                section_title=section_title,
                content=chunk_text,
                embedding=embedding,
                overlap_start=i * (chunk_size - overlap) if i > 0 else 0,
                overlap_end=min((i + 1) * chunk_size, len(content)),
                source_url=source_url,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )
            content_chunks.append(chunk)

        self.logger.info(f"Created {len(content_chunks)} content chunks with embeddings")
        return content_chunks

    async def validate_embedding_quality(self, text: str, embedding: List[float]) -> bool:
        """
        Validate the quality of an embedding (basic validation).

        Args:
            text: The original text
            embedding: The embedding vector to validate

        Returns:
            True if the embedding appears to be valid, False otherwise
        """
        # Basic validation checks
        if not embedding or len(embedding) == 0:
            return False

        # Check for NaN or infinite values
        for value in embedding:
            if not (isinstance(value, (int, float)) and value != float('inf') and value != float('-inf')):
                return False

        # In a real implementation, we might compare the embedding
        # against known good embeddings for similar content
        return True

    async def get_embedding_dimensions(self) -> int:
        """
        Get the expected dimension of embeddings from the model.

        Returns:
            The number of dimensions in the embedding vectors
        """
        # The text-embedding-3-small model produces 1536-dimensional embeddings
        # This could be verified by making a small API call if needed
        return 1536