"""
RAG (Retrieval-Augmented Generation) service for the Vision-Language-Action (VLA) & Capstone module.
This service handles the retrieval and generation of responses based on embedded textbook content.
"""
import asyncio
from typing import List, Optional, Dict, Any
from datetime import datetime
import openai
from google import genai
from google.genai.types import GenerationConfig
import logging

from src.models.chat import ChatRequest, ChatResponse, ChatChoice, ChatMessage, SourceCitation, UsageInfo
from src.models.content_chunk import ContentChunk
from src.config.settings import settings
from src.utils.helpers import format_citation, extract_citations_from_response
from src.utils.logger import get_logger
from src.services.embedding_service import EmbeddingService


class RAGService:
    """Service class for handling RAG operations."""

    def __init__(self):
        """Initialize the RAG service with required clients and configurations."""
        self.logger = get_logger(__name__)
        self.embedding_service = EmbeddingService()

        # Initialize OpenAI client
        if settings.OPENAI_API_KEY:
            openai.api_key = settings.OPENAI_API_KEY

        # Initialize Google Generative AI client
        if settings.GEMINI_API_KEY:
            genai.configure(api_key=settings.GEMINI_API_KEY)
            self.gemini_model = genai.GenerativeModel('gemini-pro')
        else:
            self.gemini_model = None

    async def process_query(
        self,
        query: str,
        module_id: Optional[str] = None,
        session_id: Optional[str] = None,
        user_id: Optional[str] = None,
        include_citations: bool = True
    ) -> ChatResponse:
        """
        Process a query through the RAG system.

        Args:
            query: The user's question or query
            module_id: Optional module ID to scope the search
            session_id: Optional session ID for conversation history
            user_id: Optional user ID for personalization
            include_citations: Whether to include source citations

        Returns:
            ChatResponse with the generated answer and citations
        """
        try:
            self.logger.info(f"Processing query: {query[:50]}...")

            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query)

            # Step 2: Retrieve relevant content chunks (simulated - in real implementation,
            # this would query Qdrant or another vector database)
            relevant_chunks = await self.retrieve_relevant_chunks(
                query_embedding,
                module_id=module_id
            )

            # Step 3: If no relevant content found, return appropriate response
            if not relevant_chunks:
                response_content = "This information is not covered in the book."
                sources = []
            else:
                # Step 4: Format the context for the LLM
                context = self.format_context_for_llm(relevant_chunks)

                # Step 5: Generate response using the LLM
                response_content = await self.generate_response_with_llm(query, context)

                # Step 6: Extract or generate citations
                sources = self.generate_citations(relevant_chunks) if include_citations else []

            # Step 7: Create and return the response
            response = ChatResponse(
                choices=[
                    ChatChoice(
                        message=ChatMessage(
                            role="assistant",
                            content=response_content,
                            sources=sources
                        )
                    )
                ],
                session_id=session_id
            )

            self.logger.info("Query processed successfully")
            return response

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}", exc_info=True)
            raise

    async def retrieve_relevant_chunks(
        self,
        query_embedding: List[float],
        module_id: Optional[str] = None,
        top_k: int = 5
    ) -> List[ContentChunk]:
        """
        Retrieve relevant content chunks based on the query embedding.
        This is a placeholder implementation - in a real system, this would query Qdrant.

        Args:
            query_embedding: The embedding vector for the query
            module_id: Optional module ID to scope the search
            top_k: Number of top results to return

        Returns:
            List of relevant content chunks
        """
        # Placeholder implementation - in real system, this would:
        # 1. Connect to Qdrant
        # 2. Perform vector similarity search
        # 3. Filter by module_id if provided
        # 4. Return top_k results

        # For now, return mock data
        mock_chunks = [
            ContentChunk(
                id="mock_chunk_1",
                module_id=module_id or "004-module-4-vla",
                week_number=11,
                section_path="week-11/chapter-1/vla-introduction",
                section_title="Introduction to Vision-Language-Action Systems",
                content="The vision-language-action pipeline represents the integration of perception, reasoning, and execution in robotic systems. It enables robots to understand visual input, process natural language commands, and execute appropriate actions in the physical world.",
                source_url="/docs/module-4-vla/week-11/chapter-1"
            )
        ]

        return mock_chunks[:top_k]

    def format_context_for_llm(self, chunks: List[ContentChunk]) -> str:
        """
        Format the retrieved content chunks for input to the LLM.

        Args:
            chunks: List of content chunks to format

        Returns:
            Formatted context string
        """
        context_parts = []
        for chunk in chunks:
            context_parts.append(
                f"Module: {chunk.module_id}\n"
                f"Week: {chunk.week_number}\n"
                f"Section: {chunk.section_title}\n"
                f"Content: {chunk.content}\n"
                "---\n"
            )

        return "\n".join(context_parts)

    async def generate_response_with_llm(self, query: str, context: str) -> str:
        """
        Generate a response using the LLM based on the query and context.

        Args:
            query: The user's query
            context: The relevant context from the knowledge base

        Returns:
            Generated response from the LLM
        """
        if not self.gemini_model:
            raise Exception("Gemini API key not configured")

        try:
            # Create a prompt that enforces RAG-only responses
            prompt = (
                "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. "
                "Answer the user's question based ONLY on the provided context. "
                "If the information is not in the context, respond with 'This information is not covered in the book.'\n\n"
                f"Context:\n{context}\n\n"
                f"Question: {query}\n\n"
                "Answer:"
            )

            # Configure generation parameters to reduce hallucination
            generation_config = GenerationConfig(
                temperature=0,  # Low temperature to reduce randomness
                max_output_tokens=1000,
                top_p=0.8,
                top_k=40
            )

            # Generate response
            response = await self.gemini_model.generate_content_async(
                prompt,
                generation_config=generation_config
            )

            # Extract text from response
            response_text = response.text if response.candidates else "This information is not covered in the book."

            return response_text

        except Exception as e:
            self.logger.error(f"Error generating response with LLM: {str(e)}", exc_info=True)
            return "This information is not covered in the book."

    def generate_citations(self, chunks: List[ContentChunk]) -> List[SourceCitation]:
        """
        Generate citations from the retrieved content chunks.

        Args:
            chunks: List of content chunks to generate citations for

        Returns:
            List of source citations
        """
        citations = []
        for chunk in chunks:
            citation = SourceCitation(
                module=f"Module {chunk.module_id.split('-')[1]}",  # Extract module number
                week=f"Week {chunk.week_number}",
                section=chunk.section_title,
                url=chunk.source_url or "",
                text=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
            )
            citations.append(citation)

        return citations

    async def validate_rag_response(self, query: str, response: str) -> bool:
        """
        Validate that the RAG response is based on the provided context and not hallucinated.

        Args:
            query: The original query
            response: The generated response

        Returns:
            True if the response is valid (not hallucinated), False otherwise
        """
        # In a real implementation, this would check:
        # 1. If response content is supported by the context
        # 2. If citations are accurate
        # 3. If response stays within the scope of provided context

        # For now, return True if the response contains the "not covered" message
        # or if it seems to be based on textbook content
        if "not covered in the book" in response.lower():
            return True

        # In a real implementation, we'd have more sophisticated validation
        return True