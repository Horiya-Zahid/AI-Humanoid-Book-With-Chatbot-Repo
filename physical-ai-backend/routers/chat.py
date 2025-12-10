from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import logging

# Import the RAG response function
from ..utils import get_rag_response

# Configure logging
logger = logging.getLogger(__name__)

# Create router instance
router = APIRouter(prefix="/chat", tags=["chat"])

# Pydantic models
class Message(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    messages: List[Message]
    selected_text: Optional[str] = None
    max_tokens: Optional[int] = 1000
    temperature: Optional[float] = 0.7

class Source(BaseModel):
    module: str
    week: str
    section: str
    url: str

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    model: str

# Routes
@router.post("/", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that processes user queries against the textbook content using RAG.
    """
    try:
        # Extract the last user message
        user_message = request.messages[-1].content if request.messages else ""

        # Get RAG response
        response_text, sources = get_rag_response(user_message, request.selected_text)

        # Log the interaction
        logger.info(f"Chat query: {user_message}, Response length: {len(response_text)}")

        return ChatResponse(
            response=response_text,
            sources=sources,
            model="text-embedding-ada-002"  # Placeholder model name
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")