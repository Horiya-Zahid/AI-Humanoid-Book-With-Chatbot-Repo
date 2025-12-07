from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="Backend API for the Physical AI & Humanoid Robotics textbook RAG system",
    version="0.1.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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

class EmbeddingRequest(BaseModel):
    text: str
    module_id: str
    week_number: int
    section_path: str
    section_title: str

class EmbeddingResponse(BaseModel):
    success: bool
    chunk_id: str
    tokens: int

# Mock data for sources
MOCK_SOURCES = [
    Source(module="Module 1", week="Week 1-5", section="The Robotic Nervous System (ROS 2)", url="/docs/module-1-robotic-nervous-system/week-1"),
    Source(module="Module 2", week="Week 6-7", section="The Digital Twin (Gazebo & Unity)", url="/docs/module-2-digital-twin/week-6"),
    Source(module="Module 3", week="Week 8-10", section="The AI-Robot Brain (NVIDIA Isaac™ Platform)", url="/docs/module-3-isaac-brain/week-8"),
    Source(module="Module 4", week="Week 11-13", section="Vision-Language-Action Capstone (VLA)", url="/docs/module-4-vla-capstone/week-11"),
]

# Mock RAG response function
def get_rag_response(user_query: str, selected_text: Optional[str] = None) -> tuple[str, List[Source]]:
    """
    Mock function to simulate RAG response from the textbook content.
    In a real implementation, this would query the vector database.
    """
    user_query_lower = user_query.lower()

    # Determine relevant sources based on query
    relevant_sources = []
    if "ros" in user_query_lower or "robot" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "ROS" in s.section or "robot" in user_query_lower]
    elif "isaac" in user_query_lower or "nvidia" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "Isaac" in s.section]
    elif "vla" in user_query_lower or "vision-language" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "VLA" in s.section]
    elif "simulation" in user_query_lower or "gazebo" in user_query_lower or "unity" in user_query_lower:
        relevant_sources = [s for s in MOCK_SOURCES if "Digital Twin" in s.section]
    else:
        relevant_sources = MOCK_SOURCES[:2]  # Default to first two sources

    if not relevant_sources:
        relevant_sources = MOCK_SOURCES

    # Generate response based on query
    if "ros" in user_query_lower or "robot" in user_query_lower:
        response = (
            f"Based on the Physical AI & Humanoid Robotics textbook, "
            f"ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. "
            f"It provides a collection of tools, libraries, and conventions that aim to simplify "
            f"creating complex and robust robot behavior across a wide variety of robot platforms. "
            f"{f'This specifically relates to the selected text: {selected_text}' if selected_text else ''}"
        )
    elif "isaac" in user_query_lower or "nvidia" in user_query_lower:
        response = (
            f"The NVIDIA Isaac™ platform is a comprehensive solution for developing, simulating, "
            f"and deploying AI-powered robots. It combines hardware (Jetson platforms), software "
            f"frameworks, and simulation tools to accelerate robotics development. "
            f"{f'As mentioned in the selected text: {selected_text}' if selected_text else ''}"
        )
    elif "vla" in user_query_lower or "vision-language" in user_query_lower:
        response = (
            f"Vision-Language-Action (VLA) systems represent the next generation of AI-powered "
            f"robots that can perceive the world (Vision), understand and reason about it (Language), "
            f"and take appropriate actions (Action) in a unified framework. "
            f"{f'The selected text "{selected_text}" is relevant to this concept' if selected_text else ''}"
        )
    else:
        response = (
            f"I can help you with information from the Physical AI & Humanoid Robotics textbook. "
            f"This includes topics on ROS 2, Digital Twins, NVIDIA Isaac Platform, and Vision-Language-Action systems. "
            f"{f'Based on your selected text: {selected_text}' if selected_text else 'What would you like to know more about?'}"
        )

    return response, relevant_sources

# Routes
@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics RAG API", "status": "running"}

@app.post("/chat", response_model=ChatResponse)
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

@app.post("/embed", response_model=EmbeddingResponse)
async def embed_content(request: EmbeddingRequest):
    """
    Embed content into the vector database.
    In a real implementation, this would generate embeddings and store them in Qdrant.
    """
    try:
        # Calculate token count (simplified estimation)
        tokens = len(request.text.split())

        # Generate a mock chunk ID
        import uuid
        chunk_id = f"chunk_{request.module_id}_{request.week_number}_{uuid.uuid4().hex[:8]}"

        # In a real implementation, this would:
        # 1. Chunk the text (500 tokens with 100 overlap)
        # 2. Generate embeddings using a model like sentence-transformers
        # 3. Store in Qdrant with metadata

        logger.info(f"Embedded content for {request.module_id}, week {request.week_number}, tokens: {tokens}")

        return EmbeddingResponse(
            success=True,
            chunk_id=chunk_id,
            tokens=tokens
        )
    except Exception as e:
        logger.error(f"Error in embed endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "service": "Physical AI RAG API"}

@app.get("/modules")
async def get_modules():
    """
    Get list of available textbook modules.
    """
    modules = [
        {"id": "module-1", "name": "The Robotic Nervous System (ROS 2)", "weeks": "1-5"},
        {"id": "module-2", "name": "The Digital Twin (Gazebo & Unity)", "weeks": "6-7"},
        {"id": "module-3", "name": "The AI-Robot Brain (NVIDIA Isaac™ Platform)", "weeks": "8-10"},
        {"id": "module-4", "name": "Vision-Language-Action Capstone (VLA)", "weeks": "11-13"},
    ]
    return {"modules": modules}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)