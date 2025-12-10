from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import logging
from dotenv import load_dotenv

# Import routers
from routers.chat import router as chat_router
from routers.embeddings import router as embeddings_router

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

# Include routers
app.include_router(chat_router)
app.include_router(embeddings_router)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics RAG API", "status": "running"}

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