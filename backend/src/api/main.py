"""
Main FastAPI application for the Vision-Language-Action (VLA) & Capstone module.
This is the entry point for the backend API.
"""
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import logging
import os
from typing import Generator, List

from src.config.settings import settings
from src.api.routes import chat, selection, embed, auth, personalize, translate
from src.utils.logger import setup_logging


@asynccontextmanager
async def lifespan(app: FastAPI) -> Generator:
    """
    FastAPI lifespan event handler for startup and shutdown events.
    """
    # Startup
    setup_logging()
    logging.info("Starting up Physical AI & Humanoid Robotics API")

    # Initialize services here if needed
    # await initialize_services()

    yield

    # Shutdown
    logging.info("Shutting down Physical AI & Humanoid Robotics API")


# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Create FastAPI app instance
app = FastAPI(
    title=settings.PROJECT_NAME,
    version="1.0.0",
    description="API for the Vision-Language-Action (VLA) & Capstone module of the Physical AI & Humanoid Robotics textbook",
    lifespan=lifespan
)

# Add security middleware
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.BACKEND_CORS_ORIGINS.split(",") if settings.BACKEND_CORS_ORIGINS != "*" else ["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose custom headers
    expose_headers=["Access-Control-Allow-Origin"]
)

# Add TrustedHost middleware for security
if settings.DEBUG:
    # In development, allow all hosts for testing
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["*"])
else:
    # In production, restrict to specific hosts
    allowed_hosts = [settings.BACKEND_HOST, "localhost", "127.0.0.1", ".yourdomain.com"]
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=allowed_hosts)


# Include API routes
app.include_router(chat.router, prefix=settings.API_V1_STR, tags=["chat"])
app.include_router(selection.router, prefix=settings.API_V1_STR, tags=["selection"])
app.include_router(embed.router, prefix=settings.API_V1_STR, tags=["embed"])
app.include_router(auth.router, prefix=settings.API_V1_STR, tags=["auth"])
app.include_router(personalize.router, prefix=settings.API_V1_STR, tags=["personalize"])
app.include_router(translate.router, prefix=settings.API_V1_STR, tags=["translate"])


@app.get("/")
async def root():
    """
    Root endpoint to verify API is running.
    """
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics API",
        "version": "1.0.0",
        "module": "Vision-Language-Action (VLA) & Capstone"
    }


@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify API health.
    """
    return {
        "status": "healthy",
        "module": "Vision-Language-Action (VLA) & Capstone",
        "api_version": "1.0.0"
    }


@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """
    Global exception handler for the API.
    """
    logging.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "type": "internal_server_error",
                "message": "An unexpected error occurred"
            }
        }
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=settings.BACKEND_HOST,
        port=settings.BACKEND_PORT,
        reload=True if settings.DEBUG else False,
        log_level=settings.LOG_LEVEL.lower()
    )