"""
Logging utilities for the Vision-Language-Action (VLA) & Capstone module.
This module provides logging setup and utilities for the application.
"""
import logging
import sys
from typing import Optional
from pythonjsonlogger import jsonlogger
from src.config.settings import settings


def setup_logging():
    """
    Set up logging configuration for the application.
    Configures both console and file logging with appropriate levels and formats.
    """
    # Clear any existing handlers
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Set the root logger level
    logging.basicConfig(level=getattr(logging, settings.LOG_LEVEL.upper()))

    # Create a custom logger
    logger = logging.getLogger(__name__)
    logger.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))

    # Create handlers
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))

    # Create formatters and add to handlers
    if settings.DEBUG:
        # Detailed formatter for development
        detailed_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s'
        )
        console_handler.setFormatter(detailed_formatter)
    else:
        # JSON formatter for production
        json_formatter = jsonlogger.JsonFormatter(
            '%(asctime)s %(name)s %(levelname)s %(filename)s %(lineno)d %(message)s'
        )
        console_handler.setFormatter(json_formatter)

    # Add handlers to the logger
    logger.addHandler(console_handler)

    # Also update the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
    root_logger.addHandler(console_handler)

    # Set specific log levels for third-party libraries
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("fastapi").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a configured logger instance with the specified name.

    Args:
        name: The name for the logger (typically __name__ of the module)

    Returns:
        A configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, settings.LOG_LEVEL.upper()))
    return logger


def log_api_call(endpoint: str, method: str, status_code: int, response_time: float):
    """
    Log API call information in a structured format.

    Args:
        endpoint: The API endpoint that was called
        method: The HTTP method used (GET, POST, etc.)
        status_code: The HTTP status code returned
        response_time: The time taken to process the request in seconds
    """
    logger = get_logger("api")
    logger.info(
        "API call completed",
        extra={
            "endpoint": endpoint,
            "method": method,
            "status_code": status_code,
            "response_time_ms": round(response_time * 1000, 2)
        }
    )


def log_error(error: Exception, context: Optional[str] = None):
    """
    Log an error with context information.

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred
    """
    logger = get_logger("error")
    logger.error(
        f"Error occurred: {str(error)}",
        extra={
            "error_type": type(error).__name__,
            "context": context or "No context provided"
        },
        exc_info=True  # Include traceback information
    )


def log_performance(metric: str, value: float, unit: str = "ms"):
    """
    Log performance metrics.

    Args:
        metric: The name of the metric being logged
        value: The value of the metric
        unit: The unit of measurement (default: ms)
    """
    logger = get_logger("performance")
    logger.info(
        f"Performance metric: {metric}",
        extra={
            "metric": metric,
            "value": value,
            "unit": unit
        }
    )