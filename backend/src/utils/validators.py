"""
Validation utilities for the Vision-Language-Action (VLA) & Capstone module.
This module provides validation functions for various data types and formats.
"""
from typing import Any, Dict, List, Optional
import re
from urllib.parse import urlparse
from datetime import datetime


def validate_email(email: str) -> bool:
    """
    Validate an email address format.

    Args:
        email: The email address to validate

    Returns:
        True if the email is valid, False otherwise
    """
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def validate_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def validate_module_id(module_id: str) -> bool:
    """
    Validate a module ID format.

    Args:
        module_id: The module ID to validate

    Returns:
        True if the module ID is valid, False otherwise
    """
    pattern = r'^\d{3}-module-\d+-[a-zA-Z-]+$'
    return re.match(pattern, module_id) is not None


def validate_week_number(week_number: int) -> bool:
    """
    Validate a week number (1-13 as per requirements).

    Args:
        week_number: The week number to validate

    Returns:
        True if the week number is valid, False otherwise
    """
    return 1 <= week_number <= 13


def validate_complexity_level(level: int) -> bool:
    """
    Validate a complexity level (1-5 scale).

    Args:
        level: The complexity level to validate

    Returns:
        True if the complexity level is valid, False otherwise
    """
    return 1 <= level <= 5


def validate_language_code(code: str) -> bool:
    """
    Validate a language code (2-3 letter ISO code).

    Args:
        code: The language code to validate

    Returns:
        True if the language code is valid, False otherwise
    """
    pattern = r'^[a-z]{2,3}$'
    return re.match(pattern, code) is not None


def validate_chunk_size(content: str, max_tokens: int = 500) -> bool:
    """
    Validate that content size is within the maximum token limit.

    Args:
        content: The content to validate
        max_tokens: The maximum allowed tokens (default 500 per requirements)

    Returns:
        True if content size is valid, False otherwise
    """
    # Simple tokenization based on words - in real implementation, use proper tokenizer
    tokens = len(content.split())
    return tokens <= max_tokens


def validate_overlap_size(overlap_start: Optional[int], overlap_end: Optional[int]) -> bool:
    """
    Validate overlap parameters for content chunks.

    Args:
        overlap_start: Start position of overlap
        overlap_end: End position of overlap

    Returns:
        True if overlap parameters are valid, False otherwise
    """
    if overlap_start is None or overlap_end is None:
        return True  # Overlap is optional

    if overlap_start < 0 or overlap_end < 0:
        return False

    if overlap_start >= overlap_end:
        return False

    return True


def validate_api_response_format(response: Dict[str, Any]) -> bool:
    """
    Validate the format of an API response.

    Args:
        response: The API response to validate

    Returns:
        True if the response format is valid, False otherwise
    """
    required_keys = {"id", "object", "created", "model", "choices", "usage"}

    if not isinstance(response, dict):
        return False

    if not required_keys.issubset(response.keys()):
        return False

    # Validate specific fields
    if not isinstance(response.get("choices"), list):
        return False

    if not isinstance(response.get("usage"), dict):
        return False

    return True


def validate_citation_format(citation: Dict[str, str]) -> bool:
    """
    Validate the format of a citation.

    Args:
        citation: The citation to validate

    Returns:
        True if the citation format is valid, False otherwise
    """
    required_keys = {"module", "week", "section", "url", "text"}

    if not isinstance(citation, dict):
        return False

    if not required_keys.issubset(citation.keys()):
        return False

    # Validate that all values are strings
    for key, value in citation.items():
        if not isinstance(value, str):
            return False

    return True


def validate_user_preferences(preferences: Dict[str, Any]) -> bool:
    """
    Validate user preferences format.

    Args:
        preferences: The user preferences to validate

    Returns:
        True if the preferences format is valid, False otherwise
    """
    if not isinstance(preferences, dict):
        return False

    # Validate specific preference keys if they exist
    if "language" in preferences and not validate_language_code(preferences["language"]):
        return False

    if "complexity" in preferences and not validate_complexity_level(preferences["complexity"]):
        return False

    return True


def validate_timestamp_format(timestamp_str: str) -> bool:
    """
    Validate that a string is in a valid timestamp format (ISO 8601).

    Args:
        timestamp_str: The timestamp string to validate

    Returns:
        True if the timestamp format is valid, False otherwise
    """
    try:
        # Try to parse the timestamp
        if timestamp_str.endswith('Z'):
            timestamp_str = timestamp_str[:-1]  # Remove 'Z' suffix for parsing
        datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
        return True
    except ValueError:
        return False


def sanitize_input(input_str: str, max_length: int = 10000) -> Optional[str]:
    """
    Sanitize user input by validating length and removing potentially dangerous characters.

    Args:
        input_str: The input string to sanitize
        max_length: Maximum allowed length (default 10000)

    Returns:
        The sanitized string if valid, None otherwise
    """
    if input_str is None:
        return None

    if len(input_str) > max_length:
        return None

    # Remove potentially dangerous characters/sequences
    sanitized = input_str.replace('<script', '&lt;script').replace('</script>', '&lt;/script&gt;')
    sanitized = sanitized.replace('javascript:', 'javascript_').replace('vbscript:', 'vbscript_')

    return sanitized


def validate_content_chunk(chunk_data: Dict[str, Any]) -> List[str]:
    """
    Validate a content chunk and return a list of validation errors.

    Args:
        chunk_data: The content chunk data to validate

    Returns:
        List of validation error messages (empty if valid)
    """
    errors = []

    # Validate required fields
    required_fields = ["id", "module_id", "week_number", "section_path", "section_title", "content"]
    for field in required_fields:
        if field not in chunk_data:
            errors.append(f"Missing required field: {field}")

    if errors:
        return errors

    # Validate specific fields
    if chunk_data.get("module_id") and not validate_module_id(chunk_data["module_id"]):
        errors.append("Invalid module_id format")

    if chunk_data.get("week_number") is not None and not validate_week_number(chunk_data["week_number"]):
        errors.append("Invalid week_number (must be 1-13)")

    if chunk_data.get("content") and not validate_chunk_size(chunk_data["content"]):
        errors.append("Content exceeds maximum token limit (500 tokens)")

    if chunk_data.get("overlap_start") is not None or chunk_data.get("overlap_end") is not None:
        if not validate_overlap_size(chunk_data.get("overlap_start"), chunk_data.get("overlap_end")):
            errors.append("Invalid overlap parameters")

    return errors