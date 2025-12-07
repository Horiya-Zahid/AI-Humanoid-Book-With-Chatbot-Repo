"""
Helper utilities for the Vision-Language-Action (VLA) & Capstone module.
This module provides common utility functions used throughout the application.
"""
from typing import Any, Dict, List, Optional, Union
from datetime import datetime
import re
import json
from urllib.parse import urlparse
import uuid


def generate_unique_id(prefix: str = "") -> str:
    """
    Generate a unique identifier with an optional prefix.

    Args:
        prefix: Optional prefix to add to the ID

    Returns:
        A unique identifier string
    """
    unique_id = str(uuid.uuid4())
    return f"{prefix}_{unique_id}" if prefix else unique_id


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


def clean_text(text: str) -> str:
    """
    Clean and normalize text by removing extra whitespace and normalizing line breaks.

    Args:
        text: The text to clean

    Returns:
        The cleaned text
    """
    if not text:
        return ""

    # Normalize whitespace
    text = re.sub(r'\s+', ' ', text)
    # Remove leading/trailing whitespace
    text = text.strip()
    return text


def truncate_text(text: str, max_length: int, suffix: str = "...") -> str:
    """
    Truncate text to a maximum length while preserving words.

    Args:
        text: The text to truncate
        max_length: The maximum length of the text
        suffix: The suffix to add to truncated text

    Returns:
        The truncated text
    """
    if len(text) <= max_length:
        return text

    # Make room for the suffix
    available_length = max_length - len(suffix)
    if available_length <= 0:
        return suffix[:max_length]

    # Find the last space within the limit
    truncated = text[:available_length].rsplit(' ', 1)[0]
    if not truncated:
        # If no space found, truncate at the limit
        truncated = text[:available_length]

    return truncated + suffix


def count_tokens(text: str) -> int:
    """
    Count the number of tokens in a text (approximate using words).
    Note: This is a simple approximation. For more accurate token counting,
    consider using a tokenizer specific to your model (e.g., GPT tokenizer).

    Args:
        text: The text to count tokens for

    Returns:
        The approximate number of tokens
    """
    if not text:
        return 0

    # Simple tokenization based on whitespace and punctuation
    # This is an approximation - real tokenizers are more complex
    tokens = re.findall(r'\b\w+\b|[^\w\s]', text)
    return len(tokens)


def chunk_text(text: str, chunk_size: int, overlap: int = 0) -> List[str]:
    """
    Split text into chunks of specified size with optional overlap.

    Args:
        text: The text to chunk
        chunk_size: The maximum size of each chunk (in tokens)
        overlap: The number of tokens to overlap between chunks

    Returns:
        A list of text chunks
    """
    if not text:
        return []

    # For this implementation, we'll use a simple approach based on characters
    # In a real implementation, you'd want to use proper tokenization
    words = text.split()
    chunks = []

    start_idx = 0
    while start_idx < len(words):
        # Determine the end index for this chunk
        end_idx = start_idx + chunk_size

        # Create the chunk
        chunk_words = words[start_idx:end_idx]
        chunk = ' '.join(chunk_words)

        chunks.append(chunk)

        # Move the start index forward by chunk_size minus overlap
        start_idx = end_idx - overlap if overlap < chunk_size else end_idx

        # Ensure we don't get stuck in an infinite loop
        if start_idx <= start_idx:  # This shouldn't happen, but just in case
            break

    return chunks


def format_citation(module: str, week: str, section: str, url: str = "") -> str:
    """
    Format a citation according to the required format.

    Args:
        module: The module identifier
        week: The week identifier
        section: The section title
        url: Optional URL for the citation

    Returns:
        The formatted citation string
    """
    citation = f"Source: {module} → {week} → {section}"
    if url:
        citation += f" ({url})"
    return citation


def extract_citations_from_response(response: str) -> List[Dict[str, str]]:
    """
    Extract citations from a response string in the format:
    "Source: Module X → Week Y → Section Z"

    Args:
        response: The response string to extract citations from

    Returns:
        A list of citation dictionaries
    """
    citations = []
    # Pattern to match the citation format
    pattern = r"Source:\s*([^→]+)\s*→\s*([^→]+)\s*→\s*([^(]+)(?:\s*\(([^)]+)\))?"
    matches = re.findall(pattern, response)

    for match in matches:
        citation = {
            "module": match[0].strip(),
            "week": match[1].strip(),
            "section": match[2].strip(),
        }
        if len(match) > 3 and match[3]:  # URL is optional
            citation["url"] = match[3].strip()
        citations.append(citation)

    return citations


def safe_json_loads(json_str: str, default: Any = None) -> Any:
    """
    Safely load JSON from a string, returning a default value if parsing fails.

    Args:
        json_str: The JSON string to parse
        default: The default value to return if parsing fails

    Returns:
        The parsed JSON object or the default value
    """
    try:
        return json.loads(json_str)
    except (json.JSONDecodeError, TypeError):
        return default


def dict_to_camel_case(data: Union[Dict, List]) -> Union[Dict, List]:
    """
    Convert dictionary keys from snake_case to camelCase recursively.

    Args:
        data: The dictionary or list of dictionaries to convert

    Returns:
        The converted data structure with camelCase keys
    """
    if isinstance(data, dict):
        converted = {}
        for key, value in data.items():
            # Convert snake_case to camelCase
            camel_key = re.sub(r'_([a-z])', lambda m: m.group(1).upper(), key)
            # Recursively convert nested structures
            converted[camel_key] = dict_to_camel_case(value) if isinstance(value, (dict, list)) else value
        return converted
    elif isinstance(data, list):
        return [dict_to_camel_case(item) if isinstance(item, (dict, list)) else item for item in data]
    else:
        return data


def dict_to_snake_case(data: Union[Dict, List]) -> Union[Dict, List]:
    """
    Convert dictionary keys from camelCase to snake_case recursively.

    Args:
        data: The dictionary or list of dictionaries to convert

    Returns:
        The converted data structure with snake_case keys
    """
    if isinstance(data, dict):
        converted = {}
        for key, value in data.items():
            # Convert camelCase to snake_case
            snake_key = re.sub(r'([A-Z])', r'_\1', key).lower()
            if snake_key.startswith('_'):
                snake_key = snake_key[1:]  # Remove leading underscore if present
            # Recursively convert nested structures
            converted[snake_key] = dict_to_snake_case(value) if isinstance(value, (dict, list)) else value
        return converted
    elif isinstance(data, list):
        return [dict_to_snake_case(item) if isinstance(item, (dict, list)) else item for item in data]
    else:
        return data


def validate_module_week_section(module_id: str, week_number: int, section_path: str) -> bool:
    """
    Validate module, week, and section identifiers.

    Args:
        module_id: The module identifier to validate
        week_number: The week number to validate
        section_path: The section path to validate

    Returns:
        True if all identifiers are valid, False otherwise
    """
    # Validate module ID format
    module_pattern = r'^\d{3}-module-\d+-[a-zA-Z-]+$'
    if not re.match(module_pattern, module_id):
        return False

    # Validate week number
    if not (1 <= week_number <= 13):
        return False

    # Validate section path (should not be empty and should be a valid path)
    if not section_path or not re.match(r'^[a-zA-Z0-9/_-]+$', section_path):
        return False

    return True


def calculate_similarity_score(text1: str, text2: str) -> float:
    """
    Calculate a similarity score between two texts (0.0 to 1.0).
    This is a simple implementation using a basic algorithm.
    For production use, consider using more sophisticated algorithms like cosine similarity.

    Args:
        text1: First text to compare
        text2: Second text to compare

    Returns:
        A similarity score between 0.0 and 1.0
    """
    if not text1 or not text2:
        return 0.0

    # Convert to lowercase and split into words
    words1 = set(text1.lower().split())
    words2 = set(text2.lower().split())

    # Calculate intersection and union
    intersection = words1.intersection(words2)
    union = words1.union(words2)

    # Jaccard similarity
    if not union:
        return 1.0  # Both texts are empty

    return len(intersection) / len(union)


def format_timestamp(dt: datetime) -> str:
    """
    Format a datetime object as an ISO 8601 string.

    Args:
        dt: The datetime object to format

    Returns:
        The formatted timestamp string
    """
    return dt.isoformat() + "Z"  # Add Z to indicate UTC


def parse_timestamp(timestamp_str: str) -> Optional[datetime]:
    """
    Parse an ISO 8601 timestamp string to a datetime object.

    Args:
        timestamp_str: The timestamp string to parse

    Returns:
        The parsed datetime object or None if parsing fails
    """
    try:
        # Remove the 'Z' suffix if present and parse
        if timestamp_str.endswith('Z'):
            timestamp_str = timestamp_str[:-1]
        return datetime.fromisoformat(timestamp_str)
    except ValueError:
        return None


def mask_sensitive_info(text: str, sensitive_keywords: List[str] = None) -> str:
    """
    Mask sensitive information in text based on keywords.

    Args:
        text: The text to mask
        sensitive_keywords: List of keywords to mask (defaults to common sensitive terms)

    Returns:
        The text with sensitive information masked
    """
    if sensitive_keywords is None:
        sensitive_keywords = [
            "password", "secret", "key", "token", "api_key", "jwt",
            "authorization", "auth", "credential", "private", "client_id", "client_secret"
        ]

    masked_text = text
    for keyword in sensitive_keywords:
        # Case-insensitive replacement
        pattern = re.compile(re.escape(keyword), re.IGNORECASE)
        masked_text = pattern.sub("***REDACTED***", masked_text)

    return masked_text