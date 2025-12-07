"""
Embedding generation script for the Vision-Language-Action (VLA) & Capstone module.
This script processes content and generates embeddings for RAG operations.
"""
import asyncio
import os
from dotenv import load_dotenv
from typing import List
import json

from src.config.settings import settings
from src.services.embedding_service import EmbeddingService
from src.models.content_chunk import ContentChunk
from src.utils.logger import get_logger
from src.utils.helpers import chunk_text

# Load environment variables
load_dotenv()

logger = get_logger(__name__)


async def process_content_file(file_path: str, module_id: str, week_number: int, section_path: str) -> List[ContentChunk]:
    """
    Process a content file and generate content chunks with embeddings.

    Args:
        file_path: Path to the content file
        module_id: Module identifier
        week_number: Week number
        section_path: Section path

    Returns:
        List of ContentChunk objects with embeddings
    """
    logger.info(f"Processing content file: {file_path}")

    # Read the content from the file
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Initialize embedding service
    embedding_service = EmbeddingService()

    # Chunk and embed the content
    content_chunks = await embedding_service.chunk_and_embed_content(
        content=content,
        module_id=module_id,
        week_number=week_number,
        section_path=section_path,
        section_title=os.path.basename(file_path).replace('.md', '').replace('.mdx', ''),
        source_url=f"/docs/{section_path}/{os.path.basename(file_path)}",
        chunk_size=settings.CHUNK_SIZE_TOKENS,
        overlap=settings.CHUNK_OVERLAP_TOKENS
    )

    logger.info(f"Generated {len(content_chunks)} chunks for {file_path}")
    return content_chunks


async def process_module_directory(module_dir: str, module_id: str, week_number: int):
    """
    Process all content files in a module directory.

    Args:
        module_dir: Directory containing module content
        module_id: Module identifier
        week_number: Week number
    """
    logger.info(f"Processing module directory: {module_dir}")

    content_chunks = []

    # Walk through the directory to find all content files
    for root, dirs, files in os.walk(module_dir):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                file_path = os.path.join(root, file)
                section_path = os.path.relpath(root, module_dir).replace('\\', '/')

                chunks = await process_content_file(
                    file_path=file_path,
                    module_id=module_id,
                    week_number=week_number,
                    section_path=section_path
                )
                content_chunks.extend(chunks)

    logger.info(f"Module {module_id} processing complete: {len(content_chunks)} total chunks generated")
    return content_chunks


async def main():
    """
    Main function to run the embedding generation process.
    """
    logger.info("Starting embedding generation process...")

    if not settings.OPENAI_API_KEY:
        logger.error("OPENAI_API_KEY not set in environment variables")
        print("Error: OPENAI_API_KEY not set in environment variables")
        return

    try:
        # Define module information
        module_info = {
            "module_id": "004-module-4-vla",
            "week_number": 11,  # Starting week for this module
            "content_dir": "../frontend/docs/module-4-vla"  # Relative to script location
        }

        # Process the module content
        content_chunks = await process_module_directory(
            module_dir=module_info["content_dir"],
            module_id=module_info["module_id"],
            week_number=module_info["week_number"]
        )

        # In a real implementation, you would store these chunks in Qdrant
        # For this implementation, we'll just log the results
        logger.info(f"Embedding generation completed for {len(content_chunks)} content chunks")
        print(f"\nEmbedding generation completed!")
        print(f"Processed {len(content_chunks)} content chunks")
        print(f"Module: {module_info['module_id']}")
        print(f"Week: {module_info['week_number']}")
        print(f"Content directory: {module_info['content_dir']}")

        # Save chunk information to a file for reference
        output_file = f"embedding_output_{module_info['module_id']}.json"
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump([chunk.dict() for chunk in content_chunks], f, indent=2, default=str)

        logger.info(f"Chunk information saved to {output_file}")
        print(f"Chunk information saved to {output_file}")

    except Exception as e:
        logger.error(f"Error during embedding generation: {str(e)}", exc_info=True)
        print(f"\nError during embedding generation: {str(e)}")
        raise


if __name__ == "__main__":
    asyncio.run(main())