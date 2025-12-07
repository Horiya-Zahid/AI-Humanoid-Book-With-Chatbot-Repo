#!/usr/bin/env python3
"""
Embedding Generation Script for Physical AI & Humanoid Robotics Textbook

This script processes the textbook content, chunks it into 500-token pieces with 100-token overlap,
generates embeddings, and stores them with proper metadata in a vector database.

Features:
- 500-token chunks with 100-token overlap
- Batch processing (100 chunks at a time)
- Metadata preservation (module, week, section, path)
- Progress tracking and error handling
"""

import os
import sys
import argparse
import asyncio
import aiohttp
import logging
from pathlib import Path
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass
from tqdm import tqdm
import tiktoken
from sentence_transformers import SentenceTransformer
import json

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('embedding_generation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class Chunk:
    id: str
    module_id: str
    week_number: int
    section_path: str
    section_title: str
    content: str
    tokens: int
    overlap_start: int
    overlap_end: int
    source_url: str

class EmbeddingGenerator:
    def __init__(self,
                 chunk_size_tokens: int = 500,
                 overlap_tokens: int = 100,
                 batch_size: int = 100,
                 embedding_model: str = "all-MiniLM-L6-v2"):
        self.chunk_size_tokens = chunk_size_tokens
        self.overlap_tokens = overlap_tokens
        self.batch_size = batch_size
        self.enc = tiktoken.get_encoding("cl100k_base")  # Good for most models
        self.model = SentenceTransformer(f'sentence-transformers/{embedding_model}')
        self.backend_url = os.getenv("BACKEND_URL", "http://localhost:8000")

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken."""
        return len(self.enc.encode(text))

    def chunk_text(self, text: str, module_id: str, week_number: int,
                   section_path: str, section_title: str, source_url: str) -> List[Chunk]:
        """Chunk text into 500-token pieces with 100-token overlap."""
        tokens = self.enc.encode(text)
        chunks = []
        chunk_id = 0

        start_idx = 0
        while start_idx < len(tokens):
            # Determine end index for this chunk
            end_idx = min(start_idx + self.chunk_size_tokens, len(tokens))

            # Extract tokens for this chunk
            chunk_tokens = tokens[start_idx:end_idx]

            # Decode back to text
            chunk_text = self.enc.decode(chunk_tokens)

            # Create chunk object
            chunk = Chunk(
                id=f"{module_id}_{week_number}_{section_path.replace('/', '_')}_{chunk_id:03d}",
                module_id=module_id,
                week_number=week_number,
                section_path=section_path,
                section_title=section_title,
                content=chunk_text,
                tokens=len(chunk_tokens),
                overlap_start=start_idx,
                overlap_end=end_idx,
                source_url=source_url
            )

            chunks.append(chunk)
            chunk_id += 1

            # Move start index by chunk_size - overlap to create overlap
            start_idx = end_idx - self.overlap_tokens

            # Ensure we don't get stuck in an infinite loop
            if start_idx >= len(tokens):
                break
            if start_idx <= start_idx:  # Prevent infinite loop
                start_idx += 1

        return chunks

    async def embed_chunks(self, chunks: List[Chunk]) -> List[Dict[str, Any]]:
        """Generate embeddings for a list of chunks."""
        # Extract texts for batch processing
        texts = [chunk.content for chunk in chunks]

        # Generate embeddings
        embeddings = self.model.encode(texts)

        # Prepare results with metadata
        results = []
        for i, chunk in enumerate(chunks):
            result = {
                "id": chunk.id,
                "module_id": chunk.module_id,
                "week_number": chunk.week_number,
                "section_path": chunk.section_path,
                "section_title": chunk.section_title,
                "content": chunk.content,
                "tokens": chunk.tokens,
                "overlap_start": chunk.overlap_start,
                "overlap_end": chunk.overlap_end,
                "source_url": chunk.source_url,
                "embedding": embeddings[i].tolist()  # Convert numpy array to list
            }
            results.append(result)

        return results

    async def upload_to_backend(self, chunk_data: Dict[str, Any]) -> bool:
        """Upload a single chunk to the backend API."""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.backend_url}/embed",
                    json=chunk_data,
                    headers={"Content-Type": "application/json"}
                ) as response:
                    if response.status == 200:
                        return True
                    else:
                        logger.error(f"Failed to upload chunk {chunk_data['id']}: {response.status}")
                        return False
        except Exception as e:
            logger.error(f"Error uploading chunk {chunk_data['id']}: {str(e)}")
            return False

    async def process_file(self, file_path: Path, module_id: str, week_number: int,
                          section_path: str, section_title: str) -> List[Dict[str, Any]]:
        """Process a single markdown file and return embedded chunks."""
        logger.info(f"Processing file: {file_path}")

        # Read file content
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Create source URL based on file path
        source_url = f"/docs/{section_path}/{file_path.name}"

        # Chunk the content
        chunks = self.chunk_text(
            content, module_id, week_number, section_path, section_title, source_url
        )

        logger.info(f"Created {len(chunks)} chunks from {file_path}")

        # Embed chunks
        embedded_chunks = await self.embed_chunks(chunks)

        return embedded_chunks

    async def process_directory(self, docs_dir: Path) -> None:
        """Process all markdown files in the docs directory."""
        # Find all markdown files
        md_files = list(docs_dir.rglob("*.md"))
        logger.info(f"Found {len(md_files)} markdown files to process")

        # Process each file
        for md_file in tqdm(md_files, desc="Processing files"):
            # Determine module and week from path
            path_parts = md_file.relative_to(docs_dir).parts

            # Extract module info
            module_id = "unknown"
            week_number = 0
            section_path = ""
            section_title = md_file.stem

            if len(path_parts) >= 2:
                if path_parts[0].startswith("module-"):
                    module_id = path_parts[0]
                    # Extract week number from filename if present
                    if path_parts[1].startswith("week-"):
                        try:
                            week_number = int(path_parts[1].split("-")[1])
                        except ValueError:
                            week_number = 0
                    section_path = "/".join(path_parts[:-1])

            # Process the file
            embedded_chunks = await self.process_file(
                md_file, module_id, week_number, section_path, section_title
            )

            # Upload in batches
            for i in range(0, len(embedded_chunks), self.batch_size):
                batch = embedded_chunks[i:i + self.batch_size]

                # Upload each chunk in the batch
                for chunk_data in tqdm(batch, desc="Uploading batch", leave=False):
                    success = await self.upload_to_backend(chunk_data)
                    if not success:
                        logger.error(f"Failed to upload chunk: {chunk_data['id']}")

                logger.info(f"Uploaded batch {i//self.batch_size + 1}/{(len(embedded_chunks)-1)//self.batch_size + 1}")

    async def run(self, docs_path: str):
        """Main execution method."""
        docs_dir = Path(docs_path)

        if not docs_dir.exists():
            logger.error(f"Docs directory does not exist: {docs_dir}")
            return

        logger.info(f"Starting embedding generation for: {docs_dir}")

        # Process the directory
        await self.process_directory(docs_dir)

        logger.info("Embedding generation completed!")

def main():
    parser = argparse.ArgumentParser(description="Generate embeddings for Physical AI textbook")
    parser.add_argument(
        "--docs-path",
        type=str,
        default="../my-website/docs",
        help="Path to the docs directory (default: ../my-website/docs)"
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=500,
        help="Chunk size in tokens (default: 500)"
    )
    parser.add_argument(
        "--overlap",
        type=int,
        default=100,
        help="Overlap size in tokens (default: 100)"
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=100,
        help="Batch size for processing (default: 100)"
    )

    args = parser.parse_args()

    generator = EmbeddingGenerator(
        chunk_size_tokens=args.chunk_size,
        overlap_tokens=args.overlap,
        batch_size=args.batch_size
    )

    # Run the async function
    asyncio.run(generator.run(args.docs_path))

if __name__ == "__main__":
    main()