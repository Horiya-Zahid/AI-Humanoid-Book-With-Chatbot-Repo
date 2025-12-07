# Physical AI & Humanoid Robotics Backend

Backend API for the Physical AI & Humanoid Robotics textbook, providing RAG (Retrieval-Augmented Generation) functionality.

## Features

- FastAPI-based REST API
- RAG functionality for textbook content
- Vector database integration with Qdrant
- Text embedding and chunking
- Source citation system
- Text selection support for contextual queries

## Requirements

- Python 3.8+
- Poetry (recommended) or pip

## Installation

### Using Poetry (recommended)

```bash
# Install dependencies
poetry install

# Activate virtual environment
poetry shell

# Run the server
poetry run uvicorn main:app --reload
```

### Using pip

```bash
# Install dependencies
pip install -r requirements.txt

# Run the server
uvicorn main:app --reload
```

## Configuration

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Then update the values in `.env` with your actual configuration.

## API Endpoints

- `GET /` - Root endpoint
- `POST /chat` - Chat with RAG system
- `POST /embed` - Embed content into vector database
- `GET /health` - Health check
- `GET /modules` - Get available modules

## Running the Server

```bash
# Development mode with auto-reload
uvicorn main:app --reload

# Production mode
uvicorn main:app --host 0.0.0.0 --port 8000
```

## Environment Variables

- `API_HOST`: Host address (default: 0.0.0.0)
- `API_PORT`: Port number (default: 8000)
- `QDRANT_URL`: Qdrant vector database URL
- `QDRANT_API_KEY`: Qdrant API key
- `OPENAI_API_KEY`: OpenAI API key (optional)

## License

MIT