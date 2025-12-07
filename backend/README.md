# Physical AI & Humanoid Robotics - VLA Backend

This is the backend API for the Vision-Language-Action (VLA) & Capstone module of the Physical AI & Humanoid Robotics textbook project.

## Overview

The backend provides:
- RAG (Retrieval-Augmented Generation) functionality for textbook Q&A
- Content embedding and storage
- User authentication and personalization
- Multi-language support with Urdu translation
- Voice-to-action processing capabilities

## Tech Stack

- **Framework**: FastAPI
- **Language**: Python 3.11+
- **Database**: Neon Serverless Postgres (user data), Qdrant Cloud (vector embeddings)
- **AI Services**: OpenAI (embeddings), Google Gemini (generation)

## Installation

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Set up environment variables (copy `.env.example` to `.env` and fill in values)

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key
- `GEMINI_API_KEY`: Your Google Gemini API key
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DB_URL`: Your Neon Postgres connection string
- `JWT_SECRET`: Secret key for JWT tokens
- `BACKEND_HOST`: Host for the backend server (default: 0.0.0.0)
- `BACKEND_PORT`: Port for the backend server (default: 8000)

## Running the Server

```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

- `POST /api/v1/chat` - Main RAG query endpoint
- `POST /api/v1/chat/selection` - Text selection query endpoint
- `POST /api/v1/embed` - Embedding generation endpoint (build-time only)
- `POST /api/v1/auth/register` - User registration
- `POST /api/v1/auth/login` - User login
- `GET /api/v1/auth/me` - Get current user
- `POST /api/v1/personalize` - Update personalization settings
- `GET /api/v1/translate/{chunk_id}/{lang}` - Get translated content
- `POST /api/v1/translate/batch` - Request batch translation

## Architecture

The backend follows a service-oriented architecture with:

- **Models**: Pydantic models for data validation
- **Services**: Business logic encapsulation
- **API Routes**: FastAPI route handlers
- **Utils**: Helper functions and utilities
- **Config**: Application settings and configuration

## Key Features

### RAG System
- Strict RAG-only responses (no hallucination)
- Proper citation format: "Source: Module X → Week Y → Section Z"
- Vector similarity search with Qdrant
- Pre-generated embeddings for performance

### Security
- JWT-based authentication
- Rate limiting
- CORS configuration
- Input validation and sanitization

### Personalization
- Complexity level adjustment (1-5 scale)
- Language preferences
- Accessibility features

### Multilingual Support
- Urdu translation with RTL support
- Translation status tracking
- Fallback mechanisms

## Testing

Run the tests with:
```bash
pytest
```

## Deployment

The backend is designed for serverless deployment on platforms like Vercel or Fly.io. For production deployment:

1. Set appropriate environment variables
2. Ensure database connections are properly configured
3. Set up SSL certificates
4. Configure rate limiting appropriately

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.