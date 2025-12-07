# Quickstart Guide: Module 4 – Vision-Language-Action (VLA) & Capstone

## 5-Minute Developer Setup

This guide will get you up and running with the Physical AI & Humanoid Robotics textbook project in just 5 minutes.

## Prerequisites

- Node.js v18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Git
- OpenAI API key
- Google Gemini API key
- Qdrant Cloud account (free tier)

## Step 1: Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
```

## Step 2: Install Frontend Dependencies

```bash
# Navigate to frontend directory (root of the repo)
npm install
```

## Step 3: Install Backend Dependencies

```bash
# Create Python virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install fastapi uvicorn python-dotenv openai google-generativeai qdrant-client PyJWT
```

## Step 4: Set Up Environment Variables

Create a `.env` file in the project root with the following:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Google Gemini Configuration
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000

# Frontend Configuration
REACT_APP_BACKEND_URL=http://localhost:8000
```

## Step 5: Start the Development Servers

In separate terminals:

**Terminal 1 - Start the FastAPI backend:**
```bash
cd backend  # if backend is in separate directory
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 2 - Start the Docusaurus frontend:**
```bash
npm run start
```

## Step 6: Verify Installation

1. Frontend should be available at `http://localhost:3000`
2. Backend API should be available at `http://localhost:8000`
3. Backend API docs at `http://localhost:8000/docs`
4. Textbook content should be visible at `http://localhost:3000/module-4-vla`

## Development Workflow

### Adding New Content

1. Create markdown files in `docs/` directory following the existing structure
2. Add content to be embedded in `src/content/` directory
3. Run the embedding script to generate vector representations:

```bash
python scripts/generate_embeddings.py
```

### Running Tests

**Backend tests:**
```bash
python -m pytest tests/
```

**Frontend tests:**
```bash
npm run test
```

### Building for Production

**Frontend:**
```bash
npm run build
```

**Deploy the build to GitHub Pages or serve with a static server**

## API Endpoints

### Core Endpoints

- `POST /api/chat` - Main chat endpoint for RAG queries
- `POST /api/chat/selection` - Handle text selection queries
- `POST /api/embed` - Generate embeddings (build-time only, protected)

### Bonus Endpoints

- `POST /api/auth/login` - User authentication
- `POST /api/auth/register` - User registration
- `GET /api/personalize/settings` - Get personalization settings
- `POST /api/personalize/settings` - Update personalization settings
- `GET /api/translate/:chunkId/:lang` - Get translated content

## Database Setup

### Qdrant Collections

The system will automatically create the following collections:

1. `textbook_chunks` - For content chunk embeddings
2. `modules` - For module-level embeddings

### Postgres Tables (Bonus)

If using authentication and personalization:

1. `users` - User accounts
2. `chat_sessions` - Chat session history
3. `chat_messages` - Individual chat messages
4. `personalization_settings` - User preferences
5. `translations` - Content translations

## Troubleshooting

### Common Issues

1. **Port already in use**: Change ports in `.env` and restart
2. **API keys not working**: Verify API keys in `.env` file
3. **Embeddings not generating**: Check OpenAI API key and Qdrant connection
4. **Frontend can't connect to backend**: Verify `REACT_APP_BACKEND_URL` in `.env`

### Quick Fixes

```bash
# Reset and reinstall dependencies
rm -rf node_modules
npm install
pip uninstall -r requirements.txt -y
pip install -r requirements.txt

# Clear Docusaurus cache
npm run clear

# Restart with fresh environment
source .env
npm run start
```

## Next Steps

1. Explore the existing content structure in `docs/`
2. Review the API documentation at `/docs`
3. Check the task list in `specs/004-module-4-vla-capstone/tasks.md`
4. Begin implementing the VLA capstone project features

Your development environment is now ready! You can start contributing to the Physical AI & Humanoid Robotics textbook project.