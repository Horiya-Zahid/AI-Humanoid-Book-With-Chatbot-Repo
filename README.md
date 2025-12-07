# Physical AI & Humanoid Robotics – AI-Native Interactive Textbook

This repository contains the Vision-Language-Action (VLA) & Capstone module (Module 4) of the Physical AI & Humanoid Robotics textbook project.

## Overview

The Physical AI & Humanoid Robotics textbook is an AI-native, interactive educational platform featuring:

- **Module 4: Vision-Language-Action (VLA) & Capstone** - The culminating module that unites vision, language, and action components
- Students build a simulated humanoid that accepts natural-language voice commands and executes them end-to-end using Whisper → LLM planner → ROS 2 actions
- Purple + neon theme implementation for consistent branding
- Strict RAG-only Q&A system with proper citations
- 100% embedded content with text-selection query mode
- Bonus features: live voice-to-action demo and Urdu translation

## Architecture

The system follows a hybrid static + serverless architecture:

```
┌─────────────────┐    HTTP/API    ┌──────────────────┐
│   Frontend      │ ─────────────▶ │   Backend API    │
│  (GitHub Pages) │                │  (Vercel/Fly.io) │
│               ◀─┼────────────────┼─────────┐        │
│  Docusaurus 3.x │    SSE/HTTP    │ FastAPI │        │
└─────────────────┤                │ Python  │        │
                  │                └─────────┼────────┘
                  │                          │
                  │                ┌─────────▼────────┐
                  │                │  Qdrant Cloud    │
                  │                │  (Vector DB)     │
                  │                └──────────────────┘
                  │                          │
                  │                ┌─────────▼────────┐
                  │                │ Neon Postgres    │
                  │                │  (User Data)     │
                  └────────────────►                  │
                                   └──────────────────┘
```

## Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x
- **Language**: React/TypeScript
- **Styling**: CSS Modules with custom purple/neon theme
- **UI Components**: Custom components and ChatKit SDK
- **Deployment**: GitHub Pages

### Backend
- **Framework**: FastAPI
- **Language**: Python 3.11+
- **Database**: Neon Serverless Postgres (user data), Qdrant Cloud (vector embeddings)
- **AI Services**: OpenAI (embeddings), Google Gemini (generation)
- **Deployment**: Vercel/Fly.io (serverless)

### Key Technologies
- **Embeddings**: OpenAI text-embedding-3-small
- **Vector DB**: Qdrant Cloud Free Tier
- **LLM**: Google Gemini 1.5 Flash/Pro
- **Chat UI**: ChatKit SDK
- **Auth (Bonus)**: Better-Auth
- **DB (Bonus)**: Neon Serverless Postgres

## Features

### Core Features
- **RAG-only Q&A**: Answers exclusively from pre-generated OpenAI embeddings stored in Qdrant Cloud
- **Text Selection Query**: Users can select text and ask questions about the selected content
- **Proper Citations**: All answers include "Source: Module X → Week Y → Section Z" format
- **WCAG 2.1 AA Compliance**: Accessible to all users
- **Mobile Responsive**: Works on all device sizes
- **Performance Optimized**: <2s page load on 3G, <3s RAG response time

### VLA-Specific Features
- **Voice-to-Action Pipeline**: Complete implementation of vision-language-action systems
- **Cognitive Planning**: LLM-based task breakdown and action execution
- **Capstone Project**: Complete walkthrough of autonomous humanoid project
- **VLA Visualization**: Interactive pipeline visualization

### Bonus Features
- **Live Voice Demo**: WebRTC + Whisper.cpp for browser-based voice processing
- **Urdu Translation**: Full RTL support with Urdu translations
- **User Authentication**: Better-Auth integration
- **Personalization**: Complexity adjustment and user preferences

## Performance Requirements

- Page load < 2.0s on 3G (Lighthouse ≥ 95)
- RAG response time < 3s (95th percentile)
- Vector search < 500ms
- Full static build < 5 minutes
- Embedding generation: batch max 100 chunks/request

## Repository Structure

```
├── backend/                    # FastAPI backend
│   ├── src/
│   │   ├── models/            # Pydantic models
│   │   ├── services/          # Business logic
│   │   ├── api/              # API routes
│   │   ├── utils/            # Utility functions
│   │   └── config/           # Configuration
│   ├── requirements.txt
│   └── README.md
├── frontend/                   # Docusaurus frontend
│   ├── src/
│   │   ├── components/        # React components
│   │   ├── pages/            # Page components
│   │   └── services/         # API service functions
│   ├── docs/                 # Textbook content
│   │   ├── module-4-vla/     # Module 4 content
│   │   └── capstone-project/ # Capstone content
│   ├── docusaurus.config.js
│   └── README.md
├── scripts/                    # Deployment and utility scripts
├── specs/                      # Specification files
│   └── 004-module-4-vla-capstone/
│       ├── spec.md            # Feature specification
│       ├── plan.md            # Implementation plan
│       ├── research.md        # Technology research
│       ├── data-model.md      # Data models
│       ├── quickstart.md      # Quickstart guide
│       ├── contracts/         # API contracts
│       └── tasks.md           # Implementation tasks
└── .env.example               # Environment variables template
```

## Getting Started

### Prerequisites

- Node.js 18+
- Python 3.11+
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/physical-ai-textbook.git
   cd physical-ai-textbook
   ```

2. Set up the backend:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up the frontend:
   ```bash
   cd ../frontend
   npm install
   ```

4. Configure environment variables:
   ```bash
   # Copy the example file and fill in your API keys
   cp .env.example .env
   # Edit .env with your actual API keys and configuration
   ```

### Running Locally

1. Start the backend:
   ```bash
   cd backend
   uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
   ```

2. In a new terminal, start the frontend:
   ```bash
   cd frontend
   npm start
   ```

The frontend will be available at `http://localhost:3000` and the backend API at `http://localhost:8000`.

## API Documentation

The backend API provides the following endpoints:

- `POST /api/v1/chat` - Main RAG query endpoint
- `POST /api/v1/chat/selection` - Text selection query endpoint
- `POST /api/v1/embed` - Embedding generation endpoint (build-time only)
- `POST /api/v1/auth/*` - Authentication endpoints
- `POST /api/v1/personalize` - Personalization endpoints
- `GET /api/v1/translate/*` - Translation endpoints

## Content Structure

Module 4 content is organized in the `frontend/docs/module-4-vla/` directory:

- **Week 11**: Introduction to VLA systems
- **Week 12**: Voice-to-action processing concepts
- **Week 13**: Capstone project walkthrough

Each week contains MDX files with interactive content, diagrams, and examples.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Run tests to ensure everything works
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## Deployment

The application is designed for deployment with:

- **Frontend**: GitHub Pages (static Docusaurus build)
- **Backend**: Vercel or Fly.io (serverless FastAPI)

Deployment scripts are available in the `scripts/` directory.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Powered by [FastAPI](https://fastapi.tiangolo.com/)
- Vector search by [Qdrant](https://qdrant.tech/)
- AI services by OpenAI and Google