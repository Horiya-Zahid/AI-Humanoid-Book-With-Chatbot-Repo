# Physical AI & Humanoid Robotics Textbook

[![Deploy to GitHub Pages](https://github.com/your-org/physical-ai-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/your-org/physical-ai-textbook/actions/workflows/deploy.yml)

A comprehensive Docusaurus-based textbook on Physical AI, ROS 2, Digital Twins, NVIDIA Isaac Platform, and Vision-Language-Action systems with integrated RAG-powered chatbot.

## 🚀 Features

- **Complete 13-week curriculum** across 4 modules
- **Purple/Neon theme** with engaging visual design
- **RAG-powered chatbot** for instant answers to textbook questions
- **Text selection support** for contextual queries
- **Interactive diagrams** and visualizations
- **Self-check quizzes** in each section
- **GPU-accelerated processing** with NVIDIA Isaac integration
- **ROS 2 communication infrastructure**

## 📚 Modules

1. **Module 1**: The Robotic Nervous System (ROS 2) - Weeks 1-5
2. **Module 2**: The Digital Twin (Gazebo & Unity) - Weeks 6-7
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac™ Platform) - Weeks 8-10
4. **Module 4**: Vision-Language-Action Capstone (VLA) - Weeks 11-13

## 🏗️ Architecture

```
┌─────────────────┐    ┌──────────────────┐
│   Frontend      │    │    Backend       │
│  (Docusaurus)   │◄──►│   (FastAPI)      │
│                 │    │                  │
│ • Textbook UI   │    │ • RAG API        │
│ • Chatbot       │    │ • Embedding Gen  │
│ • Diagrams      │    │ • Qdrant Vector  │
│ • Search        │    │   DB             │
└─────────────────┘    └──────────────────┘
```

## 🛠️ One-Click Local Development

### Prerequisites
- Node.js 20+
- Python 3.8+
- npm or yarn

### Quick Start

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install frontend dependencies
cd my-website
npm install

# Install backend dependencies
cd ../physical-ai-backend
pip install -r requirements.txt

# Create environment file
cp .env.example .env
# Edit .env with your configuration

# Start backend server
uvicorn main:app --reload

# In a new terminal, start frontend
cd ../my-website
npm start
```

### Backend Setup (Optional)
```bash
# Navigate to backend
cd physical-ai-backend

# Install with Poetry (recommended)
poetry install
poetry shell

# Or with pip
pip install -r requirements.txt

# Run the server
uvicorn main:app --reload
```

### Generate Embeddings
```bash
# Run the embedding generation script
cd my-website
npm run embed
```

## 🔧 Development Commands

```bash
# Start development server
npm start

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy

# Generate embeddings
npm run embed

# Type checking
npm run typecheck

# Clear cache
npm run clear
```

## 🤖 RAG Chatbot Usage

The integrated chatbot allows you to:

1. **Ask questions** about textbook content directly
2. **Select text** and ask contextual questions
3. **Get source citations** for all answers
4. **Access quick questions** for common topics

The chatbot appears as a floating button in the bottom-right corner of each page.

## 📊 Content Structure

```
docs/
├── intro.md                    # Introduction to the textbook
├── module-1-robotic-nervous-system/
│   ├── week-1.md              # Introduction to Physical AI
│   ├── week-2.md              # ROS 2 Architecture
│   ├── week-3.md              # rclpy Python API
│   ├── week-4.md              # URDF/Xacro for Humanoids
│   └── week-5.md              # Launch Systems
├── module-2-digital-twin/
│   ├── week-6.md              # Digital Twins in Robotics
│   └── week-7.md              # Gazebo/Unity Integration
├── module-3-isaac-brain/
│   ├── week-8.md              # NVIDIA Isaac Platform
│   ├── week-9.md              # Stereo Visual SLAM
│   └── week-10.md             # Navigation & Manipulation
└── module-4-vla-capstone/
    ├── week-11.md             # VLA Systems Introduction
    ├── week-12.md             # Advanced VLA Implementation
    └── week-13.md             # Capstone Project
```

## 🧠 Technology Stack

- **Frontend**: Docusaurus v3, React, TypeScript
- **Backend**: FastAPI, Python 3.10+
- **Database**: Qdrant Vector Database (for RAG)
- **Embeddings**: Sentence Transformers
- **Language Model**: OpenAI GPT (or open-source alternative)
- **Deployment**: GitHub Pages with GitHub Actions

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

If you encounter any issues or have questions:

1. Check the [Issues](https://github.com/your-org/physical-ai-textbook/issues) page
2. Create a new issue with detailed information
3. Contact the maintainers

## 🙏 Acknowledgments

- Docusaurus team for the excellent documentation framework
- NVIDIA Isaac team for robotics platform
- ROS community for open-source robotics framework
- All contributors who made this textbook possible

---

Made with ❤️ for the Physical AI & Humanoid Robotics community.