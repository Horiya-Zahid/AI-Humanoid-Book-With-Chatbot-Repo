# Physical AI & Humanoid Robotics - Frontend

This is the frontend for the Vision-Language-Action (VLA) & Capstone module of the Physical AI & Humanoid Robotics textbook project.

## Overview

The frontend provides:
- Interactive textbook experience with Docusaurus
- Purple/neon theme implementation
- RAG-powered Q&A system with ChatKit SDK
- VLA pipeline visualization
- Capstone project walkthrough
- Voice-to-action demo (bonus feature)

## Tech Stack

- **Framework**: Docusaurus 3.x
- **Language**: React/TypeScript
- **Styling**: CSS Modules, custom theme
- **UI Components**: Custom components and ChatKit SDK

## Installation

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```
2. Install dependencies:
   ```bash
   npm install
   ```
   or if using yarn:
   ```bash
   yarn install
   ```

## Environment Variables

- `REACT_APP_BACKEND_URL`: URL of the backend API (default: http://localhost:8000)

## Running the Development Server

```bash
cd frontend
npm start
```

This command starts a local development server and opens the application in your default browser. Most changes are reflected live without restarting the server.

## Building for Production

```bash
npm run build
```

This command creates an optimized build of your site in the `build` directory, which you can serve using any static server.

## Project Structure

- `/src/` - Source files
  - `/components/` - Reusable React components
    - `/ChatInterface/` - Components for the chat interface
    - `/Theme/` - Theme-related components
    - `/Accessibility/` - Accessibility components
    - `/VLA/` - VLA-specific components
  - `/pages/` - Page components
    - `/Module4/` - Module 4 specific pages
    - `/Capstone/` - Capstone project pages
    - `/Auth/` - Authentication pages
  - `/services/` - API service functions
  - `/styles/` - Custom styles
  - `/utils/` - Utility functions
- `/docs/` - Textbook content
  - `/module-4-vla/` - Module 4 content
  - `/capstone-project/` - Capstone project content
- `/static/` - Static assets

## Key Features

### Theme Implementation
- Purple (#7048e8) and neon (#39ff14) color scheme
- WCAG 2.1 AA compliance
- Responsive design for mobile and desktop
- Accessibility features

### VLA Module
- Interactive VLA pipeline visualization
- Voice command simulation interface
- Cognitive planning visualization
- Capstone project walkthrough

### Q&A Integration
- RAG-powered chat interface using ChatKit SDK
- Text selection query functionality
- Proper citation display
- Strict textbook-only responses

### Accessibility
- High contrast mode support
- Screen reader friendly components
- Keyboard navigation
- Reduced motion support

## Customization

### Theme
The theme can be customized by modifying:
- `docusaurus.config.js` - Main configuration
- `src/css/custom.css` - Custom styles
- `/src/components/Theme/` - Theme components

### Content
Textbook content is written in MDX format in the `/docs/` directory. You can add new content by creating new MDX files in the appropriate subdirectories.

## Deployment

The frontend is designed for deployment to GitHub Pages. The default configuration in `docusaurus.config.js` is set up for this.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test the changes in the development server
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.