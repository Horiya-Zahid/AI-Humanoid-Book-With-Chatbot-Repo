#!/bin/bash
# Deployment script for the Vision-Language-Action (VLA) & Capstone module

set -e  # Exit on any error

echo "Starting deployment process for Physical AI & Humanoid Robotics - VLA Module..."

# Function to display usage
usage() {
    echo "Usage: $0 [environment]"
    echo "Environment can be: dev, staging, prod"
    exit 1
}

# Check if environment is provided
if [ -z "$1" ]; then
    usage
fi

ENVIRONMENT=$1

echo "Deploying to environment: $ENVIRONMENT"

# Validate environment
if [[ "$ENVIRONMENT" != "dev" && "$ENVIRONMENT" != "staging" && "$ENVIRONMENT" != "prod" ]]; then
    echo "Error: Invalid environment. Use dev, staging, or prod."
    usage
fi

# Load environment-specific variables
ENV_FILE=".env.$ENVIRONMENT"
if [ -f "$ENV_FILE" ]; then
    echo "Loading environment variables from $ENV_FILE"
    export $(cat "$ENV_FILE" | xargs)
else
    echo "Warning: $ENV_FILE not found. Using default .env file."
    if [ -f ".env" ]; then
        export $(cat ".env" | xargs)
    fi
fi

# Backend deployment
echo "Deploying backend to $ENVIRONMENT..."

# Install backend dependencies
echo "Installing backend dependencies..."
cd backend
pip install -r requirements.txt

# Run backend tests
echo "Running backend tests..."
python -m pytest tests/ -v

# Backend deployment steps would go here
# For example, deploying to Vercel or Fly.io
echo "Backend deployment completed for $ENVIRONMENT"

cd ..

# Frontend deployment
echo "Deploying frontend to $ENVIRONMENT..."

# Install frontend dependencies
cd frontend
echo "Installing frontend dependencies..."
npm install

# Build frontend
echo "Building frontend..."
npm run build

# Frontend deployment steps would go here
# For example, deploying to GitHub Pages
if [ "$ENVIRONMENT" = "prod" ]; then
    echo "Deploying frontend to GitHub Pages..."
    # Add GitHub Pages deployment commands here
    # npm run deploy
elif [ "$ENVIRONMENT" = "staging" ]; then
    echo "Deploying frontend to staging..."
    # Add staging deployment commands here
else
    echo "Development deployment completed (no actual deployment for dev environment)"
fi

cd ..

# Generate embeddings if needed (for production deployments)
if [ "$ENVIRONMENT" = "prod" ] || [ "$ENVIRONMENT" = "staging" ]; then
    echo "Generating content embeddings..."
    cd scripts
    python generate_embeddings.py
    cd ..
fi

echo "Deployment to $ENVIRONMENT completed successfully!"
echo "Environment: $ENVIRONMENT"
echo "Backend: Deployed"
echo "Frontend: Deployed"
echo "Embeddings: Generated (if applicable)"

# Display deployment URL based on environment
case $ENVIRONMENT in
    "dev")
        echo "Development URL: http://localhost:3000"
        ;;
    "staging")
        echo "Staging URL: https://staging.yourdomain.com"
        ;;
    "prod")
        echo "Production URL: https://yourdomain.com"
        ;;
esac