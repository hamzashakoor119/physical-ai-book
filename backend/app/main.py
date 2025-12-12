from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import rag, health, auth
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI Humanoid Robotics Book API",
    description="API for RAG-based chatbot and textbook content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routes
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(rag.router, prefix="/api", tags=["rag"])
app.include_router(auth.router, prefix="/api", tags=["auth"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)