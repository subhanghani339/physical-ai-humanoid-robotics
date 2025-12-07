from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import os
from dotenv import load_dotenv
import hashlib
import time

load_dotenv()

app = FastAPI(title="Physical AI Chatbot API")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure Gemini
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

# Use Gemini 2.5 Flash
model = genai.GenerativeModel("gemini-2.5-flash")

# Configure Qdrant
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

COLLECTION_NAME = "physical_ai_book"

# Initialize collection if it doesn't exist
try:
    qdrant_client.get_collection(COLLECTION_NAME)
    print(f"✅ Collection '{COLLECTION_NAME}' already exists")
except Exception:
    print(f"Creating collection '{COLLECTION_NAME}'...")
    qdrant_client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=768, distance=Distance.COSINE)
    )
    print(f"✅ Collection '{COLLECTION_NAME}' created")

class ChatRequest(BaseModel):
    message: str
    selected_text: str = ""

class IndexRequest(BaseModel):
    text: str
    source: str


def get_embedding(text: str):
    """Generate embedding using Gemini"""
    try:
        result = genai.embed_content(
            model="models/text-embedding-3-small",
            content=text,
            task_type="embedding"
        )
        # Small delay to avoid rate limit
        time.sleep(1)
        return result['embedding']
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return None


@app.get("/")
async def root():
    return {
        "message": "Physical AI Chatbot API",
        "status": "running",
        "endpoints": {
            "chat": "/api/chat",
            "index": "/api/index",
            "health": "/health"
        }
    }

@app.get("/health")
async def health():
    return {"status": "healthy"}

@app.post("/api/index")
async def index_content(request: IndexRequest):
    """Index book content into Qdrant"""
    try:
        embedding = get_embedding(request.text)
        if not embedding:
            raise HTTPException(status_code=500, detail="Failed to generate embedding")
        
        text_id = int(hashlib.md5(request.text.encode()).hexdigest()[:8], 16)
        
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=text_id,
                    vector=embedding,
                    payload={
                        "text": request.text,
                        "source": request.source
                    }
                )
            ]
        )
        
        return {"status": "success", "message": "Content indexed successfully", "id": text_id}
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/chat")
async def chat(request: ChatRequest):
    """Chat endpoint with RAG"""
    try:
        context = ""
        
        if request.selected_text:
            context = f"Selected text: {request.selected_text}\n\n"
        else:
            query_embedding = get_embedding(request.message)
            if query_embedding:
                search_results = qdrant_client.search(
                    collection_name=COLLECTION_NAME,
                    query_vector=query_embedding,
                    limit=3
                )
                if search_results:
                    context = "Relevant information from the book:\n\n"
                    for result in search_results:
                        context += f"{result.payload['text']}\n\n"
        
        prompt = f"""You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.

{context}

User question: {request.message}

Please provide a clear, concise answer based on the context above. If the context doesn't contain relevant information, use your knowledge about robotics, ROS 2, NVIDIA Isaac, and AI to provide a helpful response."""

        response = model.generate_content(prompt)
        
        return {"answer": response.text, "context_used": bool(context)}
    
    except Exception as e:
        print(f"Error in chat: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
