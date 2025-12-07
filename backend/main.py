from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
import hashlib
import traceback
import os
from dotenv import load_dotenv

load_dotenv()

# ----------------------
# FastAPI App
# ----------------------
app = FastAPI(title="Cohere RAG Chatbot (Physical AI)")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------
# Cohere Setup
# ----------------------
CO = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
LLM_MODEL = "command-r-plus"               # Best 2025 model
EMBED_MODEL = "embed-english-v3.0"         # Best embeddings model

# ----------------------
# Qdrant Setup
# ----------------------
COLLECTION_NAME = "physical_ai_book"

qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Create collection if missing
try:
    qdrant.get_collection(COLLECTION_NAME)
except:
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
    )


# ----------------------
# Request Models
# ----------------------
class ChatRequest(BaseModel):
    message: str
    selected_text: str = ""

class IndexRequest(BaseModel):
    text: str
    source: str


# ----------------------
# Embeddings using Cohere
# ----------------------
def get_embedding(text: str):
    try:
        resp = CO.embed(
            texts=[text],
            model=EMBED_MODEL,
            input_type="search_document"
        )
        return resp.embeddings[0]
    except Exception:
        traceback.print_exc()
        return None


# ----------------------
# Index Endpoint
# ----------------------
@app.post("/api/index")
async def index_content(req: IndexRequest):
    emb = get_embedding(req.text)
    if emb is None:
        raise HTTPException(status_code=500, detail="Embedding failed")

    text_id = hashlib.md5(req.text.encode()).hexdigest()

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=text_id,
                vector=emb,
                payload={"text": req.text, "source": req.source},
            )
        ],
    )

    return {"status": "success", "id": text_id}


# ----------------------
# Chat / Retrieval Endpoint
# ----------------------
@app.post("/api/chat")
async def chat(req: ChatRequest):

    context = ""

    # If user selected specific text (PDF highlight etc)
    if req.selected_text:
        context = f"Selected passage:\n{req.selected_text}\n"
    else:
        # Otherwise, semantic search from Qdrant
        q_emb = get_embedding(req.message)
        if q_emb:
            search_result =  qdrant.query_points(
                collection_name=COLLECTION_NAME,
                query=q_emb,
                limit=4
            )

            results = search_result.points

            # return results

            if results:
                context = "Relevant extracted sections:\n\n"
                for r in results:
                    return r.payload['text']
                    context += f"- {r.payload['text']}\n\n"

    # Cohere Prompt
    prompt = f"""
You are a robotics / humanoid AI expert trained on the textbook:

📘 "Physical AI, Humanoid Robotics, ROS 2, NVIDIA Isaac and AI Engineering"

Book Context:
{context}

User Question:
{req.message}

Answer clearly, correctly and professionally.
"""

    try:
        resp = CO.chat(
            model=LLM_MODEL,
            message=prompt,
            temperature=0.2,
        )
        answer = resp.text

    except Exception as e:
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))

    return {
        "answer": answer,
        "context_used": bool(context.strip()),
    }


# ----------------------
# Health
# ----------------------
@app.get("/health")
async def health():
    return {"status": "ok"}


# ----------------------
# Run
# ----------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
