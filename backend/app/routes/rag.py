from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from app.utils.qdrant_client import get_qdrant_client
from app.utils.embeddings import get_embeddings
from app.models.vector_schema import RAGQuery, RAGResponse, TextSelection
from openai import OpenAI
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

router = APIRouter()

# Initialize OpenAI client
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def generate_response_with_openai(context_texts: List[str], question: str, user_background: Optional[dict] = None) -> str:
    """
    Generate a response using OpenAI based on the retrieved context
    """
    if not context_texts:
        return "I couldn't find any relevant information in the textbook to answer your question."

    # Build the context string
    context_str = "\n\n".join(context_texts[:3])  # Use top 3 results

    # Create a prompt that includes user background if available
    if user_background:
        background_str = f"User background: Software experience: {user_background.get('software_experience', 'N/A')}, Hardware experience: {user_background.get('hardware_experience', 'N/A')}, Robotics knowledge: {user_background.get('robotics_knowledge', 'N/A')}. "
        background_str += "Please tailor your response to match the user's experience level.\n\n"
    else:
        background_str = ""

    # Build the full prompt
    prompt = f"""{background_str}Context information from the textbook:
    {context_str}

    Please answer the following question based on the context provided: {question}

    If the context doesn't contain enough information to answer the question, please say so and provide any related information that might be helpful."""

    try:
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",  # You can change this to gpt-4 if preferred
            messages=[
                {"role": "system", "content": "You are an AI assistant for a Physical AI and Humanoid Robotics textbook. Provide accurate, helpful answers based on the textbook content. Be concise but informative."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )

        return response.choices[0].message.content
    except Exception as e:
        print(f"Error calling OpenAI API: {str(e)}")
        # Fallback response
        return f"Based on the textbook content, here's information related to your question: '{question}'. [Note: OpenAI API call failed, showing context instead] Context: {context_str[:500]}..."

@router.post("/rag/query", response_model=RAGResponse)
async def rag_query(query: RAGQuery):
    """
    Process a RAG query and return a response based on textbook content
    """
    try:
        # Get Qdrant client
        qdrant_client = get_qdrant_client()

        # Generate embedding for the query
        query_embedding = get_embeddings([query.question])[0]

        # Search in Qdrant vector database
        search_results = qdrant_client.search(
            collection_name="textbook_content",
            query_vector=query_embedding,
            limit=query.top_k or 5,
            with_payload=True
        )

        # Extract relevant content
        context_texts = []
        sources = []
        for result in search_results:
            if result.payload:
                content = result.payload.get("content", "")
                if content:
                    context_texts.append(content)
                    # Add source information if available
                    source_info = result.payload.get("source", "")
                    if source_info:
                        sources.append(source_info)

        # Generate response using OpenAI
        response_text = generate_response_with_openai(context_texts, query.question, query.user_background.dict() if query.user_background else None)

        return RAGResponse(
            question=query.question,
            answer=response_text,
            context=context_texts,
            sources=sources,
            user_background=query.user_background
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing RAG query: {str(e)}")

@router.post("/rag/selection-query", response_model=RAGResponse)
async def selection_query(selection: TextSelection):
    """
    Process a query about selected text and return a response
    """
    try:
        # Get Qdrant client
        qdrant_client = get_qdrant_client()

        # Generate embedding for the selected text combined with the question
        query_embedding = get_embeddings([selection.selected_text + " " + selection.question])[0]

        # Search in Qdrant vector database for related content
        search_results = qdrant_client.search(
            collection_name="textbook_content",
            query_vector=query_embedding,
            limit=selection.top_k or 3,
            with_payload=True
        )

        # Extract relevant content
        context_texts = []
        sources = []
        for result in search_results:
            if result.payload:
                content = result.payload.get("content", "")
                if content:
                    context_texts.append(content)
                    # Add source information if available
                    source_info = result.payload.get("source", "")
                    if source_info:
                        sources.append(source_info)

        # Generate response using OpenAI
        full_question = f"Regarding '{selection.selected_text[:100]}...', {selection.question}"
        response_text = generate_response_with_openai(context_texts, full_question, selection.user_background.dict() if selection.user_background else None)

        return RAGResponse(
            question=selection.question,
            answer=response_text,
            context=context_texts,
            sources=sources,
            user_background=selection.user_background
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing selection query: {str(e)}")

@router.post("/embeddings/process-textbook")
async def process_textbook_content():
    """
    Endpoint to process and embed textbook content into vector database
    This would typically be called during deployment to index the textbook
    """
    try:
        # In a real implementation, this would:
        # 1. Read all textbook markdown files from the frontend docs directory
        # 2. Split them into chunks
        # 3. Generate embeddings for each chunk
        # 4. Store in Qdrant vector database
        return {"message": "Textbook content processing would start here", "status": "not_implemented_yet"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing textbook content: {str(e)}")