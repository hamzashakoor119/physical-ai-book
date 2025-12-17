from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
from app.utils.qdrant_client import get_qdrant_client
from app.utils.embeddings import get_embeddings
from app.utils.translation import translate_response_with_context, translate_to_urdu
from app.models.vector_schema import RAGQuery, RAGResponse, TextSelection, UserBackground
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

router = APIRouter()

# Lazy OpenAI client initialization
_openai_client = None


def get_openai_client():
    """Get or initialize OpenAI client lazily."""
    global _openai_client
    if _openai_client is None:
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            return None
        from openai import OpenAI
        _openai_client = OpenAI(api_key=api_key)
    return _openai_client

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
        client = get_openai_client()
        if client is None:
            return f"OpenAI API key not configured. Based on the textbook content: {context_str[:500]}..."
        response = client.chat.completions.create(
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

class TranslationRequest(BaseModel):
    text: str
    target_language: str = "ur"

class TranslationResponse(BaseModel):
    original_text: str
    translated_text: str
    target_language: str

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

        # If user prefers Urdu, translate the response
        if query.user_background and query.user_background.preferred_language == "ur":
            translation_result = translate_response_with_context(response_text, context_texts, sources)
            response_text = translation_result["urdu_answer"]
            context_texts = translation_result["urdu_context"]
            sources = translation_result["urdu_sources"]

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

        # If user prefers Urdu, translate the response
        if selection.user_background and selection.user_background.preferred_language == "ur":
            translation_result = translate_response_with_context(response_text, context_texts, sources)
            response_text = translation_result["urdu_answer"]
            context_texts = translation_result["urdu_context"]
            sources = translation_result["urdu_sources"]

        return RAGResponse(
            question=selection.question,
            answer=response_text,
            context=context_texts,
            sources=sources,
            user_background=selection.user_background
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing selection query: {str(e)}")

@router.post("/translate", response_model=TranslationResponse)
async def translate_text(translation_request: TranslationRequest):
    """
    Translate text to the specified language (currently Urdu support)
    """
    try:
        if translation_request.target_language.lower() == "ur":
            translated_text = translate_to_urdu(translation_request.text)
        else:
            # For now, only Urdu is supported, but could be extended
            translated_text = translation_request.text

        return TranslationResponse(
            original_text=translation_request.text,
            translated_text=translated_text,
            target_language=translation_request.target_language
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error translating text: {str(e)}")

def generate_response_from_selection_only(selected_text: str, question: str, user_background: Optional[dict] = None) -> str:
    """
    Generate a response using ONLY the selected text (strict mode - no Qdrant search).
    """
    if not selected_text or not selected_text.strip():
        return "No text was selected. Please select some text from the page to ask a question about it."

    # Create a prompt that strictly uses only the selected text
    if user_background:
        background_str = f"User background: Software experience: {user_background.get('software_experience', 'N/A')}, Hardware experience: {user_background.get('hardware_experience', 'N/A')}, Robotics knowledge: {user_background.get('robotics_knowledge', 'N/A')}. "
        background_str += "Please tailor your response to match the user's experience level.\n\n"
    else:
        background_str = ""

    prompt = f"""{background_str}The user has selected the following text from a Physical AI and Humanoid Robotics textbook:

---
{selected_text}
---

Question: {question}

IMPORTANT INSTRUCTIONS:
1. Answer ONLY based on the selected text above.
2. Do NOT use any external knowledge or information not present in the selected text.
3. If the selected text does not contain enough information to answer the question, explicitly say: "The selected text does not contain enough information to answer this question."
4. Be concise and accurate."""

    try:
        client = get_openai_client()
        if client is None:
            # Fallback without OpenAI - provide the selected text as context
            return f"[OpenAI not configured] Based on your selected text: '{selected_text[:300]}...' - Please review the text above for information about: {question}"

        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are an AI assistant helping users understand specific text they have selected from a Physical AI and Humanoid Robotics textbook. You must ONLY use information from the selected text. Never add external knowledge."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=400,
            temperature=0.3  # Lower temperature for more focused answers
        )

        return response.choices[0].message.content
    except Exception as e:
        print(f"Error calling OpenAI API for selection: {str(e)}")
        # Fallback response showing the selected text
        return f"[API Error] Here is the selected text you asked about: '{selected_text[:500]}...' Your question was: {question}"


class StrictSelectionQuery(BaseModel):
    """Request model for strict selection-only queries (no Qdrant search)."""
    selected_text: str
    question: str
    user_background: Optional[UserBackground] = None


@router.post("/rag/answer-from-selection")
async def answer_from_selection_only(query: StrictSelectionQuery):
    """
    Answer a question using ONLY the selected text (STRICT MODE).

    This endpoint does NOT:
    - Query Qdrant vector database
    - Use book embeddings
    - Access any content outside the selected text

    It answers strictly from the selected_text provided.
    If the answer cannot be found in the selected text, it says so explicitly.
    """
    try:
        # Generate response using ONLY the selected text
        response_text = generate_response_from_selection_only(
            query.selected_text,
            query.question,
            query.user_background.dict() if query.user_background else None
        )

        # If user prefers Urdu, translate the response
        if query.user_background and query.user_background.preferred_language == "ur":
            translated = translate_to_urdu(response_text)
            response_text = translated

        return {
            "question": query.question,
            "answer": response_text,
            "selected_text": query.selected_text[:200] + "..." if len(query.selected_text) > 200 else query.selected_text,
            "mode": "strict_selection",
            "sources": ["User-selected text"],
            "user_background": query.user_background
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing strict selection query: {str(e)}")


@router.post("/embeddings/process-textbook")
async def process_textbook_content():
    """
    Endpoint to process and embed textbook content into vector database.
    Reads markdown files from /docs, chunks them, generates embeddings,
    and stores them in Qdrant for RAG retrieval.
    """
    try:
        from app.utils.textbook_processor import index_textbook_content

        # Get Qdrant client
        qdrant_client = get_qdrant_client()

        # Index textbook content
        chunks_indexed = index_textbook_content(qdrant_client)

        if chunks_indexed == 0:
            return {
                "message": "No textbook content found to index",
                "status": "warning",
                "chunks_indexed": 0
            }

        return {
            "message": f"Successfully indexed {chunks_indexed} chunks from textbook",
            "status": "success",
            "chunks_indexed": chunks_indexed
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing textbook content: {str(e)}")