import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Dict, Any
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
def get_qdrant_client():
    """
    Initialize and return a Qdrant client instance
    """
    try:
        # Use local Qdrant instance by default, or cloud if URL is provided
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key
            )
        else:
            # For local instance without authentication
            client = QdrantClient(
                url=qdrant_url
            )

        # Ensure the collection exists
        ensure_collection_exists(client)

        return client
    except Exception as e:
        print(f"Error initializing Qdrant client: {str(e)}")
        # Return a mock client for testing purposes
        return MockQdrantClient()

def ensure_collection_exists(client: QdrantClient, collection_name: str = "textbook_content"):
    """
    Ensure that the specified collection exists in Qdrant
    """
    try:
        # Check if collection exists
        collections = client.get_collections()
        collection_names = [collection.name for collection in collections.collections]

        if collection_name not in collection_names:
            # Create collection with vector configuration
            client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=384, distance=Distance.COSINE)  # all-MiniLM-L6-v2 outputs 384-dim vectors
            )
            print(f"Created collection: {collection_name}")
        else:
            print(f"Collection {collection_name} already exists")
    except Exception as e:
        print(f"Error ensuring collection exists: {str(e)}")

def upsert_vectors(
    client: QdrantClient,
    collection_name: str,
    vectors: List[List[float]],
    payloads: List[Dict[str, Any]],
    ids: List[str]
):
    """
    Upsert vectors into the specified collection
    """
    try:
        client.upsert(
            collection_name=collection_name,
            points=models.Batch(
                ids=ids,
                vectors=vectors,
                payloads=payloads
            )
        )
        print(f"Upserted {len(ids)} vectors into {collection_name}")
    except Exception as e:
        print(f"Error upserting vectors: {str(e)}")

def search_vectors(
    client: QdrantClient,
    collection_name: str,
    query_vector: List[float],
    limit: int = 5,
    filters: Dict[str, Any] = None
):
    """
    Search for similar vectors in the collection
    """
    try:
        # Convert filters to Qdrant's filter format if provided
        qdrant_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )
            if filter_conditions:
                qdrant_filter = models.Filter(must=filter_conditions)

        results = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit,
            with_payload=True,
            query_filter=qdrant_filter
        )

        return results
    except Exception as e:
        print(f"Error searching vectors: {str(e)}")
        return []

# Mock class for testing when Qdrant is not available
class MockQdrantClient:
    def __init__(self):
        self.collections = {}
        self.points = {}

    def get_collections(self):
        class CollectionInfo:
            def __init__(self, name):
                self.name = name

        class CollectionsResponse:
            def __init__(self):
                self.collections = [CollectionInfo("textbook_content")]

        return CollectionsResponse()

    def create_collection(self, collection_name, vectors_config):
        self.collections[collection_name] = vectors_config
        self.points[collection_name] = []
        print(f"Mock: Created collection {collection_name}")

    def upsert(self, collection_name, points):
        if collection_name not in self.points:
            self.points[collection_name] = []

        for i, point_id in enumerate(points.ids):
            self.points[collection_name].append({
                "id": point_id,
                "vector": points.vectors[i],
                "payload": points.payloads[i]
            })
        print(f"Mock: Upserted {len(points.ids)} points")

    def search(self, collection_name, query_vector, limit, with_payload, query_filter=None):
        # Mock search results
        class MockResult:
            def __init__(self, id, score, payload):
                self.id = id
                self.score = score
                self.payload = payload

        # Return mock results
        mock_results = []
        for i in range(min(limit, 3)):
            mock_results.append(MockResult(
                id=f"mock_id_{i}",
                score=0.9 - (i * 0.1),
                payload={"content": f"Mock content for query result {i}"}
            ))

        return mock_results