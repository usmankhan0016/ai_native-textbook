# RAG Indexing Guide for Module 4

This guide explains how to index Module 4 content for RAG (Retrieval-Augmented Generation) chatbot integration.

---

## Overview

RAG enables students to query the textbook via natural language. To support this, we need to:

- **Semantic chunking**: Break markdown into discrete, meaningful units
- **Embedding generation**: Convert chunks to vector embeddings
- **Vector storage**: Store embeddings in a vector database (e.g., Qdrant)
- **Metadata indexing**: Store chapter info, difficulty, topics for filtering
- **Query handling**: Hybrid search (vector similarity + metadata filters)

---

## Semantic Chunks for Module 4

### Chapter 1: VLA Fundamentals

**Chunk 1.1**: Learning Objectives (5 measurable outcomes)
- **Keywords**: VLA, architecture, humanoids, cognitive robotics, behavior trees
- **Difficulty**: Beginner

**Chunk 1.2**: VLA System Architecture
- **Keywords**: Vision, Language, Action, pipeline, integration
- **Difficulty**: Beginner

**Chunk 1.3**: Real-World VLA Systems
- **Keywords**: OpenAI RT-2, DeepMind RT-X, NVIDIA, robotics applications
- **Difficulty**: Beginner

**Chunk 1.4**: Task Representation Patterns
- **Keywords**: Skills, behaviors, action graphs, behavior trees
- **Difficulty**: Intermediate

**Chunk 1.5**: Lab 1 - Analyze a Real VLA System
- **Keywords**: Lab, hands-on, analysis, project-based learning
- **Difficulty**: Beginner

[Similar chunking for Chapters 2-4 and Week 13...]

---

## Implementation Steps

### Step 1: Prepare Markdown for Chunking

**Script**: `scripts/chunk_markdown.py`

```python
import re
from pathlib import Path

def extract_chunks(markdown_file):
    """Extract semantic chunks from markdown based on H3 headers."""
    chunks = []

    with open(markdown_file, 'r') as f:
        content = f.read()

    # Split on ### headers (level 3)
    sections = re.split(r'^### ', content, flags=re.MULTILINE)

    for section in sections[1:]:  # Skip preamble
        lines = section.split('\n')
        title = lines[0].strip()
        body = '\n'.join(lines[1:])

        chunks.append({
            'title': title,
            'content': body,
            'file': markdown_file.name,
            'chapter': extract_chapter_name(markdown_file),
            'length': len(body.split()),
        })

    return chunks

# Usage
for chapter_file in Path('docs/module-4').glob('*.md'):
    chunks = extract_chunks(chapter_file)
    for chunk in chunks:
        print(f"Chunk: {chunk['title']} ({chunk['length']} words)")
```

**Output**: List of ~100 chunks across all chapters

### Step 2: Generate Embeddings

**Library**: OpenAI Embeddings API or open-source (sentence-transformers)

```python
import openai
from pathlib import Path

def generate_embeddings(chunks):
    """Generate embeddings for all chunks."""
    embeddings = []

    for chunk in chunks:
        # Create combined text for embedding
        text_to_embed = f"{chunk['title']}\n{chunk['content']}"

        # Call embedding API
        response = openai.Embedding.create(
            input=text_to_embed,
            model="text-embedding-3-small"
        )

        embedding = response['data'][0]['embedding']

        embeddings.append({
            'chunk_id': f"{chunk['chapter']}_{chunk['title'][:30]}",
            'content': chunk['content'],
            'embedding': embedding,
            'metadata': {
                'chapter': chunk['chapter'],
                'difficulty': extract_difficulty(chunk),
                'keywords': extract_keywords(chunk),
                'word_count': chunk['length'],
                'source': f"docs/module-4/{chunk['file']}"
            }
        })

    return embeddings
```

**Cost estimate**: ~500 chunks × $0.00002/embedding = ~$0.01 (very cheap)

### Step 3: Store in Vector Database

**Database**: Qdrant Cloud Free Tier (1GB storage, sufficient for textbook)

```python
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Distance, VectorParams

def store_in_qdrant(embeddings):
    """Store embeddings in Qdrant vector database."""
    client = QdrantClient(
        url="https://[your-cluster].qdrant.io",
        api_key="[your-api-key]"
    )

    # Create collection
    client.create_collection(
        collection_name="module-4-vla",
        vectors_config=VectorParams(
            size=1536,  # OpenAI embedding dimension
            distance=Distance.COSINE
        )
    )

    # Insert points
    points = [
        PointStruct(
            id=i,
            vector=emb['embedding'],
            payload={
                'content': emb['content'],
                'chapter': emb['metadata']['chapter'],
                'difficulty': emb['metadata']['difficulty'],
                'keywords': emb['metadata']['keywords'],
                'source': emb['metadata']['source']
            }
        )
        for i, emb in enumerate(embeddings)
    ]

    client.upsert(collection_name="module-4-vla", points=points)
    print(f"Stored {len(points)} chunks in Qdrant")
```

### Step 4: Query Handler

**Framework**: FastAPI (backend) + React (frontend)

```python
# Backend: FastAPI
from fastapi import FastAPI
from qdrant_client import QdrantClient
import openai

app = FastAPI()
qdrant_client = QdrantClient(url="https://[your-cluster].qdrant.io")

@app.post("/api/chat/query")
async def query_module4(question: str, difficulty_filter: str = None):
    """Handle RAG query for Module 4 content."""

    # 1. Embed the question
    question_embedding = openai.Embedding.create(
        input=question,
        model="text-embedding-3-small"
    )['data'][0]['embedding']

    # 2. Search Qdrant (with optional difficulty filter)
    filter_condition = None
    if difficulty_filter:
        filter_condition = {
            "key": "difficulty",
            "match": {"value": difficulty_filter}
        }

    search_results = qdrant_client.search(
        collection_name="module-4-vla",
        query_vector=question_embedding,
        query_filter=filter_condition,
        limit=3  # Top 3 most relevant chunks
    )

    # 3. Format search results as context
    context = "\n\n".join([
        f"From {result.payload['source']}:\n{result.payload['content']}"
        for result in search_results
    ])

    # 4. Call LLM with context
    llm_response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a helpful robotics instructor. Answer the student's question based on the textbook content provided."},
            {"role": "user", "content": f"Context from Module 4 VLA Robotics:\n{context}\n\nStudent question: {question}"}
        ]
    )

    return {
        "answer": llm_response['choices'][0]['message']['content'],
        "sources": [
            {
                "chapter": result.payload['chapter'],
                "file": result.payload['source'],
                "similarity_score": result.score
            }
            for result in search_results
        ]
    }
```

---

## Metadata Schema

Store the following metadata with each chunk:

```json
{
  "chunk_id": "chapter-1_vla-architecture",
  "source": "docs/module-4/chapter-1-vla-intro.md",
  "chapter": "Chapter 1: VLA Fundamentals",
  "section": "VLA System Architecture",
  "difficulty": "beginner",
  "keywords": ["VLA", "vision", "language", "action", "architecture"],
  "word_count": 450,
  "is_code_example": false,
  "is_lab": false,
  "is_exercise": false,
  "estimated_read_time_minutes": 5,
  "related_skills": ["Understand VLA components", "Design robot systems"],
  "prerequisite_chunks": ["chapter-1_what-is-vla"]
}
```

---

## Testing the RAG System

### Test Queries

**Beginner Query** (Should return Chapter 1 content):
- "What is VLA and how does it work?"
- Expected: Learning objectives + architecture explanation

**Intermediate Query** (Should return Chapter 2):
- "How do I implement real-time object detection for robots?"
- Expected: YOLO, ROS 2 integration, latency considerations

**Advanced Query** (Should return Chapter 4 + cross-chapter):
- "How do I deploy a VLA system to Jetson hardware?"
- Expected: Optimization techniques, real-time constraints

**Cross-Chapter Query**:
- "How does perception feed into LLM planning?"
- Expected: Ch2 perception output → Ch3 LLM input explanation

### Evaluation Metrics

- **Relevance**: % of results directly addressing the question (target: `>80%`)
- **Completeness**: % of questions answered without follow-up (target: `>70%`)
- **Accuracy**: % of provided code/examples that work (target: `100%`)
- **Latency**: Time from query to response (target: ``<2`` seconds)

---

## Deployment Checklist

- [ ] All Chapter 1-4 markdown files validated for chunking
- [ ] Week 13 capstone content indexed
- [ ] ~100 semantic chunks identified and tested
- [ ] Embeddings generated for all chunks (<$1 cost)
- [ ] Qdrant collection created and populated
- [ ] FastAPI backend deployed (Render Free Tier or Railway)
- [ ] RAG query handler tested with sample questions
- [ ] Frontend chatbot widget integrated into Docusaurus
- [ ] Rate limiting configured (10 queries/minute/user)
- [ ] Error handling and fallbacks implemented

---

## Example Integration: Docusaurus Chatbot Widget

```jsx
// docs/module-4/ChatbotWidget.jsx
import React, { useState } from 'react';
import styles from './ChatbotWidget.module.css';

export default function ChatbotWidget() {
  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState(null);
  const [loading, setLoading] = useState(false);

  const handleQuery = async () => {
    setLoading(true);
    try {
      const response = await fetch('/api/chat/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question })
      });
      const data = await response.json();
      setAnswer(data);
    } catch (error) {
      console.error('Error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbot}>
      <h3>Ask about VLA Robotics</h3>
      <input
        type="text"
        value={question}
        onChange={(e) => setQuestion(e.target.value)}
        placeholder="e.g., How do I detect objects with YOLO?"
      />
      <button onClick={handleQuery} disabled={loading}>
        {loading ? 'Loading...' : 'Ask'}
      </button>
      {answer && (
        <div className={styles.answer}>
          <p>{answer.answer}</p>
          <div className={styles.sources}>
            <strong>Sources:</strong>
            {answer.sources.map((src) => (
              <a key={src.file} href={src.file}>{src.chapter}</a>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
```

---

## Next Steps

**Immediate** (Week 1):
- [ ] Implement semantic chunking script
- [ ] Generate embeddings for all content
- [ ] Set up Qdrant collection

**Short-term** (Week 2):
- [ ] Deploy FastAPI backend
- [ ] Implement query handler
- [ ] Test with sample queries

**Medium-term** (Week 3-4):
- [ ] Integrate chatbot widget into Docusaurus
- [ ] Collect user feedback on answer quality
- [ ] Iterate on prompting and chunking strategy

---

## Resources

- **Qdrant**: https://qdrant.tech/
- **OpenAI Embeddings**: https://platform.openai.com/docs/guides/embeddings
- **FastAPI**: https://fastapi.tiangolo.com/
- **LangChain RAG**: https://python.langchain.com/docs/modules/data_connection/

---

**RAG indexing makes Module 4 content discoverable and interactive for students!**
