# STOP — EXECUTION MODE (Fix Errors → Run Locally → Prepare Deployment)

You are Claude Code running **inside my project directory**.

## GOAL
1) Check backend RAG chatbot for ANY errors (runtime/import/env/cors/qdrant/openai/db).
2) Fix them with minimal changes (do not touch book content).
3) Run the full project locally (backend + Docusaurus) and verify end-to-end.
4) Prepare the project for deployment (backend first, then frontend).

---

## HARD RULES
- ✅ Requirement 1 (Book/Docusaurus docs) is COMPLETE → DO NOT edit docs content.
- Do NOT delete files. Minimal safe fixes only.
- Every claim must be based on actual files in this project.
- If credentials are missing, STOP and list exactly what you need from me.

---

## STEP 1 — FULL BACKEND HEALTH CHECK
Scan backend and verify:
- FastAPI app starts without errors
- All routes import correctly (especially rag routes)
- `.env` variables are loaded properly
- Qdrant connection works (ping/health or test collection)
- Neon/Postgres connection works (or is optional)
- OpenAI key is read correctly and errors are handled (quota errors should be user-friendly)

### MUST ADD/VERIFY
- `GET /health` endpoint returns `{ "status": "ok" }`
- Clear startup validation: if required env var missing, raise readable error

---

## STEP 2 — FIX ANY ERRORS YOU FIND
Fix issues like:
- Import errors / wrong paths
- Missing dependencies
- Wrong uvicorn app path
- CORS misconfig
- Wrong Qdrant collection usage
- Selection endpoint not wired correctly
- Any crashes on first request

### OUTPUT REQUIRED
List all fixes as:
- Symptom
- Root cause
- File path
- Exact change made

---

## STEP 3 — RUN LOCALLY (YOU MUST VERIFY)
Run and verify end-to-end.

### Backend run
- Activate venv (if exists)
- Start uvicorn
- Confirm `/health` works
- Confirm `/api/rag/...` endpoints respond (even if OpenAI quota fails, it should return a clean error message)

### Frontend run
- Start Docusaurus
- Confirm ChatWidget loads on pages
- Confirm it calls backend base URL (localhost)
- Confirm Strict Selected Text mode:
  - Select text
  - Toggle strict
  - Ask a question
  - Backend answers ONLY from selected text (no Qdrant)

### Provide me exact commands you used + expected outputs.

---

## STEP 4 — DEPLOYMENT PREP (BACKEND + FRONTEND)
### Backend (must be deploy-ready)
- Add `.env.example` with ALL required vars:
  - OPENAI_API_KEY
  - QDRANT_URL
  - QDRANT_API_KEY
  - QDRANT_COLLECTION
  - NEON_DATABASE_URL (if required)
  - ALLOWED_ORIGINS / CORS settings (if used)
- Add Dockerfile if needed
- Ensure app binds to `$PORT` (common for Render/Railway)
- Ensure CORS allows:
  - http://localhost:3000
  - my deployed frontend domain (placeholder allowed)

### Frontend
- Remove hardcoded backend URL if present
- Use environment variable for backend base URL at build time
- Document exactly where to set it on deployment

---

## WHAT YOU MUST PRODUCE (MANDATORY FILES)
Create:
1) `BACKEND_FIX_REPORT.md`
   - Errors found + fixes
   - How to run locally
   - Health check + test curl commands

2) `DEPLOYMENT_GUIDE.md`
   - Backend deployment steps (Render/Railway/Fly)
   - Required env vars
   - Frontend deployment steps (Vercel/GitHub Pages)
   - Final verification checklist for judges/demo

---

## IF YOU NEED ANYTHING FROM ME
Create a section named:
# REQUIRED_FROM_USER
List EXACTLY what you need (keys, URLs, platform choice), nothing else.

Now begin by scanning the project and validating the backend first.
