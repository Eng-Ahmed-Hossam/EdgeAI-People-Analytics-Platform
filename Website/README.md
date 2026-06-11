# EdgeAI Location Intelligence

A location-intelligence platform that scores the commercial feasibility of opening a business at a specific location and accompanies the business after launch with performance analytics.

## Repository structure

```
.
├── frontend/   Next.js 15 app (TypeScript, Tailwind CSS v4)
└── backend/    FastAPI service (Python, PostgreSQL + PostGIS)
```

## Quick start

### Prerequisites

- Node.js ≥ 20
- Python ≥ 3.11
- PostgreSQL ≥ 14 with PostGIS extension
- A PostGIS-enabled database (`CREATE EXTENSION postgis;`)

### Backend

```bash
cd backend
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env — set DATABASE_URL and generate a JWT_SECRET_KEY:
#   python -c "import secrets; print(secrets.token_hex(32))"

# Run database migrations
alembic upgrade head

# Seed initial asset data
python run_ingestion.py

# Start the API server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Frontend

```bash
cd frontend
npm install

# Configure environment
# Create frontend/.env.local with:
#   NEXT_PUBLIC_API_URL=http://localhost:8000

npm run dev
```

Open [http://localhost:3000](http://localhost:3000).

## Environment variables

### Backend (`backend/.env`)

| Variable | Required | Description |
|---|---|---|
| `DATABASE_URL` | Yes | PostgreSQL+asyncpg connection string |
| `JWT_SECRET_KEY` | Yes | HS256 signing secret — generate fresh per environment |
| `CORS_ORIGINS` | Yes | Comma-separated allowed origins |
| `COOKIE_SECURE` | Prod | `true` in production (HTTPS); `false` for local HTTP dev |
| `ACCESS_TOKEN_EXPIRE_MINUTES` | No | JWT lifetime; default 30 |
| `BACKEND_HOST` | No | Bind address; default `0.0.0.0` |
| `BACKEND_PORT` | No | Bind port; default `8000` |

See `backend/.env.example` for the full template.

### Frontend (`frontend/.env.local`)

| Variable | Required | Description |
|---|---|---|
| `NEXT_PUBLIC_API_URL` | No | Backend API base URL; defaults to `http://localhost:8000` |

## Deployment (Vercel + managed Postgres)

The `vercel.json` at the repository root sets `rootDirectory: "frontend"` so Vercel builds from the `frontend/` directory. Set `NEXT_PUBLIC_API_URL` to your backend's production URL in the Vercel environment variables panel.

The backend can be deployed to any platform that supports Python + PostgreSQL (Railway, Render, Fly.io, etc.). Set `COOKIE_SECURE=true` and configure `CORS_ORIGINS` to match your frontend domain.

## Key pages

| Route | Description |
|---|---|
| `/` | Landing page |
| `/workspace` | Map-as-workspace site-selection workbench |
| `/compare` | Side-by-side location comparison |
| `/dashboard` | Executive intelligence overview |
| `/report` | Feasibility report demo |
| `/login` `/signup` | Authentication |
| `/how-it-works` | Methodology |
| `/pricing` | Pricing |
