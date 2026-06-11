"""
EdgeAI Location Intelligence — FastAPI backend.

Entry point. Registers routers, CORS, and startup/shutdown events.
Run with: uvicorn main:app --reload --host 0.0.0.0 --port 8000
"""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import settings
from app.routers import (
    admin_router,
    assessments_router,
    assets_router,
    auth_router,
    valuation_router,
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: nothing yet (DB tables managed by Alembic / create_db.py)
    yield
    # Shutdown: nothing yet


app = FastAPI(
    title="EdgeAI Location Intelligence API",
    version="0.1.0",
    description=(
        "Backend API for the EdgeAI Location Intelligence platform. "
        "Serves asset data to the frontend map workspace. "
        "Assessment, scoring, valuation, and temporal intelligence endpoints are future sessions."
    ),
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

app.include_router(assets_router, prefix="/api")
app.include_router(assessments_router, prefix="/api")
app.include_router(valuation_router, prefix="/api")
app.include_router(auth_router, prefix="/api")
app.include_router(admin_router, prefix="/api")


@app.get("/health", tags=["system"])
async def health_check() -> dict:
    return {"status": "ok", "version": "0.1.0"}
