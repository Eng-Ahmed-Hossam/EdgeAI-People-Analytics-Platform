"""
Dev convenience script: create all tables directly via SQLAlchemy create_all.

Use Alembic (alembic upgrade head) for production.
Use this only in local dev when you want a quick reset:
  python create_db.py

Prerequisites:
  - PostgreSQL running with PostGIS extension available
  - .env file with DATABASE_URL set
"""

import asyncio

from sqlalchemy import text
from sqlalchemy.ext.asyncio import create_async_engine

import app.models  # noqa: F401 — ensures all models are registered on Base.metadata
from app.config import settings
from app.database import Base


async def main() -> None:
    engine = create_async_engine(settings.database_url, echo=True)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    await engine.dispose()
    print("All tables created.")


if __name__ == "__main__":
    asyncio.run(main())
