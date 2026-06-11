from pydantic import field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
    )

    database_url: str = "postgresql+asyncpg://postgres:postgres@localhost:5432/edgeai"

    @field_validator("database_url", mode="before")
    @classmethod
    def _ensure_asyncpg_driver(cls, v: str) -> str:
        # Railway (and most managed Postgres services) emit postgresql:// or
        # postgres:// — SQLAlchemy asyncpg requires postgresql+asyncpg://.
        if isinstance(v, str):
            v = v.replace("postgres://", "postgresql://", 1)
            if v.startswith("postgresql://"):
                v = v.replace("postgresql://", "postgresql+asyncpg://", 1)
        return v
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000
    cors_origins: str = "http://localhost:3000,http://127.0.0.1:3000"
    seed_data_path: str = "ingestion/seed_data.json"
    geocoding_user_agent: str = "edgeai-location-intelligence/1.0"

    # ── Auth — JWT ────────────────────────────────────────────────────────────
    # Must be overridden in .env. Never hardcode in source. Generate with:
    #   python -c "import secrets; print(secrets.token_hex(32))"
    jwt_secret_key: str = "INSECURE_DEFAULT_CHANGE_IN_DOT_ENV"
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # ── Auth — Cookie ─────────────────────────────────────────────────────────
    # Set COOKIE_SECURE=true in production (.env). Keep False for local HTTP dev.
    # SameSite=Lax is always used; localhost:3000 → localhost:8000 is same-site.
    cookie_secure: bool = False

    @property
    def cors_origins_list(self) -> list[str]:
        return [o.strip() for o in self.cors_origins.split(",")]


settings = Settings()
