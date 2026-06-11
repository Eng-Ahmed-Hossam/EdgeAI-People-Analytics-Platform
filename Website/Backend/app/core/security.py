"""
Security utilities — password hashing, JWT creation, and the get_current_user
FastAPI dependency.

Rules enforced here:
  - Passwords are hashed with bcrypt; plaintext is never stored or logged.
  - JWT secret comes exclusively from settings (loaded from .env).
  - get_current_user raises HTTP 401 (not 403) for any auth failure.
  - The error message never distinguishes "wrong password" from "no such user"
    to prevent user enumeration.
  - JWT is read from the httpOnly 'access_token' cookie (preferred) with a
    fallback to the Authorization: Bearer header (for API clients / testing).
"""

from datetime import datetime, timedelta, timezone

import bcrypt as _bcrypt
from fastapi import Cookie, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from jose import JWTError, jwt
from sqlalchemy.ext.asyncio import AsyncSession

from app.config import settings
from app.database import get_db

# ── Password hashing — direct bcrypt (bypasses passlib which is incompatible
#    with bcrypt 4+; passlib's detect_wrap_bug uses a 73-byte test password
#    that newer bcrypt correctly rejects). ─────────────────────────────────────


def hash_password(plain: str) -> str:
    """Return the bcrypt hash of a plaintext password."""
    return _bcrypt.hashpw(plain.encode("utf-8"), _bcrypt.gensalt()).decode("utf-8")


def verify_password(plain: str, hashed: str) -> bool:
    """Return True iff plain matches the stored bcrypt hash."""
    return _bcrypt.checkpw(plain.encode("utf-8"), hashed.encode("utf-8"))


# ── JWT ───────────────────────────────────────────────────────────────────────

def create_access_token(user_id: str) -> str:
    """
    Create a signed JWT carrying the user's id as the 'sub' claim.
    Expiry is controlled by settings.access_token_expire_minutes.
    """
    expire = datetime.now(timezone.utc) + timedelta(
        minutes=settings.access_token_expire_minutes
    )
    payload = {"sub": user_id, "exp": expire}
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)


# ── OAuth2 bearer scheme — auto_error=False so a missing header returns None
#    instead of raising 401 immediately. Cookie is tried first; Bearer header
#    is the fallback for API clients and automated testing. ───────────────────

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/auth/login", auto_error=False)


# ── Dependency ────────────────────────────────────────────────────────────────

_CREDENTIALS_EXCEPTION = HTTPException(
    status_code=status.HTTP_401_UNAUTHORIZED,
    detail="Invalid or expired token.",
    headers={"WWW-Authenticate": "Bearer"},
)

_FORBIDDEN_EXCEPTION = HTTPException(
    status_code=status.HTTP_403_FORBIDDEN,
    detail="Admin access required.",
)


async def get_current_user(
    access_token: str | None = Cookie(default=None),
    header_token: str | None = Depends(oauth2_scheme),
    db: AsyncSession = Depends(get_db),
):
    """
    FastAPI dependency — validates the JWT and returns the User row.

    Token resolution order:
      1. httpOnly cookie named 'access_token' (set by POST /api/auth/login)
      2. Authorization: Bearer header (fallback for API clients / testing)

    Raises HTTP 401 if:
      - No token is found via either mechanism.
      - The signature is invalid or the token has expired.
      - The sub claim is absent.
      - The user id in sub does not exist in the database.
      - The user's is_active flag is False.

    Import pattern for endpoints:
        from app.core.security import get_current_user
        from app.models.user import User

        @router.get("/protected")
        async def protected(current_user: User = Depends(get_current_user)):
            ...
    """
    from app.crud.users import get_user_by_id  # deferred to avoid circular import

    token = access_token or header_token
    if token is None:
        raise _CREDENTIALS_EXCEPTION

    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret_key,
            algorithms=[settings.jwt_algorithm],
        )
        user_id: str | None = payload.get("sub")
        if user_id is None:
            raise _CREDENTIALS_EXCEPTION
    except JWTError:
        raise _CREDENTIALS_EXCEPTION

    user = await get_user_by_id(db, user_id)
    if user is None or not user.is_active:
        raise _CREDENTIALS_EXCEPTION

    return user


async def require_admin(
    current_user=Depends(get_current_user),
):
    """
    FastAPI dependency — requires the authenticated user to have role='admin'.

    Raises HTTP 403 (Forbidden) if the user is authenticated but not an admin.
    HTTP 401 is raised upstream by get_current_user for any auth failure.

    Usage:
        @router.get("/admin/thing")
        async def thing(_: User = Depends(require_admin)):
            ...
    """
    if current_user.role != "admin":
        raise _FORBIDDEN_EXCEPTION
    return current_user
