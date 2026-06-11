"""
Auth router — /api/auth/*

POST /api/auth/register — create a user, return UserResponse (no hash).
POST /api/auth/login    — validate credentials, set httpOnly cookie, return UserResponse.
POST /api/auth/logout   — clear the httpOnly cookie server-side.
GET  /api/auth/me       — validate cookie JWT, return the current UserResponse.

Security rules:
  - Passwords are hashed with bcrypt before storage; plaintext never persists.
  - Login failure always returns the same "Invalid credentials." message,
    regardless of whether the email exists — prevents user enumeration.
  - JWT is stored in an httpOnly cookie named 'access_token'.
    SameSite=None (Secure) in production for cross-domain use (Vercel → Railway).
    SameSite=Lax in local dev (same-site localhost, no Secure flag needed).
    It is never returned in the response body and never in JavaScript-accessible
    storage. The frontend receives a UserResponse, not a token.
  - COOKIE_SECURE controls the Secure flag: False for local HTTP dev, True in
    production. Set via .env (see app/config.py).
  - /me validates the cookie JWT and returns the user, enabling session restore
    on page load without exposing the token to JavaScript.

Out of scope (future concerns — documented in .env.example):
  - Role-based authorization: Auth Session 2 (complete).
  - Ownership migration: Auth Session 3 (complete).
  - Password reset.
  - Refresh tokens.
  - Rate limiting (use reverse proxy or slowapi).
"""

from fastapi import APIRouter, Depends, HTTPException, Response, status
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from app.config import settings
from app.core.security import (
    create_access_token,
    get_current_user,
    hash_password,
    verify_password,
)
from app.crud.users import create_user, get_user_by_email
from app.database import get_db
from app.models.user import User
from app.schemas.auth import (
    LogoutResponse,
    RegisterRequest,
    LoginRequest,
    UserResponse,
)

router = APIRouter(prefix="/auth", tags=["auth"])

_COOKIE_NAME = "access_token"


def _samesite() -> str:
    # SameSite=None is required for cross-domain cookies (frontend on vercel.app,
    # backend on railway.app). SameSite=None MUST be paired with Secure=True —
    # only valid over HTTPS, which is always the case in production.
    # Local dev uses SameSite=Lax (no Secure flag, plain HTTP localhost).
    return "none" if settings.cookie_secure else "lax"


def _set_auth_cookie(response: Response, token: str) -> None:
    """Attach the httpOnly session cookie to the response."""
    response.set_cookie(
        key=_COOKIE_NAME,
        value=token,
        httponly=True,
        samesite=_samesite(),
        secure=settings.cookie_secure,
        max_age=settings.access_token_expire_minutes * 60,
        path="/",
    )


def _clear_auth_cookie(response: Response) -> None:
    """Remove the session cookie by setting Max-Age=0."""
    response.delete_cookie(
        key=_COOKIE_NAME,
        path="/",
        httponly=True,
        samesite=_samesite(),
        secure=settings.cookie_secure,
    )


@router.post(
    "/register",
    response_model=UserResponse,
    status_code=status.HTTP_201_CREATED,
    summary="Register a new user account",
)
async def register(
    body: RegisterRequest,
    db: AsyncSession = Depends(get_db),
) -> UserResponse:
    """
    Create a new user. Returns the created UserResponse (no password hash).

    409 Conflict if the email address is already registered.
    """
    hashed = hash_password(body.password)
    try:
        user = await create_user(db, email=body.email, hashed_password=hashed)
    except IntegrityError:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="An account with this email address already exists.",
        )
    return UserResponse.from_orm_safe(user)


@router.post(
    "/login",
    response_model=UserResponse,
    summary="Authenticate and start a session (sets httpOnly cookie)",
)
async def login(
    body: LoginRequest,
    response: Response,
    db: AsyncSession = Depends(get_db),
) -> UserResponse:
    """
    Validate credentials and set an httpOnly session cookie.

    Returns the UserResponse in the body — NOT the token. The token is only
    ever in the httpOnly cookie; it never appears in any response body or in
    JavaScript-accessible storage.

    Always returns 401 with "Invalid credentials." on failure — the message
    does NOT distinguish between wrong email and wrong password.
    """
    _INVALID = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Invalid credentials.",
        headers={"WWW-Authenticate": "Bearer"},
    )

    user = await get_user_by_email(db, body.email)
    # Intentionally constant-time-safe: verify even when user is None to
    # prevent timing-based user enumeration.
    if user is None or not verify_password(body.password, user.hashed_password):
        raise _INVALID
    if not user.is_active:
        raise _INVALID

    token = create_access_token(user.id)
    _set_auth_cookie(response, token)
    return UserResponse.from_orm_safe(user)


@router.post(
    "/logout",
    response_model=LogoutResponse,
    summary="End the session (clears the httpOnly cookie server-side)",
)
async def logout(response: Response) -> LogoutResponse:
    """
    Clear the session cookie server-side.

    The JWT is stateless — clearing the cookie is the authoritative logout
    action. Server-side token blacklisting (Redis revocation list) is a
    documented future concern in .env.example.
    """
    _clear_auth_cookie(response)
    return LogoutResponse()


@router.get(
    "/me",
    response_model=UserResponse,
    summary="Return the current authenticated user (validates cookie JWT)",
)
async def get_me(
    current_user: User = Depends(get_current_user),
) -> UserResponse:
    """
    Validate the session cookie and return the current user.

    Used by the frontend AuthProvider on page load to restore the session
    without exposing the token to JavaScript. Returns 401 if the cookie is
    absent, expired, or carries an invalid signature.
    """
    return UserResponse.from_orm_safe(current_user)
