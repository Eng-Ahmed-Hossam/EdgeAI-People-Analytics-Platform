"""
User CRUD operations.

get_user_by_email  — used by the login endpoint to look up a candidate user.
get_user_by_id     — used by get_current_user to validate a JWT's sub claim.
create_user        — used by the register endpoint; accepts a pre-hashed password.

Passwords are NEVER accepted in plaintext here. Callers must hash before passing.
"""

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.user import User


async def get_user_by_email(db: AsyncSession, email: str) -> User | None:
    result = await db.execute(select(User).where(User.email == email))
    return result.scalar_one_or_none()


async def get_user_by_id(db: AsyncSession, user_id: str) -> User | None:
    result = await db.execute(select(User).where(User.id == user_id))
    return result.scalar_one_or_none()


async def create_user(db: AsyncSession, email: str, hashed_password: str) -> User:
    """
    Insert a new user row. Raises IntegrityError if the email is already taken
    (the caller should catch this and return 409 Conflict).

    role is always set to 'user' server-side — the caller cannot influence it.
    """
    user = User(email=email, hashed_password=hashed_password, role="user")
    db.add(user)
    await db.flush()   # assigns id + server-side created_at
    await db.refresh(user)
    return user
