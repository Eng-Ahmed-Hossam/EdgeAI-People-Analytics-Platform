"""
User ORM model.

Stores registered user accounts. Passwords are NEVER stored in plaintext —
only bcrypt hashes. The hashed_password column is never included in any
API response schema. The role column is likewise excluded from all response
schemas — it is an internal authorization detail, not identity data.

Roles:
  'user'  — default; all registrations receive this automatically.
  'admin' — assigned directly in the database (no promotion endpoint yet).

owner_id on the Assessment model uses users.id as its value after Auth Session 3.
"""

import uuid

from sqlalchemy import Boolean, Column, DateTime, String, func

from app.database import Base


class User(Base):
    __tablename__ = "users"

    id = Column(String(255), primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String(255), unique=True, nullable=False, index=True)
    hashed_password = Column(String(255), nullable=False)
    is_active = Column(Boolean, nullable=False, default=True)
    # Role is server-assigned; never accepted from the client.
    # 'user' (default) or 'admin' (DB-assigned only).
    role = Column(String(20), nullable=False, default="user")
    created_at = Column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )
