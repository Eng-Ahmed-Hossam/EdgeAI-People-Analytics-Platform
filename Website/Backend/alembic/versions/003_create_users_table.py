"""Create users table for authenticated accounts

Revision ID: 003
Revises: 002
Create Date: 2026-06-10

Creates the users table with: id (UUID string, PK), email (unique, indexed),
hashed_password (bcrypt hash — plaintext never stored), is_active, created_at.

Note: assessments.owner_id is NOT yet FK-constrained to users.id. It remains
String(255), nullable — the 'local-dev' placeholder records stay accessible to
all authenticated users in this session. Auth Session 3 will add the FK
constraint and migrate owner_id values to real user UUIDs.

Apply: alembic upgrade head
Revert: alembic downgrade -1
"""

from typing import Union

import sqlalchemy as sa
from alembic import op

revision: str = "003"
down_revision: Union[str, None] = "002"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "users",
        sa.Column("id", sa.String(255), primary_key=True),
        sa.Column("email", sa.String(255), nullable=False, unique=True),
        sa.Column("hashed_password", sa.String(255), nullable=False),
        sa.Column("is_active", sa.Boolean, nullable=False, server_default="true"),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
    )
    op.create_index("ix_users_email", "users", ["email"], unique=True)


def downgrade() -> None:
    op.drop_index("ix_users_email", table_name="users")
    op.drop_table("users")
