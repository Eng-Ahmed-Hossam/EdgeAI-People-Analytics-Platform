"""Add role column to users table

Revision ID: 004
Revises: 003
Create Date: 2026-06-10

Adds a role column to the users table with a NOT NULL constraint and a
server-side default of 'user'. All existing rows receive 'user' automatically.

Two valid values for now: 'user' (default, all registrations) and 'admin'
(assigned directly in the database; no promotion endpoint yet — future concern).

Apply:  alembic upgrade head
Revert: alembic downgrade -1
"""

from typing import Union

import sqlalchemy as sa
from alembic import op

revision: str = "004"
down_revision: Union[str, None] = "003"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.add_column(
        "users",
        sa.Column(
            "role",
            sa.String(20),
            nullable=False,
            server_default="user",
        ),
    )


def downgrade() -> None:
    op.drop_column("users", "role")
