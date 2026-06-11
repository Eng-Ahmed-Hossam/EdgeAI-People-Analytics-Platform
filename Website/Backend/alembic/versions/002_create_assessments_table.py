"""Create assessments table for persisted feasibility assessments

Revision ID: 002
Revises: 001
Create Date: 2026-06-10

One row per ConceptAssessment (a single business concept scored against a single
asset). FK to assets.id with ON DELETE CASCADE. Indexed for retrieval by owner,
by asset, and by created_at (history).

Apply: alembic upgrade head
Revert: alembic downgrade -1
"""

from typing import Union

import sqlalchemy as sa
from alembic import op
from sqlalchemy.dialects.postgresql import JSONB

revision: str = "002"
down_revision: Union[str, None] = "001"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "assessments",
        # ── Identity ──────────────────────────────────────────────────────────
        sa.Column("id", sa.String(255), primary_key=True),

        # ── Relationships ─────────────────────────────────────────────────────
        sa.Column(
            "asset_id",
            sa.String(255),
            sa.ForeignKey("assets.id", ondelete="CASCADE"),
            nullable=False,
        ),
        # Ownership-ready; nullable during the no-auth phase.
        sa.Column("owner_id", sa.String(255), nullable=True),

        # ── Concept ───────────────────────────────────────────────────────────
        sa.Column("concept_type", sa.String(50), nullable=False),

        # ── Reproducibility ───────────────────────────────────────────────────
        sa.Column("engine_version", sa.String(50), nullable=False),

        # ── Denormalised queryable scalars ────────────────────────────────────
        sa.Column("score", sa.Integer, nullable=False),
        sa.Column("success_likelihood", sa.Float, nullable=False),
        sa.Column("score_band", sa.String(20), nullable=False),
        sa.Column("confidence", sa.String(20), nullable=False),
        sa.Column("confidence_range_low", sa.Integer, nullable=False),
        sa.Column("confidence_range_high", sa.Integer, nullable=False),
        sa.Column("coverage_score", sa.Integer, nullable=False),
        sa.Column("data_point_count", sa.Integer, nullable=False),

        # ── Structured engine outputs as JSON ─────────────────────────────────
        sa.Column("drivers_json", JSONB, nullable=False),
        sa.Column("risks_json", JSONB, nullable=False),
        sa.Column("explanation_json", JSONB, nullable=False),

        # ── Authoritative complete output (round-trip source) ─────────────────
        sa.Column("feasibility_json", JSONB, nullable=False),

        # ── Timestamps ────────────────────────────────────────────────────────
        sa.Column("generated_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
    )

    op.create_index("ix_assessments_asset_id", "assessments", ["asset_id"])
    op.create_index("ix_assessments_owner_id", "assessments", ["owner_id"])
    op.create_index("ix_assessments_created_at", "assessments", ["created_at"])
    op.create_index(
        "ix_assessments_owner_created", "assessments", ["owner_id", "created_at"]
    )


def downgrade() -> None:
    op.drop_index("ix_assessments_owner_created", table_name="assessments")
    op.drop_index("ix_assessments_created_at", table_name="assessments")
    op.drop_index("ix_assessments_owner_id", table_name="assessments")
    op.drop_index("ix_assessments_asset_id", table_name="assessments")
    op.drop_table("assessments")
