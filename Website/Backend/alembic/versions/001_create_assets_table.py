"""Create assets table

Revision ID: 001
Revises:
Create Date: 2026-06-09

Apply: alembic upgrade head
Revert: alembic downgrade -1
"""

from typing import Union

import sqlalchemy as sa
from alembic import op

revision: str = "001"
down_revision: Union[str, None] = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.create_table(
        "assets",
        # ── Identity ──────────────────────────────────────────────────────────
        sa.Column("id", sa.String(255), primary_key=True),

        # ── Spatial ───────────────────────────────────────────────────────────
        sa.Column("latitude", sa.Float, nullable=False),
        sa.Column("longitude", sa.Float, nullable=False),

        # ── Location descriptors ──────────────────────────────────────────────
        sa.Column("address", sa.String(500), nullable=False),
        sa.Column("area", sa.String(200), nullable=False),
        sa.Column("size", sa.Float, nullable=True),

        # ── Commercial terms ──────────────────────────────────────────────────
        sa.Column("monthly_rent", sa.Float, nullable=True),
        sa.Column("sale_price", sa.Float, nullable=True),
        sa.Column("currency", sa.String(10), nullable=False, server_default="EGP"),

        # ── Classification ────────────────────────────────────────────────────
        sa.Column("property_type", sa.String(50), nullable=False, server_default="retail_unit"),
        sa.Column("availability_status", sa.String(20), nullable=False, server_default="available"),

        # ── Provenance ────────────────────────────────────────────────────────
        sa.Column("listing_source", sa.String(30), nullable=False, server_default="other"),
        sa.Column("source", sa.String(100), nullable=True),
        sa.Column("source_listing_id", sa.String(255), nullable=True),
        sa.Column("source_url", sa.Text, nullable=True),
        sa.Column("first_seen_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("last_seen_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column(
            "ingestion_timestamp",
            sa.DateTime(timezone=True),
            nullable=True,
            server_default=sa.func.now(),
        ),

        # ── Intelligence readiness ────────────────────────────────────────────
        sa.Column("coverage_score", sa.Integer, nullable=False, server_default="0"),

        # ── Soft-delete ───────────────────────────────────────────────────────
        sa.Column("is_active", sa.Boolean, nullable=False, server_default="true"),
    )

    op.create_unique_constraint(
        "uq_asset_source_listing",
        "assets",
        ["source", "source_listing_id"],
    )

    op.create_index(
        "ix_assets_status_active",
        "assets",
        ["availability_status", "is_active"],
    )

    op.create_index(
        "ix_assets_lat_lng",
        "assets",
        ["latitude", "longitude"],
    )


def downgrade() -> None:
    op.drop_index("ix_assets_lat_lng", table_name="assets")
    op.drop_index("ix_assets_status_active", table_name="assets")
    op.drop_constraint("uq_asset_source_listing", "assets", type_="unique")
    op.drop_table("assets")
