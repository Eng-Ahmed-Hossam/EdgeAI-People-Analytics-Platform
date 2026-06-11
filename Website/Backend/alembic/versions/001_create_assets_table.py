"""Create assets table with PostGIS geometry column

Revision ID: 001
Revises:
Create Date: 2026-06-09

Prerequisites:
  - PostgreSQL with PostGIS extension installed
  - Run `CREATE EXTENSION IF NOT EXISTS postgis;` on the target DB before applying.

Apply: alembic upgrade head
Revert: alembic downgrade -1
"""

from typing import Union

import sqlalchemy as sa
from alembic import op
from geoalchemy2 import Geometry

revision: str = "001"
down_revision: Union[str, None] = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # PostGIS extension must exist before creating a geometry column
    op.execute("CREATE EXTENSION IF NOT EXISTS postgis")
    op.execute("CREATE EXTENSION IF NOT EXISTS postgis_topology")

    op.create_table(
        "assets",
        # ── Identity ──────────────────────────────────────────────────────────
        sa.Column("id", sa.String(255), primary_key=True),

        # ── Spatial ───────────────────────────────────────────────────────────
        # POINT in WGS84 (SRID 4326). GeoAlchemy2 registers the geometry column
        # with PostGIS and creates a spatial index automatically.
        sa.Column(
            "location",
            Geometry("POINT", srid=4326, spatial_index=True),
            nullable=False,
        ),

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
        # Allowed: available | reserved | occupied | sold | expired
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

    # Deduplication constraint: same source cannot have two records for the same listing
    op.create_unique_constraint(
        "uq_asset_source_listing",
        "assets",
        ["source", "source_listing_id"],
    )

    # Composite index for the most common API query pattern
    op.create_index(
        "ix_assets_status_active",
        "assets",
        ["availability_status", "is_active"],
    )


def downgrade() -> None:
    op.drop_index("ix_assets_status_active", table_name="assets")
    op.drop_constraint("uq_asset_source_listing", "assets", type_="unique")
    op.drop_table("assets")
