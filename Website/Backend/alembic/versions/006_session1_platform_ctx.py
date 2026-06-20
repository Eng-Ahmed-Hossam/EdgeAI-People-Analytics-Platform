"""Session 1 — platform/context layer: district, poi, asset confirmation.

Revision ID: 006
Revises: 005
Create Date: 2026-06-12

ctx.* holds cross-tenant geographic context (no tenant_id anywhere in this
schema — that is the point). The tenant-facing k-anonymity surface
(agg.district_traffic_index + its published view) lands in migration 009
with the rest of the agg layer.

Asset confirmation (A1): public.assets is the platform-layer asset entity
and is a protected contract — this migration only VERIFIES it exists with
the columns the ops bridge needs (id, latitude, longitude). It changes
nothing on it. ops.location.asset_id (007) references it.

Apply:  alembic upgrade head
Revert: alembic downgrade 005
"""

from typing import Union

from alembic import op

revision: str = "006"
down_revision: Union[str, None] = "005"
branch_labels = None
depends_on = None


def upgrade() -> None:
    # ── A1 confirmation: the asset bridge target must exist, untouched ──────
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (
                SELECT 1 FROM information_schema.columns
                WHERE table_schema = 'public' AND table_name = 'assets'
                  AND column_name IN ('id', 'latitude', 'longitude')
                GROUP BY table_name HAVING count(*) = 3
            ) THEN
                RAISE EXCEPTION
                  'public.assets (id, latitude, longitude) not found — '
                  'migrations 001-004 must run first; the asset bridge (A1) '
                  'has no target.';
            END IF;
        END $$;
        """
    )

    # ── ctx.district ─────────────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ctx.district (
            id           uuid PRIMARY KEY DEFAULT gen_random_uuid(),
            name         text NOT NULL,
            name_ar      text,
            governorate  text NOT NULL,
            geom         geography(MultiPolygon, 4326) NOT NULL,
            -- H3 res-8 cells covering the district; the cheap join key
            -- between tenant locations (ops.location.h3_r8) and context.
            h3_cells     h3index[] NOT NULL DEFAULT '{}',
            created_at   timestamptz NOT NULL DEFAULT now(),
            UNIQUE (governorate, name)
        )
        """
    )
    op.execute("CREATE INDEX ix_district_geom ON ctx.district USING gist (geom)")

    # ── ctx.poi — points of interest (EdgeAI ingestion target) ───────────────
    op.execute(
        """
        CREATE TABLE ctx.poi (
            id           uuid PRIMARY KEY DEFAULT gen_random_uuid(),
            category     text NOT NULL,       -- pharmacy | cafe | bank | transit | anchor | ...
            brand        text,
            name         text,
            geog         geography(Point, 4326) NOT NULL,
            h3_r8        h3index GENERATED ALWAYS AS (h3_lat_lng_to_cell(geog, 8)) STORED,
            district_id  uuid REFERENCES ctx.district(id),
            source       text NOT NULL DEFAULT 'osm',
            observed_at  timestamptz NOT NULL DEFAULT now()
        )
        """
    )
    op.execute("CREATE INDEX ix_poi_geog ON ctx.poi USING gist (geog)")
    op.execute("CREATE INDEX ix_poi_h3 ON ctx.poi (h3_r8)")
    op.execute("CREATE INDEX ix_poi_category ON ctx.poi (category)")

    op.execute("GRANT SELECT ON ctx.district, ctx.poi TO mahal_app")


def downgrade() -> None:
    op.execute("DROP TABLE IF EXISTS ctx.poi")
    op.execute("DROP TABLE IF EXISTS ctx.district")
