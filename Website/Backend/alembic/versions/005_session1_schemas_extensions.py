"""Session 1 — named schemas, extensions, application role.

Revision ID: 005
Revises: 004
Create Date: 2026-06-12

Creates the named-schema organization required by Gate-1 amendment A6
(dim / edge / agg / ctx / fin / ml + the new `ops` schema for the Mahal
operational core), the spatial/time-series extensions, and the
non-superuser application role (`mahal_app`) that all RLS adversarial
tests run as.

RECORDS NOTE (corrected in Session 1b, Part D): the named-schema
architecture (dim/edge/agg/ctx/fin/ml, SCD-2, the ml leakage boundary)
existed as DESIGN DECISIONS only — there never was a pre-existing "v3"
implementation in any repo (verified by repo-wide search in Session 1).
Migrations 005–010 are that architecture's FIRST materialization, amended
per Gate 1 (A1–A7). Nothing in public is moved or altered (protected
layer).

Extension policy (canonical Postgres major is 18 — founder decision D1,
see docs/adr/001-pg-version-and-timescale.md):
- postgis: REQUIRED — migration fails without it.
- h3 + h3_postgis: REQUIRED — the ops.location H3 generated column needs
  them. Verified live: h3_lat_lng_to_cell(geography, int) is IMMUTABLE
  (provolatile = 'i'), so a GENERATED ALWAYS column is legal.
- timescaledb: OPTIONAL — detected at runtime. Where absent, migration 008
  falls back to native range partitioning, with an identical canonical
  rollup surface (see 008). Both branches are exercised in CI; the
  MAHAL_FORCE_NO_TIMESCALE=1 environment variable (added in Session 1b)
  forces the fallback branch on a Timescale-capable image so the CI matrix
  can run both paths against the same PG18 image (A7).

Downgrade drops the role and the schemas (RESTRICT — only succeeds after
later migrations have downgraded their objects away). Extensions are
deliberately left installed: they are database infrastructure, not
session-1 schema state, and public.assets-era tooling may rely on them.

Apply:  alembic upgrade head
Revert: alembic downgrade 004
"""

import os
from typing import Union

from alembic import op

revision: str = "005"
down_revision: Union[str, None] = "004"
branch_labels = None
depends_on = None

SCHEMAS = ["dim", "edge", "agg", "ctx", "fin", "ml", "ops"]


def upgrade() -> None:
    # ── Extensions ───────────────────────────────────────────────────────────
    op.execute("CREATE EXTENSION IF NOT EXISTS postgis")
    # h3_postgis depends on h3 and postgis_raster; CASCADE resolves both.
    op.execute("CREATE EXTENSION IF NOT EXISTS h3")
    op.execute("CREATE EXTENSION IF NOT EXISTS h3_postgis CASCADE")
    if os.environ.get("MAHAL_FORCE_NO_TIMESCALE") == "1":
        # Session 1b: CI fallback-path job. Skipping the extension here makes
        # migration 008's runtime detection take the native-partitioning
        # branch even on an image that ships timescaledb.
        op.execute(
            "DO $$ BEGIN RAISE NOTICE 'MAHAL_FORCE_NO_TIMESCALE=1 — "
            "fallback branch forced'; END $$;"
        )
    else:
        op.execute(
            """
            DO $$
            BEGIN
                IF EXISTS (SELECT 1 FROM pg_available_extensions
                           WHERE name = 'timescaledb') THEN
                    CREATE EXTENSION IF NOT EXISTS timescaledb;
                ELSE
                    RAISE NOTICE 'timescaledb not available — migration 008 '
                                 'will use native partitioning fallback';
                END IF;
            END $$;
            """
        )

    # ── Named schemas (A6) ───────────────────────────────────────────────────
    for schema in SCHEMAS:
        op.execute(f"CREATE SCHEMA IF NOT EXISTS {schema}")

    # ── Application role ─────────────────────────────────────────────────────
    # NOLOGIN: the API connects as its own login role and SET ROLE mahal_app
    # (or is created as a member). Tests do the same. Roles are cluster-wide,
    # hence the IF NOT EXISTS guard pattern.
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'mahal_app') THEN
                CREATE ROLE mahal_app NOLOGIN;
            END IF;
        END $$;
        """
    )
    for schema in SCHEMAS:
        op.execute(f"GRANT USAGE ON SCHEMA {schema} TO mahal_app")
    # Read access to the existing platform tables (assets is the A1 bridge
    # target). No write grants: the operational core never mutates platform
    # ingestion output.
    op.execute("GRANT USAGE ON SCHEMA public TO mahal_app")
    op.execute("GRANT SELECT ON public.assets, public.assessments TO mahal_app")


def downgrade() -> None:
    op.execute("REVOKE SELECT ON public.assets, public.assessments FROM mahal_app")
    op.execute("REVOKE USAGE ON SCHEMA public FROM mahal_app")
    for schema in reversed(SCHEMAS):
        op.execute(f"REVOKE USAGE ON SCHEMA {schema} FROM mahal_app")
        op.execute(f"DROP SCHEMA IF EXISTS {schema} RESTRICT")
    op.execute(
        """
        DO $$
        BEGIN
            IF EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'mahal_app') THEN
                DROP OWNED BY mahal_app;
                DROP ROLE mahal_app;
            END IF;
        END $$;
        """
    )
    # Extensions intentionally retained — see module docstring.
