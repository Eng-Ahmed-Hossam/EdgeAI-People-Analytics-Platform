"""Session 1 — agg schema: the operations→intelligence contract.

Revision ID: 009
Revises: 008
Create Date: 2026-06-12

agg.location_metrics_daily IS the contract table of design doc 03 §4 /
doc 11 §4: dashboards, the health score, the copilot, and every
intelligence engine read THIS, never the ops fact tables directly
(amendment A2, the OLTP/OLAP boundary). Sensor-derived columns are
nullable: a branch without a Mahal Sense simply has NULL passersby /
conversion / capture — honest absence at the schema level.

agg.district_traffic_index is the cross-tenant "street ticker". The
k-anonymity floor (sample_tenants >= 5) is ENFORCED, not documented:
- the published view filters sample_tenants >= 5, AND
- mahal_app has SELECT on the VIEW only — the base table is not granted.
  (The view is owned by the migration role, so it reads the base table
  with owner privileges; a tenant session physically cannot see an
  under-k row.)

Apply:  alembic upgrade head
Revert: alembic downgrade 008
"""

from typing import Union

from alembic import op

revision: str = "009"
down_revision: Union[str, None] = "008"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.execute(
        """
        CREATE TABLE agg.location_metrics_daily (
            tenant_id           uuid NOT NULL,
            location_id         uuid NOT NULL,
            day                 date NOT NULL,
            -- money (from ops facts)
            revenue             numeric NOT NULL DEFAULT 0,
            transactions        integer NOT NULL DEFAULT 0,
            avg_basket          numeric,
            gross_margin        numeric,
            expenses_fixed      numeric NOT NULL DEFAULT 0,
            expenses_variable   numeric NOT NULL DEFAULT 0,
            -- sensor-derived; NULL when the branch has no paired device
            passersby           integer,
            entries             integer,
            conversion_pct      numeric,
            capture_pct         numeric,
            -- customers
            new_customers       integer NOT NULL DEFAULT 0,
            returning_customers integer NOT NULL DEFAULT 0,
            computed_at         timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, location_id, day),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE
        )
        """
    )

    op.execute("ALTER TABLE agg.location_metrics_daily ENABLE ROW LEVEL SECURITY")
    op.execute("ALTER TABLE agg.location_metrics_daily FORCE ROW LEVEL SECURITY")
    op.execute(
        """
        CREATE POLICY tenant_isolation ON agg.location_metrics_daily
        USING (tenant_id = ops.current_tenant_id())
        WITH CHECK (tenant_id = ops.current_tenant_id())
        """
    )
    # Tenants read their rollups; only the rollup worker (system role) writes.
    op.execute("GRANT SELECT ON agg.location_metrics_daily TO mahal_app")

    # ── The street ticker ────────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE agg.district_traffic_index (
            district_id     uuid NOT NULL REFERENCES ctx.district(id),
            bucket_start    timestamptz NOT NULL,
            index_value     numeric NOT NULL,
            sample_tenants  integer NOT NULL CHECK (sample_tenants >= 1),
            computed_at     timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (district_id, bucket_start)
        )
        """
    )
    op.execute(
        """
        -- k-anonymity floor: only districts with >= 5 contributing tenants
        -- are visible to any tenant-facing surface. Enforced here AND by
        -- privileges (no grant on the base table).
        CREATE VIEW agg.district_traffic_index_published AS
        SELECT district_id, bucket_start, index_value, sample_tenants
        FROM agg.district_traffic_index
        WHERE sample_tenants >= 5
        """
    )
    op.execute("GRANT SELECT ON agg.district_traffic_index_published TO mahal_app")
    # Deliberately NO grant on agg.district_traffic_index itself.


def downgrade() -> None:
    op.execute("DROP VIEW IF EXISTS agg.district_traffic_index_published")
    op.execute("DROP TABLE IF EXISTS agg.district_traffic_index")
    op.execute("DROP TABLE IF EXISTS agg.location_metrics_daily")
