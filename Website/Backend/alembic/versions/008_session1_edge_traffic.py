"""Session 1 — edge schema: device registry, traffic_sample, rollups.

Revision ID: 008
Revises: 007
Create Date: 2026-06-12
Edited: Session 1b (rollup isolation fix + records hygiene) — see EDIT LOG.

CANONICAL ORIGIN (A6, reconciled in Session 1b Part C): edge.device_registry
is the canonical device-identity contract, originating HERE. The
EdgeAI-People-Analytics-Platform repo was searched (SQL/DDL, ORM models,
firmware, MQTT config, docs) and contains NO device registry, provisioning,
or device-identity definition — its telemetry prototype publishes
unauthenticated, device-anonymous JSON to a public broker, and its DBMS v1
schema keys sensor data on bare location_id. The hardware repo must adopt
THIS contract for device provisioning (integration requirement for the
sensor-ingest session). Data-quality compatibility with DBMS v1 is kept:
traffic_sample carries both quality (0–100) and the v1-style
data_quality_flag smallint (DEFAULT 0 = clean).

RECORDS NOTE (Session 1b Part D): the named-schema architecture this
migration participates in (dim/edge/agg/ctx/fin/ml) existed as DESIGN
DECISIONS only before Session 1 — there was no pre-existing "v3"
implementation anywhere. Migrations 005–010 are its first materialization,
amended per Gate 1 (A1–A7).

ROLLUP BRANCHES (A7) — detected at runtime, verified on both paths:
- timescaledb PRESENT  → edge.traffic_sample becomes a hypertable;
  continuous aggregates are created as INTERNAL objects
  (edge.traffic_15m_cagg / _1h_cagg / _1d_cagg) with refresh policies.
  create_hypertable() runs inside the migration transaction (transactional
  in TS 2.x — verified live on 2.26.4); the continuous aggregates and
  their policies run in an explicit autocommit block because they cannot
  run inside a transaction (verified: the ladder passes with exactly this
  scoping).
- timescaledb ABSENT → native monthly RANGE partitions on traffic_sample.

TENANT ISOLATION OF ROLLUPS (Session 1b Part B — the fix):
Continuous aggregates are materialized data; RLS cannot attach to them.
The original Session-1 design gave the two branches DIFFERENT semantics
(fallback: security_invoker views inheriting RLS; timescale: no tenant
access at all — and the naive GRANT was proven to leak cross-tenant rows
on PG18/TS 2.26.4). Fix: BOTH branches now expose the canonical surface
edge.traffic_15m / traffic_1h / traffic_1d as owner-privileged views with
an EXPLICIT tenant predicate
    WHERE tenant_id = ops.current_tenant_id()
which replicates the RLS policy expression exactly (unset GUC → NULL →
zero rows). mahal_app is granted SELECT on these canonical views ONLY.
The underlying objects (the _cagg aggregates / the raw hypertable or
partitioned table) carry no tenant grants; system workers (rollup,
district-index builders) read them directly under a system role. This was
chosen over cagg-level filtering because it makes the two branches
identical BY CONSTRUCTION — one mechanism, one predicate, same names,
same column signatures (proven by catalog diff in the Session 1b record).

EDIT LOG (Session 1b): canonical rollup names converted from
branch-dependent objects (caggs vs security_invoker views) to uniform
tenant-filtering views; caggs renamed *_cagg; grants moved to the views;
docstring rewritten per Parts B/C/D. Defect that justified the edit:
provably non-identical isolation semantics between branches (Part B
before-probe: permission denied vs RLS-scoped reads; post-GRANT leak of
2 tenants / 72 buckets).

Apply:  alembic upgrade head
Revert: alembic downgrade 007
"""

from typing import Union

import sqlalchemy as sa
from alembic import op

revision: str = "008"
down_revision: Union[str, None] = "007"
branch_labels = None
depends_on = None

PARTITION_MONTHS = [
    ("p2026_04", "2026-04-01", "2026-05-01"),
    ("p2026_05", "2026-05-01", "2026-06-01"),
    ("p2026_06", "2026-06-01", "2026-07-01"),
    ("p2026_07", "2026-07-01", "2026-08-01"),
]

ROLLUPS = [  # (canonical name, bucket step)
    ("traffic_15m", "15 minutes"),
    ("traffic_1h", "1 hour"),
    ("traffic_1d", "1 day"),
]

# The one column list, used by every rollup object on both branches.
ROLLUP_COLS = (
    "tenant_id, location_id, device_id, bucket, passersby, entries, exits, "
    "quality_avg, data_quality_flag, samples"
)


def _timescale_available() -> bool:
    return bool(
        op.get_bind()
        .execute(sa.text("SELECT 1 FROM pg_extension WHERE extname = 'timescaledb'"))
        .scalar()
    )


def upgrade() -> None:
    # ── Canonical device registry (origin: this migration; see docstring) ───
    op.execute(
        """
        CREATE TABLE edge.device_registry (
            id                   uuid PRIMARY KEY DEFAULT gen_random_uuid(),
            serial_number        text NOT NULL UNIQUE,
            model                text NOT NULL DEFAULT 'mahal-sense-1',
            firmware_version     text,
            claimed_tenant_id    uuid REFERENCES ops.tenant(id),
            claimed_location_id  uuid,
            claimed_at           timestamptz,
            status               text NOT NULL DEFAULT 'unclaimed' CHECK (status IN
                                     ('unclaimed','live','degraded','offline','retired')),
            last_heartbeat_at    timestamptz,
            created_at           timestamptz NOT NULL DEFAULT now(),
            -- A claim binds device → (tenant, location) atomically (doc 09 §2).
            CHECK ((claimed_tenant_id IS NULL) = (claimed_location_id IS NULL)),
            FOREIGN KEY (claimed_tenant_id, claimed_location_id)
                REFERENCES ops.location (tenant_id, id)
        )
        """
    )

    # ── traffic_sample — the sensor stream (doc 03 §3.3) ────────────────────
    # PK includes bucket_start: required by BOTH the hypertable contract and
    # the native-partitioning fallback (review defect #1 class).
    op.execute(
        """
        CREATE TABLE edge.traffic_sample (
            tenant_id          uuid NOT NULL,
            location_id        uuid NOT NULL,
            device_id          uuid NOT NULL REFERENCES edge.device_registry(id),
            bucket_start       timestamptz NOT NULL,
            passersby          integer NOT NULL CHECK (passersby >= 0),
            entries            integer NOT NULL CHECK (entries >= 0),
            exits              integer NOT NULL CHECK (exits >= 0),
            dwell_avg_s        integer,
            -- Device-reported confidence 0–100 (doc 09 §3).
            quality            smallint NOT NULL CHECK (quality BETWEEN 0 AND 100),
            -- v1 DBMS compatibility (RealTime_Metrics.data_quality_flag):
            -- 0 = clean, non-zero = flagged by the quality pipeline.
            data_quality_flag  smallint NOT NULL DEFAULT 0,
            seq                bigint,
            PRIMARY KEY (tenant_id, location_id, device_id, bucket_start),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id)
        )
        """
        + ("" if _timescale_available() else " PARTITION BY RANGE (bucket_start)")
    )

    if _timescale_available():
        # Transactional in TimescaleDB 2.x — stays inside the migration txn.
        op.execute(
            """
            SELECT create_hypertable('edge.traffic_sample', 'bucket_start',
                                     chunk_time_interval => interval '7 days')
            """
        )
        # Continuous aggregates + policies cannot run inside a transaction —
        # autocommit block scoped to exactly these statements (A7, verified).
        with op.get_context().autocommit_block():
            for view, step in ROLLUPS:
                op.execute(
                    f"""
                    CREATE MATERIALIZED VIEW edge.{view}_cagg
                    -- materialized_only=false: real-time aggregation above
                    -- the watermark, so freshness semantics match the
                    -- fallback branch's always-live views (Part-B parity).
                    -- Historical backfills still need an explicit
                    -- refresh_continuous_aggregate (standard Timescale
                    -- practice; the seed script does this).
                    WITH (timescaledb.continuous,
                          timescaledb.materialized_only = false) AS
                    SELECT tenant_id, location_id, device_id,
                           time_bucket(interval '{step}', bucket_start) AS bucket,
                           sum(passersby) AS passersby,
                           sum(entries)   AS entries,
                           sum(exits)     AS exits,
                           avg(quality)::numeric(5,2) AS quality_avg,
                           max(data_quality_flag)     AS data_quality_flag,
                           count(*)                   AS samples
                    FROM edge.traffic_sample
                    GROUP BY tenant_id, location_id, device_id, 4
                    WITH NO DATA
                    """
                )
                op.execute(
                    f"""
                    SELECT add_continuous_aggregate_policy('edge.{view}_cagg',
                        start_offset      => interval '3 days',
                        end_offset        => interval '15 minutes',
                        schedule_interval => interval '15 minutes')
                    """
                )
        # Canonical tenant surface: same names, same columns as the fallback
        # branch; explicit tenant predicate = the Part-B isolation mechanism.
        for view, _ in ROLLUPS:
            op.execute(
                f"""
                CREATE VIEW edge.{view} AS
                SELECT {ROLLUP_COLS}
                FROM edge.{view}_cagg
                WHERE tenant_id = ops.current_tenant_id()
                """
            )
            op.execute(f"GRANT SELECT ON edge.{view} TO mahal_app")
        # Deliberately NO grant on the *_cagg objects.
    else:
        for suffix, lo, hi in PARTITION_MONTHS:
            op.execute(
                f"CREATE TABLE edge.traffic_sample_{suffix} "
                f"PARTITION OF edge.traffic_sample "
                f"FOR VALUES FROM ('{lo}') TO ('{hi}')"
            )
        op.execute(
            "CREATE TABLE edge.traffic_sample_default "
            "PARTITION OF edge.traffic_sample DEFAULT"
        )
        for view, step in ROLLUPS:
            op.execute(
                f"""
                CREATE VIEW edge.{view} AS
                SELECT tenant_id, location_id, device_id,
                       date_bin(interval '{step}', bucket_start,
                                timestamptz '2000-01-01') AS bucket,
                       sum(passersby) AS passersby,
                       sum(entries)   AS entries,
                       sum(exits)     AS exits,
                       avg(quality)::numeric(5,2) AS quality_avg,
                       max(data_quality_flag)     AS data_quality_flag,
                       count(*)                   AS samples
                FROM edge.traffic_sample
                WHERE tenant_id = ops.current_tenant_id()
                GROUP BY tenant_id, location_id, device_id, 4
                """
            )
            op.execute(f"GRANT SELECT ON edge.{view} TO mahal_app")

    op.execute(
        "CREATE INDEX ix_traffic_branch_time "
        "ON edge.traffic_sample (tenant_id, location_id, bucket_start)"
    )

    # ── RLS on the raw stream ────────────────────────────────────────────────
    op.execute("ALTER TABLE edge.traffic_sample ENABLE ROW LEVEL SECURITY")
    op.execute("ALTER TABLE edge.traffic_sample FORCE ROW LEVEL SECURITY")
    op.execute(
        """
        CREATE POLICY tenant_isolation ON edge.traffic_sample
        USING (tenant_id = ops.current_tenant_id())
        WITH CHECK (tenant_id = ops.current_tenant_id())
        """
    )
    op.execute("ALTER TABLE edge.device_registry ENABLE ROW LEVEL SECURITY")
    op.execute("ALTER TABLE edge.device_registry FORCE ROW LEVEL SECURITY")
    op.execute(
        """
        CREATE POLICY tenant_claimed_devices ON edge.device_registry
        USING (claimed_tenant_id = ops.current_tenant_id())
        WITH CHECK (claimed_tenant_id = ops.current_tenant_id())
        """
    )

    op.execute("GRANT SELECT, INSERT ON edge.traffic_sample TO mahal_app")
    op.execute("GRANT SELECT, UPDATE ON edge.device_registry TO mahal_app")


def downgrade() -> None:
    for view, _ in ROLLUPS:
        op.execute(f"DROP VIEW IF EXISTS edge.{view}")
    if _timescale_available():
        with op.get_context().autocommit_block():
            for view, _ in ROLLUPS:
                op.execute(f"DROP MATERIALIZED VIEW IF EXISTS edge.{view}_cagg")
    op.execute("DROP TABLE IF EXISTS edge.traffic_sample CASCADE")
    op.execute("DROP TABLE IF EXISTS edge.device_registry CASCADE")
