"""Session 1 — ml schema: tiered outcome labels + point-in-time training view.

Revision ID: 010
Revises: 009
Create Date: 2026-06-12

MAPPING DECISION (stated): the design docs name a `feasibility_study`
artifact; the existing backend already persists exactly that as
public.assessments (one concept scored against one asset, immutable,
engine-versioned — a protected contract). Rather than duplicate it, the
ml layer treats public.assessments AS the feasibility_study table.
ml.outcome_label.study_id references it.

A4 — labels are tiered: every label carries
label_source ∈ {os_invoiced, partner, manual}; the training view exposes
a per-tier weight so sources are NEVER pooled unweighted. OS-derived
revenue is the self-reported tier (weight 1.0 because it is
transaction-grounded, but still a distinct tier so models can learn
source-specific bias).

A3 — training reads ONLY ml.* views; the view enforces point-in-time
discipline (leakage guard documented inline) and exposes the
tenant+location grouping keys for cross-validation.

Apply:  alembic upgrade head
Revert: alembic downgrade 009
"""

from typing import Union

from alembic import op

revision: str = "010"
down_revision: Union[str, None] = "009"
branch_labels = None
depends_on = None


def upgrade() -> None:
    op.execute(
        """
        CREATE TABLE ml.outcome_label (
            tenant_id            uuid NOT NULL,
            id                   uuid NOT NULL DEFAULT gen_random_uuid(),
            location_id          uuid NOT NULL,
            asset_id             varchar(255) REFERENCES public.assets(id),
            -- the feasibility_study this outcome labels (see mapping decision)
            study_id             varchar(255) REFERENCES public.assessments(id),
            -- the observation window the label aggregates over
            label_window         daterange NOT NULL,
            revenue_actual_egp   numeric,
            transactions_actual  integer,
            -- A4: tiering is a column with a closed domain, not a convention
            label_source         text NOT NULL CHECK (label_source IN
                                     ('os_invoiced','partner','manual')),
            observed_at          timestamptz NOT NULL DEFAULT now(),
            created_at           timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE
        )
        """
    )
    op.execute(
        "CREATE INDEX ix_outcome_label_study ON ml.outcome_label (study_id)"
    )

    op.execute("ALTER TABLE ml.outcome_label ENABLE ROW LEVEL SECURITY")
    op.execute("ALTER TABLE ml.outcome_label FORCE ROW LEVEL SECURITY")
    op.execute(
        """
        CREATE POLICY tenant_isolation ON ml.outcome_label
        USING (tenant_id = ops.current_tenant_id())
        WITH CHECK (tenant_id = ops.current_tenant_id())
        """
    )
    op.execute("GRANT SELECT ON ml.outcome_label TO mahal_app")

    # ── Training role: model training never touches base tables (A3) ────────
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'mahal_ml') THEN
                CREATE ROLE mahal_ml NOLOGIN;
            END IF;
        END $$;
        """
    )
    op.execute("GRANT USAGE ON SCHEMA ml TO mahal_ml")

    op.execute(
        """
        /* ============================================================
           ml.v_feasibility_training — point-in-time training view (A3)

           One row = one labeled feasibility decision:
             a feasibility_study (public.assessments row) made at
             decision_time, joined to the outcome observed AFTER it.

           THE LEAKAGE GUARD, explicitly:

           1. decision_time = a.created_at. Every FEATURE in this view is
              computed over rows STRICTLY BEFORE decision_time:
                - fingerprint_* aggregates filter
                      m.day < a.created_at::date
                  i.e. the operator's "economics fingerprint" is
                  reconstructed exactly as it stood when the decision was
                  made (A3: snapshotted at decision time). The candidate
                  location itself has no pre-decision operating data by
                  definition — its features come from the operator's
                  OTHER, already-active locations.
                - score / coverage_score / engine_version are copied from
                  the immutable assessments row, which was written at
                  decision_time and never updated (assessment.py contract).

           2. The LABEL must postdate the decision:
                  lower(ol.label_window) > a.created_at::date
              is a WHERE predicate, not a convention. A label window that
              overlaps or precedes its decision cannot enter training.

           3. No feature may be computed from data postdating its label:
              the only time-varying features here end strictly before
              decision_time, which itself precedes the label window (by 1
              and 2). Any future feature added to this view MUST keep an
              explicit  < decision_time  predicate — treat that as a
              review-blocking requirement.

           4. Cross-validation grouping (A3): group folds by
              (cv_group_tenant, cv_group_location). Splitting by row leaks
              tenant- and street-level idiosyncrasies across folds and
              inflates validation scores.

           5. Label tiers are never pooled unweighted (A4): label_weight
              encodes the tier; training code multiplies sample weight by
              it (or stratifies by label_source).
           ============================================================ */
        CREATE VIEW ml.v_feasibility_training AS
        SELECT
            a.id                    AS study_id,
            l.tenant_id             AS cv_group_tenant,
            l.id                    AS cv_group_location,
            a.asset_id,
            a.created_at            AS decision_time,
            -- features frozen in the immutable study row
            a.concept_type,
            a.score                 AS predicted_score,
            a.confidence_range_low,
            a.confidence_range_high,
            a.coverage_score,
            a.engine_version,
            -- operator economics fingerprint, point-in-time (guard #1)
            fp.active_locations     AS fingerprint_active_locations,
            fp.avg_daily_revenue    AS fingerprint_avg_daily_revenue,
            fp.avg_margin           AS fingerprint_avg_margin,
            -- the label (guard #2)
            ol.label_window,
            ol.revenue_actual_egp   AS label_revenue,
            ol.transactions_actual  AS label_transactions,
            ol.label_source,
            CASE ol.label_source            -- guard #5 (A4 tier weights)
                WHEN 'os_invoiced' THEN 1.00
                WHEN 'partner'     THEN 0.80
                WHEN 'manual'      THEN 0.50
            END                     AS label_weight
        FROM public.assessments a
        JOIN ops.location l
          ON l.asset_id = a.asset_id
        JOIN ml.outcome_label ol
          ON ol.study_id = a.id
         AND ol.tenant_id = l.tenant_id
         AND ol.location_id = l.id
         AND lower(ol.label_window) > a.created_at::date   -- guard #2
        LEFT JOIN LATERAL (
            SELECT count(DISTINCT m.location_id) AS active_locations,
                   avg(m.revenue)                AS avg_daily_revenue,
                   avg(m.gross_margin)           AS avg_margin
            FROM agg.location_metrics_daily m
            WHERE m.tenant_id = l.tenant_id
              AND m.location_id <> l.id
              AND m.day < a.created_at::date               -- guard #1
        ) fp ON true
        """
    )
    op.execute("COMMENT ON VIEW ml.v_feasibility_training IS "
               "'Point-in-time training view. Features strictly predate "
               "decision_time; labels strictly postdate it; CV groups by "
               "tenant+location; label tiers carry weights (A3/A4). See "
               "migration 010 for the full leakage-guard contract.'")
    op.execute("GRANT SELECT ON ml.v_feasibility_training TO mahal_ml")


def downgrade() -> None:
    op.execute("DROP VIEW IF EXISTS ml.v_feasibility_training")
    op.execute("DROP TABLE IF EXISTS ml.outcome_label")
    op.execute("REVOKE USAGE ON SCHEMA ml FROM mahal_ml")
    op.execute(
        """
        DO $$
        BEGIN
            IF EXISTS (SELECT 1 FROM pg_roles WHERE rolname = 'mahal_ml') THEN
                DROP OWNED BY mahal_ml;
                DROP ROLE mahal_ml;
            END IF;
        END $$;
        """
    )
