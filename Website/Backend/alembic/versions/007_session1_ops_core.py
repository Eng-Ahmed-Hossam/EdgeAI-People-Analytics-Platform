"""Session 1 — ops schema: the Mahal operational core.

Revision ID: 007
Revises: 006
Create Date: 2026-06-12

Implements design doc 03 corrected by the Gate-1 amendments:

- A1  ops.location carries a NULLABLE asset_id → public.assets(id).
      Candidate→active "birth certificate" flow rides this bridge.
- A5  SCD-2: economically meaningful mutable attributes of location
      (name, category, rent_egp_month, operating_hours, status) are
      history-tracked in ops.location_history via trigger. No silent
      overwrites.
- A7 / review defect #1: every partitioned table includes its partition
      key in the PRIMARY KEY:
        invoice  PK (tenant_id, id, issued_at)   PARTITION BY RANGE (issued_at)
        payment  PK (tenant_id, id, paid_at)     PARTITION BY RANGE (paid_at)
        expense  PK (tenant_id, id, incurred_at) PARTITION BY RANGE (incurred_at)
      invoice_line therefore carries (invoice_id, invoice_issued_at) and its
      FK targets the full invoice PK.
- Review defect #2: the H3 column is a true GENERATED ALWAYS column.
      Verified live on PG18 + h3_postgis 4.1.4: h3_lat_lng_to_cell(geography,
      integer) has provolatile = 'i' (IMMUTABLE), so the generated expression
      is legal. No trigger fallback is needed.

RLS: every tenant-scoped ops table gets ENABLE + FORCE ROW LEVEL SECURITY
with a tenant_isolation policy keyed on the app.tenant_id GUC. The single
documented exception is ops.app_user, which deliberately has NO tenant_id:
identity is cross-tenant (one human, several tenants via ops.membership).
Branch (location_scope) authorization is a policy-layer concern per design
doc 04 §4 — NOT implemented in RLS, by design.

Grants go to mahal_app on PARENT tables only. Direct access to a partition
(e.g. ops.invoice_p2026_06) fails with permission denied — adversarially
tested.

Apply:  alembic upgrade head
Revert: alembic downgrade 006
"""

from typing import Union

from alembic import op

revision: str = "007"
down_revision: Union[str, None] = "006"
branch_labels = None
depends_on = None

# Monthly partitions covering the seed window (30 days back from 2026-06-12)
# plus headroom, plus a DEFAULT partition so no insert can ever fail on a
# missing partition. Real deployments extend this via a scheduled job.
PARTITION_MONTHS = [
    ("p2026_04", "2026-04-01", "2026-05-01"),
    ("p2026_05", "2026-05-01", "2026-06-01"),
    ("p2026_06", "2026-06-01", "2026-07-01"),
    ("p2026_07", "2026-07-01", "2026-08-01"),
]

# (table, tenant column) — every ops table under RLS. app_user is the
# documented exception (no tenant_id; see module docstring).
RLS_TABLES = [
    ("ops.tenant", "id"),
    ("ops.membership", "tenant_id"),
    ("ops.location", "tenant_id"),
    ("ops.location_history", "tenant_id"),
    ("ops.product", "tenant_id"),
    ("ops.price", "tenant_id"),
    ("ops.customer", "tenant_id"),
    ("ops.customer_location_affinity", "tenant_id"),
    ("ops.invoice", "tenant_id"),
    ("ops.invoice_line", "tenant_id"),
    ("ops.payment", "tenant_id"),
    ("ops.expense", "tenant_id"),
    ("ops.stock_level", "tenant_id"),
    ("ops.stock_move", "tenant_id"),
    ("ops.followup_task", "tenant_id"),
    ("ops.whatsapp_thread", "tenant_id"),
    ("ops.whatsapp_message", "tenant_id"),
]


def _make_partitions(table: str, parent: str) -> None:
    for suffix, lo, hi in PARTITION_MONTHS:
        op.execute(
            f"CREATE TABLE ops.{table}_{suffix} PARTITION OF {parent} "
            f"FOR VALUES FROM ('{lo}') TO ('{hi}')"
        )
    op.execute(f"CREATE TABLE ops.{table}_default PARTITION OF {parent} DEFAULT")


def upgrade() -> None:
    # ── Tenancy & identity ───────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ops.tenant (
            id                 uuid PRIMARY KEY DEFAULT gen_random_uuid(),
            name               text NOT NULL,
            business_category  text NOT NULL,
            created_at         timestamptz NOT NULL DEFAULT now()
        )
        """
    )

    op.execute(
        """
        -- Cross-tenant identity. Deliberately NO tenant_id (one human may
        -- belong to several tenants); tenancy attaches via ops.membership.
        -- This is the single ops table outside RLS — documented exception.
        CREATE TABLE ops.app_user (
            id          uuid PRIMARY KEY DEFAULT gen_random_uuid(),
            email       text NOT NULL UNIQUE,
            full_name   text NOT NULL,
            phone_e164  text,
            created_at  timestamptz NOT NULL DEFAULT now()
        )
        """
    )

    op.execute(
        """
        CREATE TABLE ops.membership (
            tenant_id       uuid NOT NULL REFERENCES ops.tenant(id) ON DELETE CASCADE,
            user_id         uuid NOT NULL REFERENCES ops.app_user(id) ON DELETE CASCADE,
            role            text NOT NULL CHECK (role IN
                                ('owner','manager','staff','accountant','analyst')),
            -- NULL = ALL locations; otherwise an explicit allow-list.
            -- Enforced by the API policy layer (doc 04 §4), not by RLS.
            location_scope  uuid[],
            created_at      timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, user_id)
        )
        """
    )

    # ── Location — the atomic unit (doc 03 §3.1, amended A1/A5) ─────────────
    op.execute(
        """
        CREATE TABLE ops.location (
            tenant_id        uuid NOT NULL REFERENCES ops.tenant(id) ON DELETE CASCADE,
            id               uuid NOT NULL DEFAULT gen_random_uuid(),
            name             text NOT NULL,
            name_ar          text,
            geog             geography(Point, 4326) NOT NULL,
            -- Review defect #2 fix: TRUE generated column. Legal because
            -- h3_lat_lng_to_cell(geography, int) is IMMUTABLE (verified
            -- live: provolatile = 'i' on h3_postgis 4.1.4).
            h3_r8            h3index GENERATED ALWAYS AS
                                 (h3_lat_lng_to_cell(geog, 8)) STORED,
            district_id      uuid REFERENCES ctx.district(id),
            -- A1: the asset bridge. NULLABLE — an active branch may predate
            -- any listed asset; a candidate created from a feasibility study
            -- carries the asset it was studied against.
            asset_id         varchar(255) REFERENCES public.assets(id),
            address          jsonb,
            category         text NOT NULL,
            area_m2          numeric,
            rent_egp_month   numeric,
            opened_at        date,
            operating_hours  jsonb,
            eta_branch_code  text,
            status           text NOT NULL DEFAULT 'active' CHECK (status IN
                                 ('active','paused','closed','candidate','hq')),
            created_at       timestamptz NOT NULL DEFAULT now(),
            updated_at       timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id)
        )
        """
    )
    op.execute("CREATE INDEX ix_location_geog ON ops.location USING gist (geog)")
    op.execute("CREATE INDEX ix_location_h3 ON ops.location (h3_r8)")
    op.execute("CREATE INDEX ix_location_asset ON ops.location (asset_id)")

    # A5: SCD-2 attribute history. One open row (valid_to IS NULL) per
    # location; closed rows are the audit trail of economic attributes.
    op.execute(
        """
        CREATE TABLE ops.location_history (
            tenant_id        uuid NOT NULL,
            location_id      uuid NOT NULL,
            valid_from       timestamptz NOT NULL,
            valid_to         timestamptz,
            name             text NOT NULL,
            category         text NOT NULL,
            rent_egp_month   numeric,
            operating_hours  jsonb,
            status           text NOT NULL,
            PRIMARY KEY (tenant_id, location_id, valid_from),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE
        )
        """
    )

    op.execute(
        """
        -- clock_timestamp(), not now(): two tracked updates inside one
        -- transaction must not collide on the (tenant, location, valid_from)
        -- PK, and history must record real wall-clock ordering.
        CREATE FUNCTION ops.location_track_history() RETURNS trigger
        LANGUAGE plpgsql AS $$
        BEGIN
            IF TG_OP = 'INSERT' THEN
                INSERT INTO ops.location_history
                    (tenant_id, location_id, valid_from, valid_to,
                     name, category, rent_egp_month, operating_hours, status)
                VALUES
                    (NEW.tenant_id, NEW.id, clock_timestamp(), NULL,
                     NEW.name, NEW.category, NEW.rent_egp_month,
                     NEW.operating_hours, NEW.status);
            ELSIF TG_OP = 'UPDATE' AND (
                   NEW.name            IS DISTINCT FROM OLD.name
                OR NEW.category        IS DISTINCT FROM OLD.category
                OR NEW.rent_egp_month  IS DISTINCT FROM OLD.rent_egp_month
                OR NEW.operating_hours IS DISTINCT FROM OLD.operating_hours
                OR NEW.status          IS DISTINCT FROM OLD.status
            ) THEN
                UPDATE ops.location_history
                   SET valid_to = clock_timestamp()
                 WHERE tenant_id = NEW.tenant_id
                   AND location_id = NEW.id
                   AND valid_to IS NULL;
                INSERT INTO ops.location_history
                    (tenant_id, location_id, valid_from, valid_to,
                     name, category, rent_egp_month, operating_hours, status)
                VALUES
                    (NEW.tenant_id, NEW.id, clock_timestamp(), NULL,
                     NEW.name, NEW.category, NEW.rent_egp_month,
                     NEW.operating_hours, NEW.status);
            END IF;
            RETURN NEW;
        END $$;
        """
    )
    op.execute(
        """
        CREATE TRIGGER trg_location_history
        AFTER INSERT OR UPDATE ON ops.location
        FOR EACH ROW EXECUTE FUNCTION ops.location_track_history()
        """
    )

    # ── Catalog ──────────────────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ops.product (
            tenant_id      uuid NOT NULL REFERENCES ops.tenant(id) ON DELETE CASCADE,
            id             uuid NOT NULL DEFAULT gen_random_uuid(),
            sku            text NOT NULL,
            name           text NOT NULL,
            name_ar        text,
            category       text,
            unit_cost_egp  numeric NOT NULL DEFAULT 0,
            is_active      boolean NOT NULL DEFAULT true,
            created_at     timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id),
            UNIQUE (tenant_id, sku)
        )
        """
    )

    op.execute(
        """
        CREATE TABLE ops.price (
            tenant_id    uuid NOT NULL,
            id           uuid NOT NULL DEFAULT gen_random_uuid(),
            product_id   uuid NOT NULL,
            -- NULL location_id = org-wide price; a row with a location_id
            -- overrides it for that branch.
            location_id  uuid,
            price_egp    numeric NOT NULL CHECK (price_egp >= 0),
            valid_from   date NOT NULL DEFAULT current_date,
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, product_id)
                REFERENCES ops.product (tenant_id, id) ON DELETE CASCADE,
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE
        )
        """
    )
    op.execute("CREATE INDEX ix_price_product ON ops.price (tenant_id, product_id)")

    # ── CRM ──────────────────────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ops.customer (
            tenant_id          uuid NOT NULL REFERENCES ops.tenant(id) ON DELETE CASCADE,
            id                 uuid NOT NULL DEFAULT gen_random_uuid(),
            name               text NOT NULL,
            phone_e164         text NOT NULL,
            whatsapp_opt_in    boolean NOT NULL DEFAULT false,
            opt_in_at          timestamptz,
            -- Coarse by design (district centroid), doc 03 §3.4.
            home_area_geog     geography(Point, 4326),
            tags               text[] NOT NULL DEFAULT '{}',
            first_location_id  uuid,
            created_at         timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id),
            UNIQUE (tenant_id, phone_e164),
            FOREIGN KEY (tenant_id, first_location_id)
                REFERENCES ops.location (tenant_id, id)
        )
        """
    )

    op.execute(
        """
        CREATE TABLE ops.customer_location_affinity (
            tenant_id    uuid NOT NULL,
            customer_id  uuid NOT NULL,
            location_id  uuid NOT NULL,
            visits       integer NOT NULL DEFAULT 0,
            revenue_egp  numeric NOT NULL DEFAULT 0,
            last_seen_at timestamptz,
            PRIMARY KEY (tenant_id, customer_id, location_id),
            FOREIGN KEY (tenant_id, customer_id)
                REFERENCES ops.customer (tenant_id, id) ON DELETE CASCADE,
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE
        )
        """
    )

    # ── Money facts (partitioned; PKs include the partition key) ────────────
    op.execute(
        """
        CREATE TABLE ops.invoice (
            tenant_id    uuid NOT NULL,
            location_id  uuid NOT NULL,
            id           uuid NOT NULL DEFAULT gen_random_uuid(),
            customer_id  uuid,
            issued_at    timestamptz NOT NULL,
            status       text NOT NULL DEFAULT 'issued' CHECK (status IN
                             ('draft','issued','paid','void','credit_note')),
            channel      text NOT NULL DEFAULT 'pos' CHECK (channel IN
                             ('pos','invoice','delivery','marketplace')),
            -- {net, vat, gross, discount} in EGP
            totals       jsonb NOT NULL,
            eta_state    text CHECK (eta_state IN
                             ('pending','submitted','accepted','rejected','exempt')),
            eta_uuid     text,
            created_by   uuid,
            -- Review defect #1 fix: partition key in the PK.
            PRIMARY KEY (tenant_id, id, issued_at),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id),
            FOREIGN KEY (tenant_id, customer_id)
                REFERENCES ops.customer (tenant_id, id)
        ) PARTITION BY RANGE (issued_at)
        """
    )
    _make_partitions("invoice", "ops.invoice")
    op.execute(
        "CREATE INDEX ix_invoice_branch_day "
        "ON ops.invoice (tenant_id, location_id, issued_at)"
    )
    op.execute("CREATE INDEX ix_invoice_eta ON ops.invoice (tenant_id, eta_state)")

    op.execute(
        """
        CREATE TABLE ops.invoice_line (
            tenant_id          uuid NOT NULL,
            id                 uuid NOT NULL DEFAULT gen_random_uuid(),
            invoice_id         uuid NOT NULL,
            -- Carried so the FK can target the full invoice PK
            -- (tenant_id, id, issued_at) on the partitioned parent.
            invoice_issued_at  timestamptz NOT NULL,
            product_id         uuid NOT NULL,
            qty                numeric NOT NULL CHECK (qty <> 0),
            unit_price_egp     numeric NOT NULL,
            line_total_egp     numeric NOT NULL,
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, invoice_id, invoice_issued_at)
                REFERENCES ops.invoice (tenant_id, id, issued_at)
                ON DELETE CASCADE,
            FOREIGN KEY (tenant_id, product_id)
                REFERENCES ops.product (tenant_id, id)
        )
        """
    )
    op.execute(
        "CREATE INDEX ix_invoice_line_invoice "
        "ON ops.invoice_line (tenant_id, invoice_id)"
    )

    op.execute(
        """
        CREATE TABLE ops.payment (
            tenant_id          uuid NOT NULL,
            location_id        uuid NOT NULL,
            id                 uuid NOT NULL DEFAULT gen_random_uuid(),
            invoice_id         uuid,
            invoice_issued_at  timestamptz,
            method             text NOT NULL CHECK (method IN
                                   ('cash','card','wallet','transfer')),
            amount_egp         numeric NOT NULL,
            paid_at            timestamptz NOT NULL,
            PRIMARY KEY (tenant_id, id, paid_at),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id),
            -- MATCH SIMPLE: unenforced when invoice_id IS NULL (cash drawer
            -- events without an invoice), enforced when fully present.
            FOREIGN KEY (tenant_id, invoice_id, invoice_issued_at)
                REFERENCES ops.invoice (tenant_id, id, issued_at)
        ) PARTITION BY RANGE (paid_at)
        """
    )
    _make_partitions("payment", "ops.payment")
    op.execute(
        "CREATE INDEX ix_payment_branch_day "
        "ON ops.payment (tenant_id, location_id, paid_at)"
    )

    op.execute(
        """
        CREATE TABLE ops.expense (
            tenant_id    uuid NOT NULL,
            location_id  uuid NOT NULL,
            id           uuid NOT NULL DEFAULT gen_random_uuid(),
            category     text NOT NULL,
            subcategory  text,
            amount_egp   numeric NOT NULL CHECK (amount_egp >= 0),
            incurred_at  timestamptz NOT NULL,
            is_recurring boolean NOT NULL DEFAULT false,
            receipt_url  text,
            note         text,
            created_by   uuid,
            PRIMARY KEY (tenant_id, id, incurred_at),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id)
        ) PARTITION BY RANGE (incurred_at)
        """
    )
    _make_partitions("expense", "ops.expense")
    op.execute(
        "CREATE INDEX ix_expense_branch_day "
        "ON ops.expense (tenant_id, location_id, incurred_at)"
    )

    # ── Inventory ────────────────────────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ops.stock_level (
            tenant_id      uuid NOT NULL,
            location_id    uuid NOT NULL,
            product_id     uuid NOT NULL,
            on_hand        numeric NOT NULL DEFAULT 0,
            reorder_point  numeric,
            updated_at     timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, location_id, product_id),
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id) ON DELETE CASCADE,
            FOREIGN KEY (tenant_id, product_id)
                REFERENCES ops.product (tenant_id, id) ON DELETE CASCADE
        )
        """
    )

    op.execute(
        """
        -- Two location columns make inter-branch transfers first-class
        -- (doc 03 §3.2). receive: from NULL → to. transfer: from → to.
        -- adjust/count: one side NULL. sale: from → NULL.
        CREATE TABLE ops.stock_move (
            tenant_id         uuid NOT NULL,
            id                uuid NOT NULL DEFAULT gen_random_uuid(),
            product_id        uuid NOT NULL,
            from_location_id  uuid,
            to_location_id    uuid,
            qty               numeric NOT NULL CHECK (qty > 0),
            move_type         text NOT NULL CHECK (move_type IN
                                  ('receive','transfer','adjust','count','sale')),
            moved_at          timestamptz NOT NULL DEFAULT now(),
            note              text,
            PRIMARY KEY (tenant_id, id),
            CHECK (from_location_id IS NOT NULL OR to_location_id IS NOT NULL),
            FOREIGN KEY (tenant_id, product_id)
                REFERENCES ops.product (tenant_id, id),
            FOREIGN KEY (tenant_id, from_location_id)
                REFERENCES ops.location (tenant_id, id),
            FOREIGN KEY (tenant_id, to_location_id)
                REFERENCES ops.location (tenant_id, id)
        )
        """
    )
    op.execute(
        "CREATE INDEX ix_stock_move_product "
        "ON ops.stock_move (tenant_id, product_id, moved_at)"
    )

    # ── Follow-ups & WhatsApp stubs ──────────────────────────────────────────
    op.execute(
        """
        CREATE TABLE ops.followup_task (
            tenant_id    uuid NOT NULL,
            id           uuid NOT NULL DEFAULT gen_random_uuid(),
            customer_id  uuid NOT NULL,
            location_id  uuid NOT NULL,
            reason       text NOT NULL,
            status       text NOT NULL DEFAULT 'open' CHECK (status IN
                             ('open','done','snoozed','dismissed')),
            due_at       timestamptz,
            completed_at timestamptz,
            assigned_to  uuid,
            created_at   timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, customer_id)
                REFERENCES ops.customer (tenant_id, id) ON DELETE CASCADE,
            FOREIGN KEY (tenant_id, location_id)
                REFERENCES ops.location (tenant_id, id)
        )
        """
    )
    op.execute(
        "CREATE INDEX ix_followup_queue "
        "ON ops.followup_task (tenant_id, location_id, status, due_at)"
    )

    # Stubs: structure only, no integration logic this session (scope rule).
    op.execute(
        """
        CREATE TABLE ops.whatsapp_thread (
            tenant_id        uuid NOT NULL,
            id               uuid NOT NULL DEFAULT gen_random_uuid(),
            customer_id      uuid NOT NULL,
            phone_e164       text NOT NULL,
            last_message_at  timestamptz,
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, customer_id)
                REFERENCES ops.customer (tenant_id, id) ON DELETE CASCADE
        )
        """
    )
    op.execute(
        """
        CREATE TABLE ops.whatsapp_message (
            tenant_id     uuid NOT NULL,
            id            uuid NOT NULL DEFAULT gen_random_uuid(),
            thread_id     uuid NOT NULL,
            direction     text NOT NULL CHECK (direction IN ('inbound','outbound')),
            body          text,
            template_key  text,
            sent_at       timestamptz NOT NULL DEFAULT now(),
            PRIMARY KEY (tenant_id, id),
            FOREIGN KEY (tenant_id, thread_id)
                REFERENCES ops.whatsapp_thread (tenant_id, id) ON DELETE CASCADE
        )
        """
    )

    # ── RLS (deliverable 2) ──────────────────────────────────────────────────
    # NULLIF guards the empty-string case; current_setting(..., true) returns
    # NULL when the GUC is unset → policy predicate is NULL → no rows. An
    # unscoped session sees and writes NOTHING.
    op.execute(
        """
        CREATE FUNCTION ops.current_tenant_id() RETURNS uuid
        LANGUAGE sql STABLE AS $$
            SELECT NULLIF(current_setting('app.tenant_id', true), '')::uuid
        $$;
        """
    )
    for table, col in RLS_TABLES:
        op.execute(f"ALTER TABLE {table} ENABLE ROW LEVEL SECURITY")
        op.execute(f"ALTER TABLE {table} FORCE ROW LEVEL SECURITY")
        op.execute(
            f"CREATE POLICY tenant_isolation ON {table} "
            f"USING ({col} = ops.current_tenant_id()) "
            f"WITH CHECK ({col} = ops.current_tenant_id())"
        )

    # ── Grants — parent tables only (partition access must fail) ────────────
    for table, _ in RLS_TABLES:
        op.execute(f"GRANT SELECT, INSERT, UPDATE, DELETE ON {table} TO mahal_app")
    op.execute("GRANT SELECT, INSERT, UPDATE ON ops.app_user TO mahal_app")


def downgrade() -> None:
    tables = [
        "whatsapp_message", "whatsapp_thread", "followup_task", "stock_move",
        "stock_level", "expense", "payment", "invoice_line", "invoice",
        "customer_location_affinity", "customer", "price", "product",
        "location_history", "location", "membership", "app_user", "tenant",
    ]
    op.execute("DROP TRIGGER IF EXISTS trg_location_history ON ops.location")
    op.execute("DROP FUNCTION IF EXISTS ops.location_track_history()")
    for t in tables:
        op.execute(f"DROP TABLE IF EXISTS ops.{t} CASCADE")
    op.execute("DROP FUNCTION IF EXISTS ops.current_tenant_id()")
