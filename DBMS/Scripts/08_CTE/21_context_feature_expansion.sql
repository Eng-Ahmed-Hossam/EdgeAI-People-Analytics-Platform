-- ============================================================
-- Daily Weather Aggregation
-- Purpose:
--   Derive daily, model-ready contextual features by expanding
--   raw Weather, Promotions, and Events data to a daily grain.
--
-- Description:
--   • daily_weather  → Aggregates hourly weather into daily
--     averages and extreme-heat indicators per location.
--   • active_promos  → Expands promotion date ranges into
--     daily rows and resolves overlapping promos via max lift.
--   • event_days     → Expands multi-day events into a daily
--     calendar for binary event-impact joins.
--
-- Intended Use:
--   - Upstream dependency for feature_store assembly
--   - Label-safe contextual enrichment for ML pipelines
--   - Deterministic, rebuildable daily features
-- ============================================================

CREATE OR REPLACE VIEW daily_weather AS (
  SELECT
    location_id,
    DATE(timestamp) AS date,
    AVG(temp_c) AS avg_temp_c,
    MAX(CASE WHEN temp_c >= 40 THEN 1 ELSE 0 END) AS is_extreme_heat
  FROM Weather
  GROUP BY location_id, DATE(timestamp)
);

CREATE OR REPLACE VIEW active_promos AS (
  SELECT
    business_id,
    date,
    MAX(expected_lift_pct) AS promo_lift_pct
  FROM (
    SELECT
      p.business_id,
      d::date AS date,
      p.expected_lift_pct
    FROM Promotions p
    JOIN generate_series(p.start_date, p.end_date, '1 day') d
      ON TRUE
  ) x
  GROUP BY business_id, date
);

CREATE OR REPLACE VIEW event_days AS (
  SELECT DISTINCT
    d::date AS date
  FROM Events e
  JOIN generate_series(e.start_date, e.end_date, '1 day') d
    ON TRUE
);

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Daily Weather Aggregation inserted successfully ✅';
END $$;