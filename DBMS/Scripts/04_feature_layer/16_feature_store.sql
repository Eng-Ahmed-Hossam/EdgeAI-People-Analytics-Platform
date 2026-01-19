-- ============================================================
-- Feature Engineering: feature_store
-- Description:
--   This INSERT constructs a daily feature record for each
--   business by integrating multiple upstream feature tables
--   and metadata sources, then generates a deterministic
--   hash representing the full feature state at that date.
--
-- Feature Sources & Semantics:
--   • Daily_FPGA_Metrics (df)
--       - daily_footfall        → Core demand signal
--       - daily_avg_dwell       → Engagement depth
--
--   • behavior_trends (bt)
--       - footfall_growth_rate  → Short-term momentum
--       - traffic_volatility_index
--                              → Demand stability vs. noise
--
--   • daily_weather (dw)
--       - avg_temp_c            → Environmental context
--       - is_extreme_heat       → Heat stress binary signal
--
--   • Business_Profile (bp)
--       - store_area_m2         → Capacity proxy
--       - work_hours_per_day    → Operational exposure
--
--   • operational_efficiency (oe)
--       - staff_cost_ratio      → Labor efficiency
--       - rent_cost_ratio       → Fixed-cost pressure
--
--   • active_promos (ap)
--       - promo_lift_pct        → Expected promotion uplift
--         (COALESCE to 0 if no active promotion)
--
--   • event_days (ed)
--       - Event presence flag   → Binary exogenous shock
--
-- Intended Use:
--   - Centralized ML feature store
--   - Offline model training & backtesting
--   - Online inference consistency checks
--   - Feature drift and lineage tracking
-- ============================================================

TRUNCATE TABLE feature_store RESTART IDENTITY CASCADE;

INSERT INTO feature_store
(business_id, location_id, date, feature_vector_hash, generated_at)
SELECT
  bp.business_id,
  bp.location_id,
  dr.date,

  md5(
    ROW(
      df.daily_footfall,
      df.daily_avg_dwell,
      bt.footfall_growth_rate,
      bt.traffic_volatility_index,
      dw.avg_temp_c,
      dw.is_extreme_heat,
      bp.store_area_m2,
      bp.work_hours_per_day,
      oe.staff_cost_ratio,
      oe.rent_cost_ratio,
      COALESCE(ap.promo_lift_pct, 0),
      CASE WHEN ed.date IS NOT NULL THEN 1 ELSE 0 END
    )::text
  ) AS feature_vector_hash,

  NOW()
FROM Daily_Revenue dr
JOIN Business_Profile bp
  ON dr.business_id = bp.business_id
JOIN Daily_FPGA_Metrics df
  ON df.location_id = bp.location_id
 AND df.date = dr.date
LEFT JOIN behavior_trends bt
  ON bt.location_id = bp.location_id
 AND bt.date = dr.date
LEFT JOIN daily_weather dw
  ON dw.location_id = bp.location_id
 AND dw.date = dr.date
LEFT JOIN operational_efficiency oe
  ON oe.business_id = dr.business_id
 AND oe.date = dr.date
LEFT JOIN active_promos ap
  ON ap.business_id = dr.business_id
 AND ap.date = dr.date
LEFT JOIN event_days ed
  ON ed.date = dr.date;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Feature Store table inserted successfully ✅';
END $$;