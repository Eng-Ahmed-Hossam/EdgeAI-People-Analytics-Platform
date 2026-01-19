-- ============================================================
-- FEATURE ENGINEERING: behavior_trends (Rolling & Volatility)
-- Engineering Objectives:
--   • Encode recent demand level via rolling averages
--   • Capture short-term directionality using growth rates
--   • Quantify stability vs. volatility in traffic patterns
--   • Ensure features are label-safe, rebuildable, and auditable
--
-- Source Table:
--   Daily_FPGA_Metrics
--
-- Windowing Strategy:
--   • Rolling window: 7 days (explicit and fixed)
--   • Partitioning: per location_id
--   • Ordering: chronological by date
--
-- Feature Definitions:
--   1) footfall_7d_avg
--      → 7-day rolling average of daily_footfall
--      → Represents recent demand level (smoothed signal)
--
--   2) footfall_growth_rate
--      → Day-over-day relative change:
--        (footfall_t − footfall_{t−1}) / footfall_{t−1}
--      → Captures short-term momentum and demand direction
--      → NULL-safe via NULLIF on previous day value
--
--   3) dwell_trend_slope
--      → Linear regression slope of daily_avg_dwell over
--        the last 7 days
--      → Computed using REGR_SLOPE with time (epoch) as X
--      → Positive slope indicates increasing engagement
--
--   4) traffic_volatility_index
--      → Coefficient of variation over last 7 days:
--        σ / μ = STDDEV_SAMP(daily_footfall) /
--                AVG(daily_footfall)
--      → Normalized volatility measure, scale-invariant
--
--   5) trend_window_days
--      → Constant metadata field (value = 7)
--      → Enables downstream auditability and reproducibility
--
-- Implementation Notes:
--   • LAG is used to access prior-day footfall safely
--   • Rolling aggregates use ROWS BETWEEN 6 PRECEDING
--     AND CURRENT ROW (exact 7-day window)
--   • Regression is recomputed per (location_id, date)
--     using a bounded date range for correctness
--   • LEFT JOIN preserves rows where slope cannot be
--     computed due to insufficient history
--
-- Intended Use:
--   • Behavioral trend modeling
--   • Early demand acceleration / deceleration detection
--   • Stability vs. shock analysis
--   • Feature inputs for forecasting and profit models
--
-- Assumptions:
--   • Dates are contiguous or sparse-safe by design
--   • Rolling windows tolerate partial history at start
--   • No leakage from future data (strictly backward-looking)
-- ============================================================

TRUNCATE TABLE behavior_trends RESTART IDENTITY CASCADE;

INSERT INTO behavior_trends
(location_id, date,
 footfall_7d_avg, footfall_growth_rate,
 dwell_trend_slope, traffic_volatility_index,
 trend_window_days)
WITH ordered AS (
  SELECT
    location_id,
    date,
    daily_footfall,
    daily_avg_dwell,
    LAG(daily_footfall) OVER (
      PARTITION BY location_id ORDER BY date
    ) AS prev_footfall
  FROM Daily_FPGA_Metrics
),
rolling AS (
  SELECT
    location_id,
    date,
    daily_footfall,
    daily_avg_dwell,
    prev_footfall,

    AVG(daily_footfall) OVER (
      PARTITION BY location_id
      ORDER BY date
      ROWS BETWEEN 6 PRECEDING AND CURRENT ROW
    ) AS footfall_7d_avg,

    STDDEV_SAMP(daily_footfall) OVER (
      PARTITION BY location_id
      ORDER BY date
      ROWS BETWEEN 6 PRECEDING AND CURRENT ROW
    ) AS footfall_std,

    AVG(daily_avg_dwell) OVER (
      PARTITION BY location_id
      ORDER BY date
      ROWS BETWEEN 6 PRECEDING AND CURRENT ROW
    ) AS dwell_avg_7d
  FROM ordered
),
dwell_slope AS (
  -- Linear regression slope of dwell over 7 days
  SELECT
    r.location_id,
    r.date,
    REGR_SLOPE(d.daily_avg_dwell, EXTRACT(EPOCH FROM d.date))
      AS dwell_trend_slope
  FROM rolling r
  JOIN Daily_FPGA_Metrics d
    ON d.location_id = r.location_id
   AND d.date BETWEEN r.date - INTERVAL '6 days' AND r.date
  GROUP BY r.location_id, r.date
)
SELECT
  r.location_id,
  r.date,

  ROUND(r.footfall_7d_avg, 2) AS footfall_7d_avg,

  ROUND(
    (r.daily_footfall - r.prev_footfall)
    / NULLIF(r.prev_footfall, 0),
    4
  ) AS footfall_growth_rate,

  ROUND(ds.dwell_trend_slope::numeric, 6) AS dwell_trend_slope,

  ROUND(
    r.footfall_std / NULLIF(r.footfall_7d_avg, 0),
    4
  ) AS traffic_volatility_index,

  7 AS trend_window_days
FROM rolling r
LEFT JOIN dwell_slope ds
  ON ds.location_id = r.location_id
 AND ds.date = r.date;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Behavior Trends table inserted successfully ✅';
END $$;