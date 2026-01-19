-- ============================================================
-- AGGREGATION LOGIC: Hourly_FPGA_Metrics
-- Purpose:
--   Generate hourly-level aggregated metrics from high-frequency
--   RealTime_Metrics data, optimized for FPGA/edge analytics
--   and downstream reporting.
--
-- Derived Fields:
--   location_id           → Source location identifier
--   date                  → Calendar date (derived from timestamp)
--   hour                  → Hour of day (0–23)
--   hourly_footfall       → Number of sensor records in the hour
--   hourly_avg_dwell      → Mean dwell time for the hour (minutes)
--   peak_hour_flag        → TRUE if hourly activity exceeds the
--                           90th percentile of daily activity
--                           for the same location
--
-- Intended Use:
--   - FPGA/edge-level pre-aggregation to reduce data volume
--   - Peak-hour detection for staffing and pricing strategies
--   - Input features for demand forecasting models
--   - Hourly dashboards and alerting systems
--
-- Notes:
--   • Assumes RealTime_Metrics.timestamp is TIMESTAMPTZ.
--   • Peak detection is relative (daily), not absolute.
--   • COUNT(*) can be replaced with SUM(people_count)
--     if raw footfall counts are required.
-- ============================================================

TRUNCATE TABLE Hourly_FPGA_Metrics RESTART IDENTITY CASCADE;

INSERT INTO Hourly_FPGA_Metrics
(location_id, date, hour, hourly_footfall, hourly_avg_dwell, peak_hour_flag)
WITH hourly_base AS (
  SELECT
    location_id,
    DATE(timestamp) AS date,
    EXTRACT(HOUR FROM timestamp)::INT AS hour,
    SUM(COALESCE(people_count,0)) AS hourly_footfall,
    AVG(dwell_time_avg) AS hourly_avg_dwell
  FROM RealTime_Metrics
  GROUP BY location_id, DATE(timestamp), EXTRACT(HOUR FROM timestamp)
),
ranked AS (
  SELECT
    *,
    NTILE(10) OVER (
      PARTITION BY location_id, date
      ORDER BY hourly_footfall DESC
    ) AS footfall_decile
  FROM hourly_base
)
SELECT
  location_id,
  date,
  hour,
  hourly_footfall,
  ROUND(hourly_avg_dwell, 2),
  CASE
    WHEN footfall_decile = 1 THEN TRUE
    ELSE FALSE
  END AS peak_hour_flag
FROM ranked;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Hourly_FPGA_Metrics inserted successfully ✅';
END $$;