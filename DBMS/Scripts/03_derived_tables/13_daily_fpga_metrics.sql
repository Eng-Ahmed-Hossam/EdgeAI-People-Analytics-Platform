-- ============================================================
-- AGGREGATION LOGIC: Daily_FPGA_Metrics
-- Purpose:
--   Aggregate high-frequency RealTime_Metrics into daily-level
--   summaries suitable for FPGA pre-processing, analytics,
--   dashboards, and machine learning pipelines.
--
-- Metrics Generated:
--   • daily_footfall        → Total pedestrian volume per day
--                             (SUM of people_count)
--   • daily_avg_dwell       → Average dwell time per day
--                             (AVG of dwell_time_avg)
--   • daily_vehicle_count   → Total vehicle traffic per day
--                             (SUM of vehicle_count)
--   • hours_of_operation    → Number of distinct active hours
--                             detected during the day
--   • missing_intervals     → Count of records flagged with
--                             degraded or invalid sensor data
--
-- Analytical Notes:
--   - DATE(timestamp) is used to normalize all events into
--     daily buckets (timezone-aware).
--   - hours_of_operation acts as a proxy for sensor uptime
--     and business activity span.
--   - missing_intervals provides a daily data reliability
--     signal for filtering, weighting, or anomaly handling
--     in downstream analytics and models.
--
-- Intended Use:
--   - Daily demand and traffic analysis
--   - Feature engineering for revenue/profit prediction
--   - Data quality monitoring at location/day granularity
--   - FPGA or edge-layer data volume reduction
-- ============================================================

TRUNCATE TABLE Daily_FPGA_Metrics RESTART IDENTITY CASCADE;

INSERT INTO Daily_FPGA_Metrics
(location_id, date, daily_footfall, daily_avg_dwell,
 daily_vehicle_count, hours_of_operation, missing_intervals)
WITH base AS (
  SELECT
    location_id,
    DATE(timestamp) AS date,
    EXTRACT(HOUR FROM timestamp)::INT AS hour,
    COUNT(*) AS obs_per_hour,
    SUM(COALESCE(people_count,0)) AS footfall,
    AVG(dwell_time_avg) AS avg_dwell,
    SUM(COALESCE(vehicle_count,0)) AS vehicles
  FROM RealTime_Metrics
  GROUP BY location_id, DATE(timestamp), EXTRACT(HOUR FROM timestamp)
),
hourly_agg AS (
  SELECT
    location_id,
    date,
    SUM(footfall) AS daily_footfall,
    AVG(avg_dwell) AS daily_avg_dwell,
    SUM(vehicles) AS daily_vehicle_count,
    COUNT(DISTINCT hour) AS hours_of_operation,
    MAX(obs_per_hour) AS max_obs_per_hour,
    SUM(obs_per_hour) AS total_obs
  FROM base
  GROUP BY location_id, date
)
SELECT
  location_id,
  date,
  daily_footfall,
  ROUND(daily_avg_dwell, 2),
  daily_vehicle_count,
  hours_of_operation,
  GREATEST((24 * max_obs_per_hour) - total_obs, 0) AS missing_intervals
FROM hourly_agg;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Daily_FPGA_Metrics inserted successfully ✅';
END $$;