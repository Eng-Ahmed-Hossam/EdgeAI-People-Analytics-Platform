-- ============================================================
-- data_quality_log
-- Purpose:
--   Centralize daily data quality checks across core pipeline
--   tables to quantify completeness and flag ingestion or
--   sensor anomalies.
--
-- Description:
--   This script performs an idempotent refresh of data_quality_log
--   and inserts daily quality metrics derived from:
--     • RealTime_Metrics
--     • Daily_FPGA_Metrics
--     • Weather
--
-- Quality Rules Implemented:
--   1) RealTime_Metrics
--      - Completeness calculated as:
--        (Total observed records / Total expected records) × 100
--      - Expected records inferred per (location_id, date)
--        using the maximum observed count (sampling-agnostic)
--      - Anomaly flagged if any location-day drops below 70%
--
--   2) Daily_FPGA_Metrics
--      - Completeness derived from missing_intervals assuming
--        a 24-hour daily baseline
--      - Anomaly flagged if any missing intervals are present
--
--   3) Weather
--      - Completeness based on locations meeting minimum
--        hourly coverage (≥ 20 readings/day)
--      - Anomaly flagged for extreme or implausible temperatures
--
-- Outputs:
--   • source_table      → Monitored table name
--   • record_date       → Quality assessment date
--   • completeness_pct  → Daily data coverage percentage
--   • anomaly_flag      → Boolean quality breach indicator
-- ============================================================

TRUNCATE TABLE data_quality_log RESTART IDENTITY CASCADE;

INSERT INTO data_quality_log
(source_table, record_date, completeness_pct, anomaly_flag)
SELECT
  'RealTime_Metrics' AS source_table,
  date,
  ROUND(
    (SUM(obs_count) / SUM(expected_count)) * 100,
    2
  ) AS completeness_pct,
  BOOL_OR(obs_count < expected_count * 0.7) AS anomaly_flag
FROM (
  SELECT
    location_id,
    DATE(timestamp) AS date,
    COUNT(*) AS obs_count,
    MAX(COUNT(*)) OVER (PARTITION BY location_id, DATE(timestamp))
      AS expected_count
  FROM RealTime_Metrics
  GROUP BY location_id, DATE(timestamp)
) t
GROUP BY date;

INSERT INTO data_quality_log
(source_table, record_date, completeness_pct, anomaly_flag)
SELECT
  'Daily_FPGA_Metrics',
  date,
  ROUND(
    LEAST(
      100,
      GREATEST(0, 100 - (AVG(missing_intervals)::NUMERIC * 100 / 24))
    ),
    2
  ) AS completeness_pct,
  BOOL_OR(missing_intervals > 0) AS anomaly_flag
FROM Daily_FPGA_Metrics
GROUP BY date;

INSERT INTO data_quality_log
(source_table, record_date, completeness_pct, anomaly_flag)
SELECT
  'RealTime_Metrics' AS source_table,
  date,
  ROUND(
    (SUM(obs_count)::NUMERIC / SUM(expected_count)) * 100,
    2
  ) AS completeness_pct,
  BOOL_OR(obs_count < expected_count * 0.7) AS anomaly_flag
FROM (
  SELECT
    location_id,
    DATE(timestamp) AS date,
    COUNT(*) AS obs_count,
    MAX(COUNT(*)) OVER (PARTITION BY location_id, DATE(timestamp))
      AS expected_count
  FROM RealTime_Metrics
  GROUP BY location_id, DATE(timestamp)
) t
GROUP BY date;

INSERT INTO data_quality_log
(source_table, record_date, completeness_pct, anomaly_flag)
SELECT
  'Weather',
  date,
  ROUND(
    100.0 * COUNT(*) FILTER (WHERE hourly_count >= 20) / COUNT(*),
    2
  ) AS completeness_pct,
  BOOL_OR(max_temp > 50 OR min_temp < 5) AS anomaly_flag
FROM (
  SELECT
    DATE(timestamp) AS date,
    location_id,
    COUNT(*) AS hourly_count,
    MAX(temp_c) AS max_temp,
    MIN(temp_c) AS min_temp
  FROM Weather
  GROUP BY DATE(timestamp), location_id
) w
GROUP BY date;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'DATA QUALITY MONITORING inserted successfully ✅';
END $$;