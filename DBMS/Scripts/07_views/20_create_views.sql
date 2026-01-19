-- ============================================================
-- Script Name: create_views.sql
-- Purpose: Define analytical views and rollups for the Edge AI Location Intelligence database
-- Database: edge_ai_location_intelligence
-- Schema: public
-- ============================================================

-- ==========================================
-- VIEW: hourly_metrics
-- Summarizes real-time data into hourly intervals per location
-- ==========================================
CREATE OR REPLACE VIEW hourly_metrics AS
SELECT
    location_id,
    date + hour * INTERVAL '1 hour' AS hour,
    hourly_footfall AS total_people,
    hourly_avg_dwell AS avg_dwell,
    NULL::INTEGER AS total_vehicles,
    NULL::INTEGER AS bad_intervals
    -- SUM(vehicle_count) AS total_vehicles,
    -- SUM(CASE WHEN data_quality_flag <> 0 THEN 1 ELSE 0 END) AS bad_intervals
FROM Hourly_FPGA_Metrics
GROUP BY location_id, date_trunc('hour', timestamp)
ORDER BY location_id, hour;

-- ==========================================
-- VIEW: daily_summary
-- Joins daily footfall metrics with weather and revenue
-- ==========================================
CREATE OR REPLACE VIEW daily_summary AS
SELECT
    df.location_id,
    lm.location_type,
    lm.avg_income,
    df.date,
    df.daily_footfall,
    df.daily_avg_dwell,
    df.daily_vehicle_count,
    w.temp_c,
    w.is_rain,
    dr.total_revenue,
    dr.avg_transaction_value,
    (dr.total_revenue / NULLIF(df.daily_footfall, 0))::NUMERIC(12,2) AS revenue_per_visitor
FROM Daily_FPGA_Metrics df
JOIN Location_Metadata lm ON df.location_id = lm.location_id
LEFT JOIN Weather w ON df.location_id = w.location_id AND date_trunc('day', w.timestamp) = df.date
LEFT JOIN Business_Profile bp ON lm.location_id = bp.location_id
LEFT JOIN Daily_Revenue dr ON bp.business_id = dr.business_id AND dr.date = df.date
ORDER BY df.date DESC;

-- ==========================================
-- VIEW: weekly_summary
-- Aggregates daily summaries into weekly performance per location
-- ==========================================
CREATE OR REPLACE VIEW weekly_summary AS
SELECT
    location_id,
    date_trunc('week', date)::date AS week_start,
    SUM(daily_footfall) AS weekly_footfall,
    ROUND(AVG(daily_avg_dwell), 2) AS avg_weekly_dwell,
    SUM(daily_vehicle_count) AS weekly_vehicles,
    ROUND(AVG(total_revenue), 2) AS avg_weekly_revenue
FROM daily_summary
GROUP BY location_id, date_trunc('week', date)
ORDER BY week_start DESC;

-- ==========================================
-- Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE '✅ Analytical views (hourly, daily, weekly) created successfully.';
END $$;
