	-- ============================================================
	-- Script Name: etl_daily_aggregation.sql
	-- Purpose: Aggregate real-time sensor data into daily summaries
	-- Database: edge_ai_location_intelligence
	-- Schema: public
	-- ============================================================
	
	-- 🧹 Optional: Clean previous aggregation for today (to re-run ETL safely)
	DELETE FROM Daily_FPGA_Metrics
	WHERE date = current_date;
	
	-- 🧩 Step 1: Aggregate today's real-time metrics into daily summaries
	INSERT INTO Daily_FPGA_Metrics (
	    location_id,
	    date,
	    daily_footfall,
	    daily_avg_dwell,
	    daily_vehicle_count,
	    hours_of_operation,
	    missing_intervals,
	    created_at
	)
	SELECT
	    rm.location_id,
	    current_date AS date,
	    COUNT(rm.people_count) AS daily_footfall,
	    ROUND(AVG(rm.dwell_time_avg), 2) AS daily_avg_dwell,
	    SUM(rm.vehicle_count) AS daily_vehicle_count,
	    EXTRACT(HOUR FROM MAX(rm.timestamp) - MIN(rm.timestamp))::INTEGER AS hours_of_operation,
	    SUM(CASE WHEN rm.data_quality_flag <> 0 THEN 1 ELSE 0 END) AS missing_intervals,
	    now() AS created_at
	FROM RealTime_Metrics rm
	WHERE rm.timestamp::date = current_date
	GROUP BY rm.location_id
	ON CONFLICT (location_id, date)
	DO UPDATE SET
	    daily_footfall = EXCLUDED.daily_footfall,
	    daily_avg_dwell = EXCLUDED.daily_avg_dwell,
	    daily_vehicle_count = EXCLUDED.daily_vehicle_count,
	    hours_of_operation = EXCLUDED.hours_of_operation,
	    missing_intervals = EXCLUDED.missing_intervals,
	    created_at = now();
	
	-- 🧮 Step 2: Log ETL run status
	INSERT INTO Events (name, location_lat, location_lon, start_date, end_date, expected_attendance)
	VALUES ('ETL Daily Aggregation Run', NULL, NULL, current_date, current_date, NULL);
	
	-- ✅ Confirmation message
	DO $$
	BEGIN
	    RAISE NOTICE '✅ Daily FPGA metrics aggregated successfully on %', current_date;
	END $$;
