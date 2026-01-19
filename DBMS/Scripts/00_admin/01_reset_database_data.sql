-- ============================================================
-- Script Name: reset_database_data.sql
-- Purpose: Safely delete all data while keeping tables, schema, and constraints intact
-- Database: edge_ai_location_intelligence
-- Schema: public
-- ============================================================

-- 🧠 Temporarily disable referential integrity checks
SET session_replication_role = 'replica';

-- ==========================================
-- 1️⃣ Delete from child tables first (to avoid FK violations)
-- ==========================================
TRUNCATE TABLE
    Prediction_Output,
    feature_store,
    behavior_trends,
    operational_efficiency,
    Hourly_FPGA_Metrics,
    Daily_FPGA_Metrics,
    data_quality_log,
    Daily_Revenue,
    Promotions,
    Events,
    Weather,
    RealTime_Metrics,
    Business_Profile,
    Location_Metadata
RESTART IDENTITY CASCADE;

-- ==========================================
-- 2️⃣ Re-enable foreign key constraints
-- ==========================================
SET session_replication_role = 'origin';

-- ==========================================
-- 3️⃣ Log the reset operation
-- ==========================================
INSERT INTO Events (name, start_date, end_date, expected_attendance)
VALUES ('Database Reset Operation', current_date, current_date, NULL);

-- ==========================================
-- Confirmation message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE '✅ All data cleared successfully — schema structure preserved.';
END $$;


SELECT * FROM Location_Metadata;  -- should be empty
SELECT COUNT(*) FROM Business_Profile;  -- should be 0