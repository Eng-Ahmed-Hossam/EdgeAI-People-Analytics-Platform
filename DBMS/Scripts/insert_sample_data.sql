-- ==========================================
-- SAMPLE DATA: Edge AI Location Intelligence Database
-- ==========================================
-- Purpose: Insert representative data across all entities
-- Compatible with the finalized schema
-- ==========================================

-- 🧹 Clear previous data (safe reset)
TRUNCATE TABLE 
    RealTime_Metrics,
    Daily_FPGA_Metrics,
    Daily_Revenue,
    Prediction_Output,
    Weather,
    Promotions,
    Events,
    Business_Profile,
    Location_Metadata
RESTART IDENTITY CASCADE;

-- ==========================================
-- 🌍 LOCATION_METADATA
-- ==========================================
INSERT INTO Location_Metadata (latitude, longitude, location_name, location_type, population, avg_income, competitor_count, catchment_radius_m)
VALUES
(30.0444, 31.2357, 'Downtown Cairo', 'urban', 500000, 15000, 25, 3000),
(29.9753, 31.1376, 'Giza Plateau', 'tourist', 120000, 13000, 10, 5000),
(31.2001, 29.9187, 'Alexandria Corniche', 'coastal', 350000, 14000, 20, 4000);

-- ==========================================
-- 🏪 BUSINESS_PROFILE
-- ==========================================
INSERT INTO Business_Profile (location_id, name, business_type, product_type, owner_experience_years, work_hours_per_day, work_days_per_week, store_area_m2, rent_monthly, staff_cost_monthly, utilities_cost_monthly, opening_date)
VALUES
(1, 'Cairo Coffee Co.', 'cafe', 'coffee', 5, 12, 6, 80, 15000, 10000, 3000, '2020-02-01'),
(1, 'Tahrir Market', 'retail', 'groceries', 10, 14, 7, 200, 25000, 15000, 5000, '2018-09-10'),
(2, 'Pyramid Gifts', 'souvenir', 'handicrafts', 7, 10, 6, 60, 10000, 6000, 2000, '2021-05-12'),
(3, 'Sea Breeze Restaurant', 'restaurant', 'seafood', 8, 12, 6, 150, 30000, 18000, 7000, '2019-11-03');

-- ==========================================
-- 📊 REALTIME_METRICS
-- ==========================================
INSERT INTO RealTime_Metrics (location_id, timestamp, people_count, dwell_time_avg, vehicle_count, congestion_level, data_quality_flag)
VALUES
(1, now() - interval '3 hours', 60, 120.5, 20, 'medium', 0),
(1, now() - interval '2 hours', 90, 115.3, 25, 'high', 0),
(1, now() - interval '1 hour', 70, 130.2, 22, 'medium', 0),
(2, now() - interval '3 hours', 50, 90.4, 15, 'low', 0),
(2, now() - interval '2 hours', 65, 100.1, 18, 'medium', 0),
(3, now() - interval '2 hours', 120, 140.8, 30, 'high', 0);

-- ==========================================
-- 📆 DAILY_FPGA_METRICS
-- ==========================================
INSERT INTO Daily_FPGA_Metrics (location_id, date, daily_footfall, daily_avg_dwell, daily_vehicle_count, hours_of_operation, missing_intervals)
VALUES
(1, current_date - 1, 2500, 110.4, 450, 12, 0),
(2, current_date - 1, 1300, 95.2, 320, 11, 2),
(3, current_date - 1, 2100, 125.6, 400, 13, 1);

-- ==========================================
-- 💵 DAILY_REVENUE
-- ==========================================
INSERT INTO Daily_Revenue (business_id, date, total_revenue, total_transactions, avg_transaction_value)
VALUES
(1, current_date - 1, 5500.00, 180, 30.5),
(2, current_date - 1, 12500.00, 350, 35.7),
(3, current_date - 1, 4200.00, 150, 28.0),
(4, current_date - 1, 9800.00, 200, 49.0);

-- ==========================================
-- 📈 PREDICTION_OUTPUT
-- ==========================================
INSERT INTO Prediction_Output (business_id, location_id, prediction_date, predicted_revenue, predicted_profit, confidence_score, model_version)
VALUES
(1, 1, current_date, 5800.00, 2300.00, 0.92, 'v1.0'),
(2, 1, current_date, 13000.00, 5000.00, 0.89, 'v1.0'),
(3, 2, current_date, 4500.00, 1800.00, 0.94, 'v1.0'),
(4, 3, current_date, 10200.00, 4100.00, 0.90, 'v1.0');

-- ==========================================
-- 🌦️ WEATHER
-- ==========================================
INSERT INTO Weather (location_id, timestamp, temp_c, precip_mm, is_rain)
VALUES
(1, now() - interval '3 hours', 25.0, 0.0, false),
(1, now() - interval '1 hour', 26.5, 0.0, false),
(2, now() - interval '2 hours', 27.2, 0.5, true),
(3, now() - interval '2 hours', 24.8, 0.0, false);

-- ==========================================
-- 🎯 PROMOTIONS
-- ==========================================
INSERT INTO Promotions (business_id, start_date, end_date, promo_type, expected_lift_pct)
VALUES
(1, current_date - 5, current_date - 2, 'discount', 10.0),
(2, current_date - 3, current_date + 1, 'bundle', 15.0),
(4, current_date - 1, current_date + 2, 'festival_offer', 20.0);

-- ==========================================
-- 🎉 EVENTS
-- ==========================================
INSERT INTO Events (name, location_lat, location_lon, start_date, end_date, expected_attendance)
VALUES
('Cairo Shopping Festival', 30.0450, 31.2335, current_date - 1, current_date + 3, 20000),
('Alexandria Beach Music Night', 31.2156, 29.9440, current_date - 2, current_date + 1, 15000);

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Sample data inserted successfully ✅';
END $$;
