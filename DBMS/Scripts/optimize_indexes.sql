-- ============================================================
-- Script Name: optimize_indexes.sql
-- Purpose: Improve performance for joins, lookups, and analytics
-- Database: edge_ai_location_intelligence
-- Schema: public
-- ============================================================

-- 🧭 Location Metadata: spatial and lookup indexes
CREATE INDEX IF NOT EXISTS idx_location_coords 
ON Location_Metadata(latitude, longitude);

CREATE INDEX IF NOT EXISTS idx_location_type 
ON Location_Metadata(location_type);

CREATE INDEX IF NOT EXISTS idx_location_income 
ON Location_Metadata(avg_income);

-- 🏪 Business Profile: link to locations + business types
CREATE INDEX IF NOT EXISTS idx_business_location 
ON Business_Profile(location_id);

CREATE INDEX IF NOT EXISTS idx_business_type 
ON Business_Profile(business_type);

CREATE INDEX IF NOT EXISTS idx_business_rent 
ON Business_Profile(rent_monthly);

-- 📊 Real-Time Metrics: time-based performance
CREATE INDEX IF NOT EXISTS idx_metrics_location_ts 
ON RealTime_Metrics(location_id, timestamp);

CREATE INDEX IF NOT EXISTS idx_metrics_quality 
ON RealTime_Metrics(data_quality_flag);

-- 💵 Daily Revenue: lookup by business and date
CREATE INDEX IF NOT EXISTS idx_revenue_business_date 
ON Daily_Revenue(business_id, date);

-- 📈 Prediction Outputs: performance for reporting
CREATE INDEX IF NOT EXISTS idx_prediction_business_date 
ON Prediction_Output(business_id, prediction_date);

CREATE INDEX IF NOT EXISTS idx_prediction_location 
ON Prediction_Output(location_id);

-- 🎯 Promotions: connect to business
CREATE INDEX IF NOT EXISTS idx_promotions_business 
ON Promotions(business_id);

CREATE INDEX IF NOT EXISTS idx_promotions_type 
ON Promotions(promo_type);

-- 🌦️ Weather: join by time or station
CREATE INDEX IF NOT EXISTS idx_weather_timestamp 
ON Weather(timestamp);

-- 🎉 Events: time and location relevance
CREATE INDEX IF NOT EXISTS idx_event_time 
ON Events(start_date, end_date);

CREATE INDEX IF NOT EXISTS idx_event_location 
ON Events(location_lat, location_lon);

-- Optional optimization ideas
CREATE INDEX IF NOT EXISTS idx_metrics_congestion 
ON RealTime_Metrics(congestion_level);

CREATE INDEX IF NOT EXISTS idx_weather_location 
ON Weather(location_id);

-- ✅ Confirmation message
DO $$
BEGIN
    RAISE NOTICE 'All performance indexes created successfully ✅';
END$$;
