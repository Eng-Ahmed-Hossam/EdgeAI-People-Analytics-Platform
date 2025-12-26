-- ==========================================
-- CREATE SCHEMA: Edge AI Location Intelligence Database
-- ==========================================
-- This script creates all tables, constraints, and relationships.
-- Compatible with PostgreSQL 13+ (tested on PostgreSQL 17)
-- ==========================================

-- 1️⃣ Ensure schema exists
CREATE SCHEMA IF NOT EXISTS public;

SET search_path TO public;

-- ==========================================
-- TABLE: Location_Metadata
-- Stores static metadata about each location
-- ==========================================
CREATE TABLE IF NOT EXISTS Location_Metadata (
    location_id SERIAL PRIMARY KEY,
    latitude DOUBLE PRECISION NOT NULL,
    longitude DOUBLE PRECISION NOT NULL,
    location_name TEXT,
    location_type TEXT NOT NULL,  -- e.g. mall, street, market, etc.
    population INTEGER,
    avg_income NUMERIC(12,2),
    competitor_count INTEGER,
    catchment_radius_m INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- ==========================================
-- TABLE: Business_Profile
-- Stores information about businesses linked to locations
-- ==========================================
CREATE TABLE IF NOT EXISTS Business_Profile (
    business_id SERIAL PRIMARY KEY,
    location_id INTEGER NOT NULL REFERENCES Location_Metadata(location_id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    business_type TEXT NOT NULL,  -- e.g. restaurant, shop, cafe
    product_type TEXT,
    owner_experience_years INTEGER,
    work_hours_per_day INTEGER,
    work_days_per_week INTEGER,
    store_area_m2 NUMERIC,
    rent_monthly NUMERIC(12,2),
    staff_cost_monthly NUMERIC(12,2),
    utilities_cost_monthly NUMERIC(12,2),
    opening_date DATE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

CREATE INDEX IF NOT EXISTS idx_business_location ON Business_Profile(location_id);

-- ==========================================
-- TABLE: RealTime_Metrics
-- High-frequency data from FPGA / Edge AI sensors
-- ==========================================
CREATE TABLE IF NOT EXISTS RealTime_Metrics (
    metric_id BIGSERIAL PRIMARY KEY,
    location_id INTEGER NOT NULL REFERENCES Location_Metadata(location_id) ON DELETE CASCADE,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    people_count INTEGER,
    dwell_time_avg NUMERIC,
    vehicle_count INTEGER,
    congestion_level TEXT,  -- Added field for crowd density (low/medium/high)
    data_quality_flag SMALLINT DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

CREATE INDEX IF NOT EXISTS idx_realtime_loc_ts ON RealTime_Metrics(location_id, timestamp);

-- ==========================================
-- TABLE: Daily_FPGA_Metrics
-- Aggregated daily metrics derived from RealTime_Metrics
-- ==========================================
CREATE TABLE IF NOT EXISTS Daily_FPGA_Metrics (
    location_id INTEGER NOT NULL REFERENCES Location_Metadata(location_id) ON DELETE CASCADE,
    date DATE NOT NULL,
    daily_footfall INTEGER,
    daily_avg_dwell NUMERIC,
    daily_vehicle_count INTEGER,
    hours_of_operation INTEGER,
    missing_intervals INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now(),
    PRIMARY KEY (location_id, date)
);

-- ==========================================
-- TABLE: Daily_Revenue
-- Stores business-level daily revenue (used for ML labels)
-- ==========================================
CREATE TABLE IF NOT EXISTS Daily_Revenue (
    business_id INTEGER NOT NULL REFERENCES Business_Profile(business_id) ON DELETE CASCADE,
    date DATE NOT NULL,
    total_revenue NUMERIC(14,2),
    total_transactions INTEGER,
    avg_transaction_value NUMERIC(12,2),
    PRIMARY KEY (business_id, date)
);

-- ==========================================
-- TABLE: Prediction_Output
-- Stores ML model predictions for revenue or profit
-- ==========================================
CREATE TABLE IF NOT EXISTS Prediction_Output (
    prediction_id BIGSERIAL PRIMARY KEY,
    business_id INTEGER NOT NULL REFERENCES Business_Profile(business_id) ON DELETE CASCADE,
    location_id INTEGER NOT NULL REFERENCES Location_Metadata(location_id) ON DELETE CASCADE,
    prediction_date DATE,
    predicted_revenue NUMERIC(14,2),
    predicted_profit NUMERIC(14,2),
    confidence_score REAL,
    model_version TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- ==========================================
-- TABLE: Weather
-- External weather data linked by location & timestamp
-- ==========================================
CREATE TABLE IF NOT EXISTS Weather (
    weather_id SERIAL PRIMARY KEY,
    location_id INTEGER NOT NULL REFERENCES Location_Metadata(location_id) ON DELETE CASCADE,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
    temp_c REAL,
    precip_mm REAL,
    is_rain BOOLEAN,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

CREATE INDEX IF NOT EXISTS idx_weather_loc_ts ON Weather(location_id, timestamp);

-- ==========================================
-- TABLE: Promotions
-- Business promotions and their effects
-- ==========================================
CREATE TABLE IF NOT EXISTS Promotions (
    promo_id SERIAL PRIMARY KEY,
    business_id INTEGER NOT NULL REFERENCES Business_Profile(business_id) ON DELETE CASCADE,
    start_date DATE,
    end_date DATE,
    promo_type TEXT,
    expected_lift_pct REAL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- ==========================================
-- TABLE: Events
-- External public events affecting traffic and revenue
-- ==========================================
CREATE TABLE IF NOT EXISTS Events (
    event_id SERIAL PRIMARY KEY,
    name TEXT,
    location_lat DOUBLE PRECISION,
    location_lon DOUBLE PRECISION,
    start_date DATE,
    end_date DATE,
    expected_attendance INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- ==========================================
-- END OF SCHEMA CREATION
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE '✅ Edge AI Location Intelligence schema created successfully.';
END $$;

