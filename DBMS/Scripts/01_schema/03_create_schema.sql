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
	competitor_density NUMERIC(10,4), -- competitor_density = competitor_count / (PI * (catchment_radius_m / 1000)^2)
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
-- TABLE: Hourly_FPGA_Metrics
-- Aggregated hourly metrics derived from RealTime_Metrics
-- This directly supports:
	-- Peak-hour intensity
	-- Demand volatility
	-- Better feasibility scoring
-- ==========================================
CREATE TABLE Hourly_FPGA_Metrics (
    location_id INT REFERENCES location_metadata(location_id),
    date DATE,
    hour INT,
    hourly_footfall INT,
    hourly_avg_dwell NUMERIC,
    peak_hour_flag BOOLEAN
);

-- ==========================================
-- TABLE: behavior_trends
-- Stores engineered temporal behavior features derived from historical FPGA metrics,
-- including footfall trends, growth rates, and activity volatility indicators.
-- Used for time-series analysis and feasibility prediction models.
-- ==========================================
CREATE TABLE IF NOT EXISTS behavior_trends (
  location_id INT REFERENCES location_metadata(location_id),
  date DATE,
  footfall_7d_avg NUMERIC(10,2),
  footfall_growth_rate NUMERIC(6,3),
  dwell_trend_slope NUMERIC(6,3),
  traffic_volatility_index NUMERIC(6,3),
  trend_window_days INT
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
	revenue_per_visitor NUMERIC,
	transactions_per_hour NUMERIC,
    PRIMARY KEY (business_id, date)
);

-- ==========================================
-- TABLE: operational_efficiency
-- Stores derived business efficiency metrics calculated from operational costs
-- and revenue data, providing normalized indicators of profitability and cost structure.
-- Used to compare performance across different business sizes and locations.
-- ==========================================
CREATE TABLE IF NOT EXISTS operational_efficiency (
  business_id INT REFERENCES business_profile(business_id),
  date DATE,
  staff_cost_ratio NUMERIC(6,3),
  rent_cost_ratio NUMERIC(6,3),
  profit_margin_est NUMERIC(6,3)
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
-- TABLE: Events
-- External public events affecting traffic and revenue
-- ==========================================
CREATE TABLE IF NOT EXISTS Events (
    event_id SERIAL PRIMARY KEY,
    name TEXT,
	event_type TEXT,
    location_lat DOUBLE PRECISION,
    location_lon DOUBLE PRECISION,
    start_date DATE,
    end_date DATE,
    expected_attendance INTEGER,
	event_influence_radius_m INT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

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
-- TABLE: feature_store
-- Centralized repository for precomputed feature vectors used by machine learning models.
-- Ensures feature consistency, reproducibility, and separation between raw data
-- and model-ready inputs.
-- ==========================================
CREATE TABLE IF NOT EXISTS feature_store (
  feature_id BIGSERIAL PRIMARY KEY,
  business_id INT,
  location_id INT,
  date DATE,
  feature_vector_hash TEXT,
  generated_at TIMESTAMP DEFAULT NOW()
);

-- ==========================================
-- TABLE: model_metadata
-- Stores metadata and evaluation results for trained machine learning models,
-- including training periods, feature versions, and performance metrics.
-- Enables model traceability, version control, and auditability.
-- ==========================================
CREATE TABLE IF NOT EXISTS model_metadata (
  model_version TEXT PRIMARY KEY,
  model_type TEXT,
  training_start_date DATE,
  training_end_date DATE,
  feature_set_version TEXT,
  evaluation_mae NUMERIC(8,4),
  evaluation_r2 NUMERIC(6,4)
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
	feasibility_score NUMERIC(4,2),
    confidence_score REAL,
    model_version TEXT REFERENCES model_metadata(model_version),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now()
);

-- ==========================================
-- TABLE: data_quality_log
-- Records data completeness and anomaly detection results across source tables.
-- Used to monitor data reliability and prevent low-quality inputs
-- from affecting analytics and model predictions.
-- ==========================================
CREATE TABLE IF NOT EXISTS data_quality_log (
  log_id BIGSERIAL PRIMARY KEY,
  source_table TEXT,
  record_date DATE,
  completeness_pct NUMERIC(5,2),
  anomaly_flag BOOLEAN
);

-- ==========================================
-- END OF SCHEMA CREATION
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE '✅ Edge AI Location Intelligence schema created successfully.';
END $$;

