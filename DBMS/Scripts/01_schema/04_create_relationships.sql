-- BUSINESS_PROFILE → LOCATION_METADATA
ALTER TABLE Business_Profile
ADD CONSTRAINT fk_business_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- REALTIME_METRICS → LOCATION_METADATA
ALTER TABLE RealTime_Metrics
ADD CONSTRAINT fk_metrics_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- DAILY_FPGA_METRICS → LOCATION_METADATA
ALTER TABLE Daily_FPGA_Metrics
ADD CONSTRAINT fk_daily_fpga_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- DAILY_REVENUE → BUSINESS_PROFILE
ALTER TABLE Daily_Revenue
ADD CONSTRAINT fk_revenue_business
FOREIGN KEY (business_id)
REFERENCES Business_Profile(business_id)
ON DELETE CASCADE;

-- PREDICTION_OUTPUT → BUSINESS_PROFILE
ALTER TABLE Prediction_Output
ADD CONSTRAINT fk_prediction_business
FOREIGN KEY (business_id)
REFERENCES Business_Profile(business_id)
ON DELETE CASCADE;

-- PREDICTION_OUTPUT → LOCATION_METADATA
ALTER TABLE Prediction_Output
ADD CONSTRAINT fk_prediction_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- PROMOTIONS → BUSINESS_PROFILE
ALTER TABLE Promotions
ADD CONSTRAINT fk_promotions_business
FOREIGN KEY (business_id)
REFERENCES Business_Profile(business_id)
ON DELETE CASCADE;

-- WEATHER → LOCATION_METADATA
ALTER TABLE Weather
ADD CONSTRAINT fk_weather_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- BEHAVIOR_TRENDS → LOCATION_METADATA
ALTER TABLE behavior_trends
ADD CONSTRAINT fk_behavior_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;

-- OPERATIONAL_EFFICIENCY → BUSINESS_PROFILE
ALTER TABLE operational_efficiency
ADD CONSTRAINT fk_efficiency_business
FOREIGN KEY (business_id)
REFERENCES Business_Profile(business_id)
ON DELETE CASCADE;

-- FEATURE_STORE → BUSINESS_PROFILE
ALTER TABLE feature_store
ADD CONSTRAINT fk_feature_business
FOREIGN KEY (business_id)
REFERENCES Business_Profile(business_id)
ON DELETE CASCADE;

-- FEATURE_STORE → LOCATION_METADATA
ALTER TABLE feature_store
ADD CONSTRAINT fk_feature_location
FOREIGN KEY (location_id)
REFERENCES Location_Metadata(location_id)
ON DELETE CASCADE;