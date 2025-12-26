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
