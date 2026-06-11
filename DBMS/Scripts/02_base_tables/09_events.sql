-- ============================================================
-- DATA SEED: Events
-- Purpose:
--   Populate the Events table with major planned activities
--   occurring across Greater Cairo to model exogenous demand
--   drivers affecting footfall, traffic, dwell behavior, and
--   commercial performance.
--
-- Description:
--   This insert captures multi-day and single-day events of
--   varying types (cultural, sports, commercial, religious,
--   educational, and business) with geospatial coordinates,
--   duration, and estimated attendance impact.
--
-- Coverage:
--   - Central Cairo (Downtown, Zamalek, Al Azhar Park)
--   - Nasr City and New Cairo
--   - Maadi and surrounding residential-commercial zones
--   - City-wide events with large influence radii
--
-- Key Attributes:
--   name                     → Event name
--   event_type               → Event classification
--                               (cultural, sports, food,
--                                commercial, music, business,
--                                religious, education)
--   location_lat / location_lon
--                            → Event epicenter coordinates
--   start_date / end_date    → Event duration window
--   expected_attendance      → Estimated total visitors
--   event_influence_radius_m → Spatial impact radius (meters)
--
-- Analytical Notes:
--   - Multi-day events (e.g., book fairs, sales campaigns)
--     introduce sustained demand uplift.
--   - Single-day events (concerts, derbies, marathons)
--     generate sharp, short-lived traffic and footfall spikes.
--   - Influence radius enables spatial joins with nearby
--     Location_Metadata for localized impact scoring.
--
-- Intended Use:
--   - External factor enrichment for demand & profit models
--   - Event-driven anomaly detection in footfall data
--   - Scenario simulation and forecasting
--   - Urban mobility and congestion analysis
-- ============================================================

TRUNCATE TABLE Events RESTART IDENTITY CASCADE;

INSERT INTO Events
(name, event_type, location_lat, location_lon, start_date, end_date, expected_attendance, event_influence_radius_m)
VALUES
('Cairo International Book Fair','cultural',30.0116,31.2089,'2024-09-15','2024-09-28',1200000,5000),

('Zamalek Derby Screening','sports',30.0611,31.2197,'2024-09-22','2024-09-22',35000,3000),

('Downtown Street Food Festival','food',30.0444,31.2357,'2024-09-20','2024-09-23',180000,2500),

('Nasr City Mall Clearance Sale','commercial',30.0561,31.3300,'2024-09-18','2024-09-25',220000,4000),

('Open-Air Concert at Al Azhar Park','music',30.0408,31.2623,'2024-09-21','2024-09-21',15000,2000),

('Tech Startup Meetup Cairo','business',30.0735,31.3451,'2024-09-24','2024-09-24',1200,1500),

('Friday Prayer Overflow','religious',30.0449,31.2499,'2024-09-20','2024-09-20',80000,1800),

('University Orientation Week','education',30.0645,31.2786,'2024-09-17','2024-09-24',60000,3500),

('Night Market – Maadi','commercial',29.9602,31.2569,'2024-09-22','2024-09-23',22000,2000),

('Charity Marathon – New Cairo','sports',30.0276,31.4966,'2024-09-29','2024-09-29',9000,4500);

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Events table inserted successfully ✅';
END $$;