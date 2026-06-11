-- ============================================================
-- DATA SEED: Promotions
-- Purpose:
--   Populate the Promotions table with time-bound marketing
--   campaigns linked to individual businesses in order to
--   model promotional uplift and demand stimulation effects.
--
-- Description:
--   This insert defines multiple promotional activities with
--   varying durations, incentive mechanisms, and expected
--   performance lift percentages. Promotions are associated
--   with specific businesses and overlap with real-world
--   seasonal, event-driven, and tactical marketing periods.
--
-- Key Attributes:
--   business_id        → FK reference to Business_Profile
--   start_date         → Promotion start date
--   end_date           → Promotion end date
--   promo_type         → Promotion mechanism
--                        (discount, BOGO, clearance, loyalty,
--                         seasonal, student, flash, bundle)
--   expected_lift_pct  → Anticipated percentage increase in
--                        demand or revenue due to promotion
--
-- Analytical Notes:
--   - Promotions can overlap with events and weather effects,
--     enabling interaction analysis.
--   - expected_lift_pct is an estimate used for simulation,
--     forecasting, and model training—not actual realized lift.
--   - Duration length and promo type allow differentiation
--     between short spikes and sustained uplift.
--
-- Intended Use:
--   - Feature enrichment for demand and profit prediction
--   - Promotion effectiveness and ROI modeling
--   - Scenario testing and what-if analysis
--   - Attribution modeling in multi-factor analytics
-- ============================================================


TRUNCATE TABLE Promotions RESTART IDENTITY CASCADE;

INSERT INTO Promotions
(business_id, start_date, end_date, promo_type, expected_lift_pct)
VALUES
(201,'2024-09-20','2024-09-22','discount_10pct',8.5),
(202,'2024-09-21','2024-09-23','buy_one_get_one',15.0),
(203,'2024-09-18','2024-09-25','clearance_sale',22.0),
(204,'2024-09-22','2024-09-22','flash_sale',12.0),
(205,'2024-09-19','2024-09-26','bundle_offer',9.0),

(206,'2024-09-23','2024-09-24','student_discount',6.5),
(207,'2024-09-20','2024-09-30','loyalty_points_boost',5.0),
(208,'2024-09-24','2024-09-24','happy_hour',10.0),
(209,'2024-09-22','2024-09-28','seasonal_offer',7.5),
(210,'2024-09-25','2024-09-27','weekend_promo',11.0),

-- Restaurants & Cafés
(201,'2024-09-25','2024-09-30','happy_hour',9.0),
(202,'2024-09-26','2024-10-02','combo_meal',11.5),
(203,'2024-10-01','2024-10-07','seasonal_menu',6.0),
(204,'2024-09-28','2024-09-28','flash_sale',13.0),
(205,'2024-10-03','2024-10-10','loyalty_bonus',4.5),

-- Retail
(211,'2024-09-22','2024-10-05','clearance_sale',24.0),
(212,'2024-09-29','2024-10-06','buy_two_get_one',14.0),
(213,'2024-10-01','2024-10-15','seasonal_offer',8.0),
(214,'2024-09-27','2024-09-30','discount_15pct',12.0),

-- Services
(215,'2024-09-24','2024-10-01','new_customer_offer',10.0),
(216,'2024-10-02','2024-10-09','referral_bonus',5.5),

-- Overlapping promos (intentional edge cases)
(201,'2024-09-27','2024-09-29','weekend_promo',7.0),
(203,'2024-10-05','2024-10-06','flash_sale',16.0),

-- Low-impact noise promos
(217,'2024-09-23','2024-09-30','branding_campaign',2.0),
(218,'2024-09-25','2024-10-25','awareness_offer',1.5),

-- High impact but short
(219,'2024-10-04','2024-10-04','grand_opening',30.0),
(220,'2024-10-06','2024-10-06','one_day_sale',18.0);

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Promotions table inserted successfully ✅';
END $$;