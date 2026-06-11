-- ============================================================
-- DATA: Business_Profile
-- Purpose:
--   Populate the Business_Profile table with detailed operational,
--   financial, and experiential attributes for small and
--   medium-sized businesses linked to Location_Metadata.
--
-- Key Attributes:
--   location_id               → FK reference to Location_Metadata
--   name                      → Business name
--   business_type             → High-level category
--                               (cafe, restaurant, retail,
--                                grocery, pharmacy, service)
--   product_type              → Primary product/service focus
--   owner_experience_years    → Years of owner experience
--   work_hours_per_day        → Daily operating hours
--   work_days_per_week        → Weekly operating days
--   store_area_m2             → Physical store size (m²)
--   rent_monthly              → Monthly rent cost (EGP)
--   staff_cost_monthly        → Monthly staff payroll (EGP)
--   utilities_cost_monthly    → Monthly utilities cost (EGP)
--   opening_date              → Business start date
--
-- Intended Use:
--   - Profitability and cost-structure analysis
--   - Owner experience vs. performance modeling
--   - Feature engineering for revenue and survival prediction
--   - Scenario simulation for SME growth and risk assessment
--
-- Notes:
--   • Data is synthetic but internally consistent.
--   • Values are calibrated to reflect realistic Egyptian
--     SME operating conditions.
--   • Designed for analytics, dashboards, and ML pipelines
--     rather than transactional accuracy.
-- ============================================================

TRUNCATE TABLE Business_Profile RESTART IDENTITY CASCADE;

INSERT INTO Business_Profile
(location_id, name, business_type, product_type, owner_experience_years,
 work_hours_per_day, work_days_per_week, store_area_m2,
 rent_monthly, staff_cost_monthly, utilities_cost_monthly, opening_date)
VALUES
(1, 'Talaat Harb Cafe', 'cafe', 'coffee_beverages', 8, 14, 7, 85, 22000, 18000, 4200, '2019-03-15'),
(2, 'Downtown Express Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 60, 18000, 14000, 3500, '2018-07-10'),
(3, 'Abdeen Grill House', 'restaurant', 'grilled_food', 10, 13, 7, 120, 26000, 24000, 5200, '2020-01-20'),
(4, 'Ramses Mobile Center', 'retail', 'electronics', 6, 11, 7, 45, 15000, 9000, 2800, '2021-05-05'),
(5, 'El Gomhoreya Stationery', 'retail', 'office_supplies', 9, 10, 6, 55, 13000, 8000, 2200, '2017-09-12'),

(6, 'Shubra Fresh Market', 'grocery', 'food_items', 15, 14, 7, 140, 21000, 26000, 4800, '2016-11-03'),
(7, 'Rod El Farag Fish Shop', 'retail', 'seafood', 18, 13, 7, 90, 17000, 15000, 3600, '2015-04-18'),
(8, 'Shubra Electronics Hub', 'retail', 'home_appliances', 7, 11, 6, 75, 16000, 12000, 3000, '2021-02-14'),
(9, 'Massara Bakery', 'bakery', 'bread_pastries', 20, 15, 7, 65, 14000, 16000, 3900, '2014-06-01'),
(10, 'Khalafawy Coffee Corner', 'cafe', 'hot_drinks', 5, 12, 7, 50, 12000, 9000, 2500, '2022-08-09'),

(11, 'Dokki Healthy Bites', 'restaurant', 'healthy_food', 6, 12, 6, 95, 24000, 21000, 4700, '2020-10-22'),
(12, 'Dokki Optical Center', 'retail', 'eyewear', 14, 10, 6, 70, 19000, 13000, 3200, '2016-03-30'),
(13, 'Sudan Street Shawarma', 'restaurant', 'fast_food', 9, 14, 7, 80, 20000, 19000, 4100, '2019-12-05'),
(14, 'Mossadak Bookstore', 'retail', 'books', 11, 9, 6, 60, 17000, 10000, 2600, '2017-05-19'),
(15, 'Shooting Club Juice Bar', 'cafe', 'fresh_juice', 4, 10, 6, 45, 15000, 8500, 2400, '2023-01-11'),

(17, 'Lebanon Square Bistro', 'restaurant', 'international_food', 13, 13, 7, 130, 28000, 26000, 5600, '2018-09-02'),
(16, 'Gameat El Dowal Fashion', 'retail', 'clothing', 10, 11, 7, 85, 22000, 15000, 3300, '2019-04-27'),
(18, 'Shehab Electronics Repair', 'service', 'device_repair', 16, 10, 6, 50, 16000, 11000, 2800, '2015-11-16'),
(19, 'Arab League Perfumes', 'retail', 'perfumes', 8, 11, 7, 40, 21000, 9000, 2500, '2021-06-08'),
(20, 'Zamalek Bridge Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 18000, 13000, 3200, '2020-08-14'),

(21, 'Abbas El Akkad Grill', 'restaurant', 'grilled_food', 14, 14, 7, 150, 30000, 28000, 6000, '2017-02-10'),
(22, 'Makram Ebeid Mobile Shop', 'retail', 'mobile_accessories', 7, 11, 7, 60, 19000, 12000, 2900, '2021-09-01'),
(23, 'Mustafa El Nahas Supermarket', 'grocery', 'food_items', 18, 15, 7, 180, 27000, 32000, 6500, '2016-05-07'),
(24, 'Nasr City Coffee Lounge', 'cafe', 'coffee_beverages', 5, 12, 7, 70, 22000, 16000, 3800, '2022-03-18'),
(25, 'El Serag Mall Kiosk', 'retail', 'snacks', 3, 10, 7, 30, 14000, 7000, 2000, '2023-06-02'),

(26, 'Korba Fine Dining', 'restaurant', 'fine_dining', 15, 12, 6, 160, 32000, 30000, 6800, '2016-12-21'),
(27, 'Baghdad Street Pharmacy', 'pharmacy', 'medicines', 11, 12, 7, 65, 20000, 14000, 3600, '2018-04-09'),
(28, 'El Merghany Fashion House', 'retail', 'clothing', 9, 11, 6, 90, 24000, 17000, 3900, '2019-07-25'),
(29, 'Roxy Dessert Cafe', 'cafe', 'desserts', 6, 12, 7, 75, 23000, 16000, 4100, '2021-11-04'),
(30, 'Almaza Organic Market', 'grocery', 'organic_food', 4, 10, 6, 85, 21000, 15000, 3400, '2022-02-16'),

(32, 'Ain Shams Book Center', 'retail', 'books', 13, 9, 6, 70, 16000, 9000, 2300, '2017-01-08'),
(33, 'Ain Shams Student Cafe', 'cafe', 'coffee_snacks', 5, 14, 7, 60, 18000, 12000, 3100, '2021-10-12'),
(34, 'Ain Shams Copy & Print', 'service', 'printing', 17, 10, 6, 55, 15000, 10000, 2600, '2015-09-03'),
(31, 'El Matareya Grocery', 'grocery', 'food_items', 20, 14, 7, 130, 19000, 24000, 4800, '2014-04-26'),
(35, 'El Zeraeiin Hardware', 'retail', 'hardware_tools', 22, 11, 6, 95, 17000, 16000, 3500, '2013-08-17'),

(51, 'Helwan Corniche Cafe', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16000, 14000, 3300, '2019-05-14'),
(52, 'Helwan University Snacks', 'restaurant', 'fast_food', 6, 14, 7, 65, 15000, 13000, 3000, '2020-09-09'),
(53, '15th of May Mini Market', 'grocery', 'food_items', 12, 13, 7, 110, 17000, 19000, 4100, '2018-02-22'),
(54, 'Masaken El Shabab Pharmacy', 'pharmacy', 'medicines', 10, 12, 7, 60, 14000, 12000, 2800, '2017-06-06'),
(55, 'Helwan Fish Grill', 'restaurant', 'seafood', 16, 14, 7, 125, 19000, 23000, 5200, '2016-10-01'),

(56, 'Agouza Nile View Cafe', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 20000, 15000, 3600, '2020-04-19'),
(57, 'Mit Akaba Market Store', 'retail', 'household_goods', 14, 11, 6, 85, 17000, 13000, 3000, '2017-08-28'),
(58, 'El Mohandiseen Repair Hub', 'service', 'electronics_repair', 19, 10, 6, 60, 16000, 14000, 3100, '2014-12-13'),
(59, 'Ard El Lewa Grocery', 'grocery', 'food_items', 21, 14, 7, 140, 18000, 26000, 5200, '2013-03-21'),
(60, 'Corniche View Restaurant', 'restaurant', 'international_food', 12, 13, 7, 145, 25000, 27000, 5900, '2018-11-30'),

(61, 'Kit Kat Square Koshary', 'restaurant', 'egyptian_food', 18, 14, 7, 90, 17000, 20000, 4200, '2015-02-07'),
(62, 'Ard El Lewa Mobile Repair', 'service', 'mobile_repair', 11, 10, 6, 50, 14000, 10000, 2500, '2019-06-18'),
(63, 'El Batal Hardware Store', 'retail', 'hardware_tools', 23, 11, 6, 100, 16000, 18000, 3600, '2012-10-29'),
(64, 'Geziret Badran Supermarket', 'grocery', 'food_items', 16, 14, 7, 150, 19000, 28000, 5400, '2016-01-16'),
(65, 'Imbaba Market Grill', 'restaurant', 'grilled_food', 20, 15, 7, 130, 18000, 25000, 5600, '2014-07-03'),

(66, 'El Khalifa Cafe', 'cafe', 'coffee_beverages', 8, 12, 7, 65, 15000, 12000, 2900, '2019-09-14'),
(67, 'Sayeda Zeinab Pharmacy', 'pharmacy', 'medicines', 13, 12, 7, 60, 14000, 13000, 3000, '2017-12-01'),
(68, 'Port Said Street Electronics', 'retail', 'electronics', 9, 11, 6, 70, 16000, 12000, 2800, '2020-06-20'),
(69, 'Sayeda Zeinab Market Grocery', 'grocery', 'food_items', 22, 14, 7, 160, 18000, 30000, 5800, '2013-05-09'),
(70, 'Corniche Axis Juice Bar', 'cafe', 'fresh_juice', 6, 11, 7, 55, 14000, 9000, 2400, '2021-08-27'),

(71, 'Bulaq Corniche Cafe', 'cafe', 'coffee_beverages', 10, 12, 7, 75, 17000, 14000, 3300, '2018-04-05'),
(72, 'Maspero Mini Market', 'grocery', 'food_items', 17, 14, 7, 135, 20000, 27000, 5600, '2016-09-11'),
(73, 'Abou El Ela Restaurant', 'restaurant', 'egyptian_food', 19, 14, 7, 140, 21000, 26000, 5900, '2015-01-24'),
(74, 'Bulaq Repair Services', 'service', 'home_appliance_repair', 15, 10, 6, 65, 15000, 12000, 2800, '2017-07-07'),
(75, 'Railway Axis Snacks', 'restaurant', 'snacks', 7, 12, 7, 55, 16000, 13000, 3100, '2020-12-19'),

(77, 'Fustat Heritage Crafts', 'retail', 'handicrafts', 18, 10, 6, 90, 17000, 15000, 3300, '2016-03-14'),
(78, 'Ain El Sira Grocery', 'grocery', 'food_items', 21, 14, 7, 150, 18000, 28000, 5400, '2013-10-02'),
(79, 'Magra El Oyoun Butchery', 'retail', 'meat_products', 24, 12, 7, 80, 16000, 17000, 3600, '2012-06-26'),
(80, 'Old Cairo Coffee House', 'cafe', 'coffee_beverages', 11, 12, 7, 70, 15000, 12000, 3000, '2018-02-17'),
(76, 'Old Cairo Market Spices', 'retail', 'spices', 27, 11, 6, 65, 14000, 15000, 2800, '2011-04-04'),

(91, 'Warraq Island Grocery', 'grocery', 'food_items', 19, 14, 7, 160, 17000, 29000, 5500, '2015-08-23'),
(92, 'Warraq Market Grill', 'restaurant', 'grilled_food', 21, 15, 7, 140, 18000, 26000, 5700, '2014-11-08'),
(93, 'Warraq Electronics Repair', 'service', 'electronics_repair', 16, 10, 6, 60, 15000, 13000, 2900, '2016-05-30'),
(94, 'Warraq Ferry Snacks', 'restaurant', 'snacks', 8, 12, 7, 50, 14000, 11000, 2600, '2021-07-12'),
(95, 'Warraq Corniche Cafe', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 16000, 14000, 3300, '2019-01-06'),

(96, 'Bab El Shaaria Electronics', 'retail', 'electronics', 14, 11, 6, 85, 17000, 14000, 3200, '2017-04-15'),
(97, 'Bab El Shaaria Market Grocery', 'grocery', 'food_items', 23, 14, 7, 170, 19000, 32000, 6000, '2012-09-09'),
(98, 'Port Said Axis Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 65, 15000, 13000, 3000, '2018-12-20'),
(99, 'Sabtiya Auto Parts', 'retail', 'auto_parts', 26, 11, 6, 110, 16000, 18000, 3500, '2011-05-28'),
(100, 'Railway Zone Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 15000, 12000, 2800, '2022-03-02'),

(101, 'El Salam Corner Grocery', 'grocery', 'food_items', 20, 14, 7, 160, 17000, 30000, 5800, '2014-08-16'),
(102, 'El Salam Market Grill', 'restaurant', 'grilled_food', 22, 15, 7, 150, 18000, 27000, 6000, '2013-02-11'),
(103, 'El Salam Industrial Supplies', 'retail', 'industrial_tools', 18, 11, 6, 120, 16000, 20000, 3800, '2016-06-05'),
(104, 'El Salam Transport Snacks', 'restaurant', 'snacks', 7, 12, 7, 50, 14000, 11000, 2600, '2021-10-21'),
(105, 'Ring Road Coffee Stop', 'cafe', 'coffee_beverages', 5, 12, 7, 60, 15000, 12000, 2900, '2022-07-18'),

(306, 'Mashtoul Main Grocery', 'grocery', 'food_items', 18, 14, 7, 155, 18000, 28000, 5500, '2015-03-19'),
(307, 'Mashtoul Market Butchery', 'retail', 'meat_products', 24, 12, 7, 90, 16000, 17000, 3600, '2012-11-07'),
(308, 'Mashtoul Repair Services', 'service', 'general_repair', 16, 10, 6, 65, 15000, 13000, 2800, '2016-04-25'),
(309, 'Mashtoul Transport Snacks', 'restaurant', 'snacks', 7, 12, 7, 55, 14000, 11000, 2600, '2021-06-30'),
(310, 'Mashtoul Coffee Corner', 'cafe', 'coffee_beverages', 9, 12, 7, 70, 15000, 13000, 3000, '2019-10-08'),

(311, 'Zagazig Road Mini Market', 'grocery', 'food_items', 21, 14, 7, 165, 19000, 31000, 5900, '2013-01-22'),
(312, 'Zagazig Road Grill', 'restaurant', 'grilled_food', 23, 15, 7, 145, 18000, 26000, 5800, '2012-05-17'),
(313, 'Zagazig Road Electronics', 'retail', 'electronics', 13, 11, 6, 80, 17000, 14000, 3200, '2017-07-29'),
(314, 'Zagazig Road Repair Hub', 'service', 'electronics_repair', 17, 10, 6, 60, 15000, 13000, 2800, '2015-09-13'),
(315, 'Zagazig Road Coffee Stop', 'cafe', 'coffee_beverages', 8, 12, 7, 70, 15000, 12000, 2900, '2020-11-05'),

(281, 'Qaha Station Grocery', 'grocery', 'food_items', 19, 14, 7, 160, 18000, 29000, 5600, '2014-02-27'),
(282, 'Qaha Market Grill', 'restaurant', 'grilled_food', 22, 15, 7, 150, 18000, 27000, 6000, '2013-06-18'),
(283, 'Qaha Electronics Repair', 'service', 'electronics_repair', 15, 10, 6, 60, 15000, 13000, 2800, '2016-08-09'),
(284, 'Qaha Coffee Corner', 'cafe', 'coffee_beverages', 7, 12, 7, 65, 14000, 12000, 2800, '2021-01-14'),
(285, 'Qaha Pharmacy', 'pharmacy', 'medicines', 11, 12, 7, 60, 15000, 13000, 3000, '2018-03-03'),

(286, 'Tukh Central Grocery', 'grocery', 'food_items', 20, 14, 7, 170, 19000, 32000, 6100, '2013-12-06'),
(287, 'Tukh Market Fish', 'retail', 'seafood', 25, 13, 7, 95, 17000, 18000, 3700, '2011-07-21'),
(288, 'Tukh Electronics', 'retail', 'electronics', 14, 11, 6, 85, 16000, 14000, 3200, '2017-10-02'),
(289, 'Tukh Repair Services', 'service', 'general_repair', 18, 10, 6, 65, 15000, 13000, 2800, '2015-04-11'),
(290, 'Tukh Coffee House', 'cafe', 'coffee_beverages', 9, 12, 7, 70, 15000, 12000, 2900, '2019-06-24'),

(36, 'Zamalek Artisan Bakery', 'bakery', 'bread_pastries', 14, 12, 6, 70, 24000, 17000, 3800, '2017-02-12'),
(37, 'Brazil Street Cafe', 'cafe', 'coffee_beverages', 6, 12, 7, 60, 23000, 15000, 3500, '2021-06-19'),
(38, 'Ismail Mohamed Boutique', 'retail', 'clothing', 11, 10, 6, 85, 26000, 16000, 3400, '2018-09-03'),
(39, 'Gezira Club Snacks', 'restaurant', 'snacks', 5, 12, 7, 50, 20000, 12000, 3000, '2022-04-27'),
(40, 'Abou El Feda Pharmacy', 'pharmacy', 'medicines', 13, 12, 7, 65, 22000, 14000, 3600, '2016-11-15'),

(41, 'Garden City Gourmet', 'restaurant', 'international_food', 16, 12, 6, 140, 30000, 29000, 6200, '2016-01-18'),
(42, 'Simon Bolivar Mini Market', 'grocery', 'food_items', 19, 14, 7, 150, 21000, 30000, 5600, '2014-07-22'),
(43, 'Corniche Nile View Cafe', 'cafe', 'coffee_beverages', 8, 12, 7, 80, 26000, 17000, 4100, '2019-10-05'),
(44, 'Embassy Area Flowers', 'retail', 'flowers', 12, 10, 6, 55, 24000, 11000, 2700, '2017-05-09'),
(45, 'Falaky Square Juice Bar', 'cafe', 'fresh_juice', 6, 11, 7, 50, 22000, 10000, 2600, '2020-08-16'),

(46, 'Ahmed El Zomor Grill', 'restaurant', 'grilled_food', 15, 14, 7, 145, 25000, 27000, 5800, '2016-04-02'),
(47, 'Zahraa Nasr Grocery', 'grocery', 'food_items', 18, 14, 7, 165, 19000, 30000, 5600, '2015-01-14'),
(48, 'Waha Mobile Center', 'retail', 'mobile_accessories', 7, 11, 7, 60, 17000, 12000, 2800, '2021-03-21'),
(49, 'El Hay El Asher Cafe', 'cafe', 'coffee_beverages', 5, 12, 7, 65, 18000, 14000, 3200, '2022-07-11'),
(50, 'El Hay El Sabea Pharmacy', 'pharmacy', 'medicines', 10, 12, 7, 60, 16000, 13000, 3000, '2018-12-29'),

(51, 'Helwan Corniche Seafood', 'restaurant', 'seafood', 17, 14, 7, 150, 21000, 26000, 6000, '2015-06-07'),
(52, 'Helwan University Print Shop', 'service', 'printing', 21, 10, 6, 55, 15000, 11000, 2600, '2013-09-18'),
(53, '15th of May Grocery', 'grocery', 'food_items', 16, 14, 7, 145, 18000, 28000, 5400, '2016-02-03'),
(54, 'Masaken El Shabab Cafe', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 16000, 13000, 3000, '2020-11-20'),
(55, 'Helwan Fish Market Grill', 'restaurant', 'grilled_food', 22, 15, 7, 160, 19000, 29000, 6200, '2013-08-26'),

(56, 'Agouza Nile Grocery', 'grocery', 'food_items', 20, 14, 7, 155, 19000, 30000, 5700, '2014-03-12'),
(57, 'Mit Akaba Hardware', 'retail', 'hardware_tools', 24, 11, 6, 110, 16000, 19000, 3600, '2012-05-19'),
(58, 'El Mohandiseen Cafe', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 19000, 15000, 3500, '2019-09-08'),
(59, 'Ard El Lewa Market Grocery', 'grocery', 'food_items', 23, 14, 7, 170, 18000, 32000, 6100, '2013-04-23'),
(60, 'Corniche View Pastry', 'bakery', 'desserts', 14, 11, 6, 65, 20000, 14000, 3300, '2017-11-30'),

(61, 'Kit Kat Electronics', 'retail', 'electronics', 13, 11, 6, 90, 17000, 15000, 3200, '2018-06-02'),
(62, 'Ard El Lewa Phone Repair', 'service', 'mobile_repair', 15, 10, 6, 55, 14000, 12000, 2700, '2016-10-15'),
(63, 'Geziret Badran Grocery', 'grocery', 'food_items', 21, 14, 7, 160, 19000, 30000, 5600, '2014-12-07'),
(64, 'Imbaba Street Grill', 'restaurant', 'grilled_food', 19, 15, 7, 145, 18000, 27000, 6000, '2015-07-09'),
(65, 'Imbaba Juice Stop', 'cafe', 'fresh_juice', 6, 11, 7, 55, 15000, 10000, 2400, '2021-05-28'),

(66, 'El Khalifa Coffee House', 'cafe', 'coffee_beverages', 8, 12, 7, 70, 15000, 12000, 2900, '2019-08-17'),
(67, 'Sayeda Zeinab Grocery', 'grocery', 'food_items', 22, 14, 7, 165, 18000, 31000, 6000, '2013-02-04'),
(68, 'Port Said Street Phones', 'retail', 'mobile_phones', 11, 11, 6, 75, 16000, 13000, 3000, '2018-09-12'),
(69, 'Sayeda Zeinab Sweets', 'bakery', 'traditional_sweets', 18, 12, 6, 60, 15000, 16000, 3300, '2015-01-27'),
(70, 'Corniche Axis Snacks', 'restaurant', 'snacks', 7, 12, 7, 55, 16000, 12000, 2800, '2020-10-01'),

(71, 'Bulaq River Cafe', 'cafe', 'coffee_beverages', 10, 12, 7, 75, 17000, 14000, 3300, '2018-03-14'),
(72, 'Maspero Daily Grocery', 'grocery', 'food_items', 19, 14, 7, 160, 20000, 30000, 5800, '2014-09-06'),
(73, 'Abou El Ela Koshary', 'restaurant', 'egyptian_food', 21, 15, 7, 140, 19000, 26000, 5900, '2013-11-18'),
(74, 'Bulaq Electronics Fix', 'service', 'electronics_repair', 16, 10, 6, 60, 15000, 13000, 2800, '2016-06-21'),
(75, 'Railway Axis Coffee Stop', 'cafe', 'coffee_beverages', 6, 12, 7, 65, 16000, 12000, 2900, '2021-12-03'),

(77, 'Fustat Craft Souvenirs', 'retail', 'handicrafts', 20, 10, 6, 95, 17000, 16000, 3400, '2014-05-29'),
(78, 'Ain El Sira Grocery', 'grocery', 'food_items', 23, 14, 7, 170, 18000, 32000, 6100, '2012-08-11'),
(79, 'Magra El Oyoun Butchery', 'retail', 'meat_products', 26, 12, 7, 85, 16000, 18000, 3600, '2011-10-02'),
(80, 'Old Cairo Tea House', 'cafe', 'tea_drinks', 12, 12, 7, 70, 15000, 13000, 3000, '2018-01-20'),
(76, 'Old Cairo Spice Market', 'retail', 'spices', 28, 11, 6, 70, 14000, 16000, 2800, '2010-06-17'),

(91, 'Warraq Island Supermarket', 'grocery', 'food_items', 20, 14, 7, 165, 18000, 30000, 5800, '2014-02-09'),
(92, 'Warraq Grill House', 'restaurant', 'grilled_food', 22, 15, 7, 150, 18000, 27000, 6000, '2013-07-28'),
(93, 'Warraq Phone Repair', 'service', 'mobile_repair', 14, 10, 6, 55, 14000, 12000, 2600, '2017-04-16'),
(94, 'Warraq Ferry Snacks', 'restaurant', 'snacks', 7, 12, 7, 50, 14000, 11000, 2500, '2021-09-05'),
(95, 'Warraq Corniche Cafe', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 16000, 14000, 3300, '2019-12-12'),

(96, 'Bab El Shaaria Electronics', 'retail', 'electronics', 15, 11, 6, 90, 17000, 15000, 3200, '2017-02-18'),
(97, 'Bab El Shaaria Market Grocery', 'grocery', 'food_items', 24, 14, 7, 175, 19000, 33000, 6200, '2012-03-07'),
(98, 'Port Said Axis Pharmacy', 'pharmacy', 'medicines', 13, 12, 7, 65, 15000, 13000, 3000, '2018-10-26'),
(99, 'Sabtiya Auto Parts', 'retail', 'auto_parts', 27, 11, 6, 120, 16000, 19000, 3600, '2011-04-03'),
(100, 'Railway Zone Snack Bar', 'restaurant', 'snacks', 6, 12, 7, 55, 15000, 12000, 2700, '2022-01-15'),

(101, 'El Salam Main Grocery', 'grocery', 'food_items', 21, 14, 7, 165, 18000, 31000, 5900, '2013-09-21'),
(102, 'El Salam Grill Corner', 'restaurant', 'grilled_food', 23, 15, 7, 150, 18000, 27000, 6000, '2012-12-11'),
(103, 'El Salam Industrial Tools', 'retail', 'industrial_tools', 19, 11, 6, 130, 16000, 21000, 3900, '2016-07-08'),
(104, 'El Salam Transport Snacks', 'restaurant', 'snacks', 7, 12, 7, 50, 14000, 11000, 2500, '2021-11-23'),
(105, 'Ring Road Coffee Point', 'cafe', 'coffee_beverages', 5, 12, 7, 60, 15000, 12000, 2900, '2022-05-30'),

(306, 'Mashtoul Central Grocery', 'grocery', 'food_items', 19, 14, 7, 160, 18000, 30000, 5800, '2014-01-09'),
(307, 'Mashtoul Market Fish', 'retail', 'seafood', 25, 13, 7, 95, 17000, 18000, 3700, '2011-06-28'),
(308, 'Mashtoul Repair Services', 'service', 'general_repair', 17, 10, 6, 65, 15000, 13000, 2800, '2016-05-14'),
(309, 'Mashtoul Snack Point', 'restaurant', 'snacks', 7, 12, 7, 55, 14000, 11000, 2600, '2021-04-18'),
(310, 'Mashtoul Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 70, 15000, 13000, 3000, '2019-09-02'),

(311, 'Zagazig Road Grocery', 'grocery', 'food_items', 22, 14, 7, 170, 19000, 33000, 6200, '2012-02-13'),
(312, 'Zagazig Road Grill House', 'restaurant', 'grilled_food', 24, 15, 7, 145, 18000, 27000, 6000, '2011-10-07'),
(313, 'Zagazig Road Electronics', 'retail', 'electronics', 14, 11, 6, 85, 16000, 14000, 3200, '2017-08-16'),
(314, 'Zagazig Road Repair Hub', 'service', 'electronics_repair', 18, 10, 6, 60, 15000, 13000, 2800, '2015-12-05'),
(315, 'Zagazig Road Coffee Corner', 'cafe', 'coffee_beverages', 8, 12, 7, 70, 15000, 12000, 2900, '2020-10-14'),

(281, 'Qaha Station Grocery', 'grocery', 'food_items', 20, 14, 7, 165, 18000, 30000, 5800, '2013-03-22'),
(282, 'Qaha Market Grill', 'restaurant', 'grilled_food', 23, 15, 7, 150, 18000, 27000, 6000, '2012-05-11'),
(283, 'Qaha Electronics Repair', 'service', 'electronics_repair', 16, 10, 6, 60, 15000, 13000, 2800, '2016-09-01'),
(284, 'Qaha Coffee Spot', 'cafe', 'coffee_beverages', 7, 12, 7, 65, 14000, 12000, 2800, '2021-02-19'),
(285, 'Qaha Community Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 60, 15000, 13000, 3000, '2018-04-07'),

(286, 'Tukh Central Grocery', 'grocery', 'food_items', 21, 14, 7, 170, 19000, 33000, 6200, '2012-01-05'),
(287, 'Tukh Market Seafood', 'retail', 'seafood', 26, 13, 7, 100, 17000, 19000, 3800, '2010-08-12'),
(288, 'Tukh Electronics Store', 'retail', 'electronics', 15, 11, 6, 90, 16000, 15000, 3300, '2017-11-21'),
(289, 'Tukh Repair Center', 'service', 'general_repair', 19, 10, 6, 70, 15000, 14000, 2900, '2015-05-03'),
(290, 'Tukh Coffee House', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 13000, 3000, '2019-07-14'),

(291, 'Shebin El Qanater Grocery', 'grocery', 'food_items', 22, 14, 7, 175, 19000, 34000, 6400, '2012-03-18'),
(292, 'Shebin El Qanater Grill', 'restaurant', 'grilled_food', 24, 15, 7, 155, 18000, 28000, 6100, '2011-12-09'),
(293, 'Shebin El Qanater Hardware', 'retail', 'hardware_tools', 28, 11, 6, 120, 16000, 20000, 3800, '2010-06-26'),
(294, 'Shebin El Qanater Phone Repair', 'service', 'mobile_repair', 17, 10, 6, 65, 15000, 14000, 2900, '2016-02-01'),
(295, 'Shebin El Qanater Cafe', 'cafe', 'coffee_beverages', 10, 12, 7, 80, 15000, 14000, 3300, '2018-09-22'),

(296, 'Abu Zaabal Industrial Grocery', 'grocery', 'food_items', 20, 14, 7, 165, 18000, 31000, 6000, '2013-04-30'),
(297, 'Abu Zaabal Market Grill', 'restaurant', 'grilled_food', 23, 15, 7, 150, 18000, 27000, 6000, '2012-07-15'),
(298, 'Abu Zaabal Electronics Repair', 'service', 'electronics_repair', 16, 10, 6, 60, 15000, 13000, 2800, '2016-11-08'),
(299, 'Abu Zaabal Coffee Stop', 'cafe', 'coffee_beverages', 8, 12, 7, 70, 15000, 12000, 2900, '2020-01-27'),
(300, 'Abu Zaabal Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 60, 15000, 13000, 3000, '2018-05-06'),

(301, 'Bilbeis Central Grocery', 'grocery', 'food_items', 22, 14, 7, 175, 19000, 34000, 6400, '2012-02-16'),
(302, 'Bilbeis Market Grill', 'restaurant', 'grilled_food', 25, 15, 7, 155, 18000, 28000, 6100, '2011-09-03'),
(303, 'Bilbeis Electronics Store', 'retail', 'electronics', 14, 11, 6, 90, 16000, 15000, 3300, '2017-06-24'),
(304, 'Bilbeis Repair Hub', 'service', 'general_repair', 18, 10, 6, 70, 15000, 14000, 2900, '2015-10-18'),
(305, 'Bilbeis Coffee House', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 13000, 3000, '2019-03-29'),

(306, 'Mashtoul Road Grocery', 'grocery', 'food_items', 21, 14, 7, 170, 19000, 33000, 6200, '2012-01-11'),
(307, 'Mashtoul Road Grill', 'restaurant', 'grilled_food', 24, 15, 7, 150, 18000, 28000, 6100, '2011-11-05'),
(308, 'Mashtoul Road Hardware', 'retail', 'hardware_tools', 29, 11, 6, 130, 16000, 21000, 3900, '2010-05-14'),
(309, 'Mashtoul Road Repair Center', 'service', 'general_repair', 19, 10, 6, 70, 15000, 14000, 2900, '2015-08-21'),
(310, 'Mashtoul Road Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 80, 15000, 14000, 3300, '2018-12-02'),

(71, 'Bulaq Central Grocery', 'grocery', 'food_items', 21, 14, 7, 160, 18500, 29500, 5700, '2014-02-12'),
(72, 'Maspero Koshary', 'restaurant', 'egyptian_food', 19, 15, 7, 130, 20000, 26000, 5600, '2015-06-21'),
(73, 'Abou El Ela Electronics', 'retail', 'electronics', 14, 11, 6, 85, 17000, 14500, 3200, '2017-03-08'),
(74, 'Bulaq Phone Repair', 'service', 'mobile_repair', 16, 10, 6, 60, 15000, 13000, 2800, '2016-10-19'),
(75, 'Corniche View Coffee', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 16500, 14500, 3300, '2019-08-04'),

(77, 'Fustat Heritage Cafe', 'cafe', 'coffee_beverages', 11, 12, 7, 70, 15000, 13000, 3000, '2018-02-15'),
(77, 'Fustat Daily Grocery', 'grocery', 'food_items', 23, 14, 7, 170, 18000, 32000, 6100, '2012-11-09'),
(78, 'Ain El Sira Snacks', 'restaurant', 'snacks', 7, 12, 7, 55, 15000, 12000, 2800, '2021-07-18'),
(79, 'Magra El Oyoun Butchery', 'retail', 'meat_products', 27, 12, 7, 90, 16000, 18500, 3700, '2011-04-22'),
(80, 'Old Cairo Spice Corner', 'retail', 'spices', 29, 11, 6, 75, 14500, 16500, 2900, '2010-09-01'),

(81, 'New Cairo Express Cafe', 'cafe', 'coffee_beverages', 6, 12, 7, 65, 23000, 15000, 3500, '2021-05-14'),
(82, 'North 90th Street Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 70, 25000, 16000, 3800, '2018-01-19'),
(83, 'South 90th Mini Market', 'grocery', 'food_items', 14, 13, 7, 140, 22000, 26000, 5200, '2019-10-07'),
(84, 'Downtown Mall Food Court', 'restaurant', 'fast_food', 8, 14, 7, 110, 28000, 24000, 5600, '2020-09-25'),
(85, 'AUC Gate Coffee', 'cafe', 'coffee_beverages', 5, 11, 7, 55, 21000, 12000, 3000, '2022-03-30'),

(86, 'Maadi Corniche Restaurant', 'restaurant', 'international_food', 15, 13, 7, 150, 26000, 28000, 6000, '2016-05-11'),
(87, 'Road 9 Grocery', 'grocery', 'food_items', 18, 14, 7, 165, 21000, 30000, 5800, '2014-07-03'),
(88, 'Degla Electronics', 'retail', 'electronics', 13, 11, 6, 85, 20000, 15000, 3400, '2017-09-22'),
(89, 'Zahraa El Maadi Pharmacy', 'pharmacy', 'medicines', 11, 12, 7, 65, 18000, 14000, 3200, '2018-11-02'),
(90, 'Maadi Market Bakery', 'bakery', 'bread_pastries', 20, 12, 6, 80, 19000, 17000, 3600, '2015-01-28'),

(91, 'Warraq Main Grocery', 'grocery', 'food_items', 22, 14, 7, 170, 17500, 32000, 6000, '2013-03-17'),
(92, 'Warraq Island Grill', 'restaurant', 'grilled_food', 24, 15, 7, 150, 18000, 27000, 6100, '2012-10-09'),
(93, 'Warraq Electronics Store', 'retail', 'electronics', 15, 11, 6, 90, 16500, 15000, 3300, '2017-02-21'),
(94, 'Warraq Mobile Fix', 'service', 'mobile_repair', 18, 10, 6, 60, 14500, 13000, 2800, '2015-06-12'),
(95, 'Warraq Coffee Point', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15500, 14000, 3300, '2019-12-01'),

(96, 'Bab El Shaaria Central Market', 'grocery', 'food_items', 25, 14, 7, 180, 19000, 34000, 6400, '2011-08-14'),
(97, 'Bab El Shaaria Grill', 'restaurant', 'grilled_food', 26, 15, 7, 160, 18500, 29000, 6300, '2010-05-03'),
(98, 'Port Said Street Phones', 'retail', 'mobile_phones', 14, 11, 6, 85, 16500, 14000, 3100, '2018-04-27'),
(99, 'Sabtiya Auto Supplies', 'retail', 'auto_parts', 28, 11, 6, 125, 17000, 19500, 3700, '2009-09-18'),
(100,'Railway Snack Corner', 'restaurant', 'snacks', 7, 12, 7, 55, 15000, 12000, 2700, '2022-02-10'),

(101, 'El Salam Mega Grocery', 'grocery', 'food_items', 23, 14, 7, 180, 18500, 33000, 6200, '2012-06-24'),
(102, 'El Salam Grill House', 'restaurant', 'grilled_food', 25, 15, 7, 155, 18000, 28000, 6100, '2011-11-15'),
(103, 'El Salam Industrial Hardware', 'retail', 'industrial_tools', 20, 11, 6, 140, 16500, 22000, 4000, '2016-03-05'),
(104, 'El Salam Phone Repair', 'service', 'mobile_repair', 17, 10, 6, 65, 15000, 14000, 2900, '2015-08-27'),
(105, 'Ring Road Coffee Stop', 'cafe', 'coffee_beverages', 6, 12, 7, 60, 15500, 12000, 2900, '2021-04-08'),

(306, 'Mashtoul Central Grocery', 'grocery', 'food_items', 21, 14, 7, 175, 18000, 33000, 6200, '2012-01-29'),
(307, 'Mashtoul Grill & BBQ', 'restaurant', 'grilled_food', 24, 15, 7, 155, 17500, 28000, 6100, '2011-07-19'),
(308, 'Mashtoul Electronics', 'retail', 'electronics', 15, 11, 6, 95, 16500, 15500, 3400, '2017-12-11'),
(309, 'Mashtoul Repair Hub', 'service', 'general_repair', 19, 10, 6, 70, 15000, 14000, 2900, '2015-03-02'),
(310, 'Mashtoul Coffee Corner', 'cafe', 'coffee_beverages', 10, 12, 7, 80, 15000, 14000, 3300, '2019-09-18'),

(311, 'Zagazig Road Grocery Plus', 'grocery', 'food_items', 23, 14, 7, 180, 19000, 34000, 6400, '2011-10-14'),
(312, 'Zagazig Road Grill', 'restaurant', 'grilled_food', 26, 15, 7, 160, 18500, 29000, 6300, '2010-06-21'),
(313, 'Zagazig Road Electronics Shop', 'retail', 'electronics', 15, 11, 6, 95, 16500, 15500, 3400, '2018-03-17'),
(314, 'Zagazig Road Repair Services', 'service', 'electronics_repair', 18, 10, 6, 65, 15000, 14000, 2900, '2016-09-09'),
(315, 'Zagazig Road Coffee Point', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 14000, 3300, '2020-12-27'),

(281, 'Qaha Central Grocery', 'grocery', 'food_items', 22, 14, 7, 175, 18500, 33000, 6200, '2012-04-06'),
(282, 'Qaha Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 155, 18000, 28000, 6100, '2011-01-25'),
(283, 'Qaha Electronics Store', 'retail', 'electronics', 14, 11, 6, 90, 16500, 15000, 3300, '2017-07-14'),
(284, 'Qaha Repair Center', 'service', 'general_repair', 19, 10, 6, 70, 15000, 14000, 2900, '2015-11-03'),
(285, 'Qaha Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 14000, 3300, '2019-06-19'),

(286, 'Tukh Mega Grocery', 'grocery', 'food_items', 23, 14, 7, 180, 19000, 34000, 6400, '2011-12-08'),
(287, 'Tukh Grill House', 'restaurant', 'grilled_food', 26, 15, 7, 160, 18500, 29000, 6300, '2010-08-23'),
(288, 'Tukh Electronics Hub', 'retail', 'electronics', 15, 11, 6, 95, 16500, 15500, 3400, '2018-02-05'),
(289, 'Tukh Repair Workshop', 'service', 'general_repair', 18, 10, 6, 70, 15000, 14000, 2900, '2016-06-14'),
(290, 'Tukh Coffee Lounge', 'cafe', 'coffee_beverages', 10, 12, 7, 80, 15000, 14000, 3300, '2019-10-31'),

(291, 'Shebin El Qanater Central Grocery', 'grocery', 'food_items', 24, 14, 7, 185, 19500, 35000, 6600, '2011-02-11'),
(292, 'Shebin El Qanater Grill', 'restaurant', 'grilled_food', 27, 15, 7, 165, 19000, 30000, 6500, '2010-04-29'),
(293, 'Shebin El Qanater Hardware', 'retail', 'hardware_tools', 29, 11, 6, 135, 17000, 21000, 3900, '2009-07-08'),
(294, 'Shebin El Qanater Repair Shop', 'service', 'general_repair', 19, 10, 6, 75, 15500, 14500, 3000, '2016-12-02'),
(295, 'Shebin El Qanater Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 85, 15500, 14500, 3400, '2018-08-15'),

(296, 'Abu Zaabal Grocery', 'grocery', 'food_items', 22, 14, 7, 175, 18500, 33000, 6200, '2012-05-20'),
(297, 'Abu Zaabal Grill', 'restaurant', 'grilled_food', 25, 15, 7, 160, 18000, 28000, 6100, '2011-03-11'),
(298, 'Abu Zaabal Electronics', 'retail', 'electronics', 14, 11, 6, 95, 16500, 15500, 3400, '2017-10-04'),
(299, 'Abu Zaabal Repair Center', 'service', 'electronics_repair', 18, 10, 6, 70, 15000, 14000, 2900, '2016-02-18'),
(300, 'Abu Zaabal Coffee Corner', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 14000, 3300, '2020-01-09'),

(301, 'Bilbeis Central Grocery', 'grocery', 'food_items', 23, 14, 7, 180, 19500, 35000, 6600, '2011-01-27'),
(302, 'Bilbeis Grill Station', 'restaurant', 'grilled_food', 26, 15, 7, 165, 19000, 30000, 6500, '2010-06-03'),
(303, 'Bilbeis Electronics Hub', 'retail', 'electronics', 15, 11, 6, 100, 17000, 16000, 3500, '2018-05-12'),
(304, 'Bilbeis Repair Services', 'service', 'general_repair', 19, 10, 6, 75, 15500, 14500, 3000, '2016-09-28'),
(305, 'Bilbeis Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 15500, 14500, 3400, '2019-11-21'),

(306, 'Mashtoul Road Grocery Plus', 'grocery', 'food_items', 22, 14, 7, 175, 19000, 34000, 6400, '2011-04-13'),
(307, 'Mashtoul Road Grill', 'restaurant', 'grilled_food', 25, 15, 7, 160, 18500, 29000, 6300, '2010-10-19'),
(308, 'Mashtoul Road Electronics', 'retail', 'electronics', 14, 11, 6, 95, 16500, 15500, 3400, '2017-07-07'),
(309, 'Mashtoul Road Repair Hub', 'service', 'general_repair', 18, 10, 6, 70, 15000, 14000, 2900, '2016-03-26'),
(310, 'Mashtoul Road Coffee Corner', 'cafe', 'coffee_beverages', 9, 12, 7, 75, 15000, 14000, 3300, '2019-08-09'),

(281, 'Qaha Market Grocery Plus', 'grocery', 'food_items', 23, 14, 7, 180, 19500, 35000, 6600, '2011-02-05'),
(282, 'Qaha Market Grill', 'restaurant', 'grilled_food', 26, 15, 7, 165, 19000, 30000, 6500, '2010-09-12'),
(283, 'Qaha Electronics Center', 'retail', 'electronics', 15, 11, 6, 100, 17000, 16000, 3500, '2018-01-28'),
(284, 'Qaha Repair Workshop', 'service', 'general_repair', 19, 10, 6, 75, 15500, 14500, 3000, '2016-07-19'),
(285, 'Qaha Coffee Lounge', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 15500, 14500, 3400, '2019-05-16'),

(286, 'Tukh Central Grocery Plus', 'grocery', 'food_items', 23, 14, 7, 185, 19500, 35000, 6600, '2011-03-02'),
(287, 'Tukh Grill & BBQ', 'restaurant', 'grilled_food', 27, 15, 7, 165, 19000, 30000, 6500, '2010-11-21'),
(288, 'Tukh Electronics World', 'retail', 'electronics', 16, 11, 6, 105, 17500, 16500, 3600, '2018-06-10'),
(289, 'Tukh Repair Services', 'service', 'general_repair', 20, 10, 6, 80, 16000, 15000, 3100, '2016-12-14'),
(290, 'Tukh Coffee Point', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 16000, 15000, 3500, '2019-10-02'),

(291, 'Shebin Central Grocery Plus', 'grocery', 'food_items', 24, 14, 7, 190, 20000, 36000, 6800, '2011-01-18'),
(292, 'Shebin Grill House', 'restaurant', 'grilled_food', 28, 15, 7, 170, 19500, 31000, 6700, '2010-08-07'),
(293, 'Shebin Electronics Center', 'retail', 'electronics', 16, 11, 6, 110, 18000, 17000, 3700, '2018-09-01'),
(294, 'Shebin Repair Workshop', 'service', 'general_repair', 21, 10, 6, 85, 16500, 15500, 3200, '2016-05-22'),
(295, 'Shebin Coffee Stop', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 16500, 15500, 3600, '2019-12-12'),

(296, 'Abu Zaabal Central Grocery Plus', 'grocery', 'food_items', 23, 14, 7, 185, 19500, 35000, 6600, '2011-06-09'),
(297, 'Abu Zaabal Grill Corner', 'restaurant', 'grilled_food', 27, 15, 7, 165, 19000, 30000, 6500, '2010-04-15'),
(298, 'Abu Zaabal Electronics World', 'retail', 'electronics', 16, 11, 6, 105, 17500, 16500, 3600, '2018-02-22'),
(299, 'Abu Zaabal Repair Services', 'service', 'electronics_repair', 20, 10, 6, 80, 16000, 15000, 3100, '2016-11-06'),
(300, 'Abu Zaabal Coffee Lounge', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 16000, 15000, 3500, '2019-07-27'),

(301, 'Bilbeis Central Grocery Plus', 'grocery', 'food_items', 24, 14, 7, 190, 20000, 36000, 6800, '2011-02-24'),
(302, 'Bilbeis Grill House', 'restaurant', 'grilled_food', 28, 15, 7, 170, 19500, 31000, 6700, '2010-09-09'),
(303, 'Bilbeis Electronics Center', 'retail', 'electronics', 16, 11, 6, 110, 18000, 17000, 3700, '2018-12-03'),
(304, 'Bilbeis Repair Workshop', 'service', 'general_repair', 21, 10, 6, 85, 16500, 15500, 3200, '2016-08-17'),
(305, 'Bilbeis Coffee Stop', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 16500, 15500, 3600, '2019-11-30'),

-- ('El Hussein Heritage Cafe', 'cafe', 'coffee_beverages', 12, 12, 7, 70, 17000, 14000, 3200, '2018-03-09'),
-- ('Khan El Khalili Gifts', 'retail', 'souvenirs', 22, 10, 6, 95, 18000, 16000, 3400, '2012-06-21'),
-- ('Azhar Street Books', 'retail', 'books', 18, 9, 6, 75, 16000, 12000, 2600, '2014-11-15'),
-- ('Gamaleya Market Grocery', 'grocery', 'food_items', 24, 14, 7, 180, 19000, 34000, 6400, '2011-04-07'),
-- ('Azhar Park Juice Bar', 'cafe', 'fresh_juice', 7, 11, 7, 55, 15000, 10000, 2400, '2021-08-12'),

-- ('Darb El Ahmar Central Bakery', 'bakery', 'bread_pastries', 26, 13, 7, 85, 15500, 18000, 3600, '2010-09-18'),
-- ('Darb El Ahmar Grocery', 'grocery', 'food_items', 23, 14, 7, 170, 17500, 32000, 6100, '2012-01-26'),
-- ('Darb El Ahmar Grill', 'restaurant', 'grilled_food', 25, 15, 7, 155, 18000, 29000, 6300, '2011-05-03'),
-- ('Citadel Road Coffee', 'cafe', 'coffee_beverages', 9, 12, 7, 70, 16000, 14000, 3300, '2019-10-27'),
-- ('Historic Zone Handicrafts', 'retail', 'handicrafts', 28, 10, 6, 100, 17000, 18000, 3500, '2009-12-14'),

(227, 'Shubra El Kheima Mega Grocery', 'grocery', 'food_items', 25, 14, 7, 200, 20000, 38000, 7000, '2011-02-03'),
(227, 'Shubra El Kheima Grill House', 'restaurant', 'grilled_food', 27, 15, 7, 175, 19500, 32000, 6700, '2010-07-19'),
(227, 'Shubra El Kheima Electronics', 'retail', 'electronics', 16, 11, 6, 120, 18500, 17500, 3800, '2018-04-08'),
(227, 'Shubra El Kheima Phone Repair', 'service', 'mobile_repair', 20, 10, 6, 80, 16500, 15500, 3200, '2015-11-02'),
(227, 'Shubra El Kheima Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 95, 17000, 15500, 3600, '2019-06-25'),

-- ('El Marg Central Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 19500, 36000, 6800, '2011-01-17'),
-- ('El Marg Market Grill', 'restaurant', 'grilled_food', 26, 15, 7, 170, 19000, 31000, 6600, '2010-04-11'),
-- ('El Marg Industrial Supplies', 'retail', 'industrial_tools', 22, 11, 6, 140, 18000, 22000, 4000, '2016-09-30'),
-- ('El Marg Repair Workshop', 'service', 'general_repair', 19, 10, 6, 85, 16500, 15500, 3200, '2015-03-18'),
-- ('El Marg Coffee Point', 'cafe', 'coffee_beverages', 10, 12, 7, 90, 16500, 15000, 3500, '2019-08-09'),

-- ('Kerdasa Crafts Market', 'retail', 'handicrafts', 27, 10, 6, 120, 17500, 19000, 3600, '2009-06-05'),
-- ('Kerdasa Central Grocery', 'grocery', 'food_items', 22, 14, 7, 175, 17000, 31000, 6000, '2013-02-14'),
-- ('Kerdasa Grill Corner', 'restaurant', 'grilled_food', 24, 15, 7, 160, 17500, 28000, 6200, '2011-10-22'),
-- ('Kerdasa Electronics', 'retail', 'electronics', 15, 11, 6, 95, 16000, 15000, 3300, '2017-05-29'),
-- ('Kerdasa Coffee House', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 15500, 14000, 3300, '2019-01-13'),

(267, 'Badrashin Central Grocery', 'grocery', 'food_items', 23, 14, 7, 180, 18000, 33000, 6200, '2012-04-01'),
(268, 'Badrashin Market Grill', 'restaurant', 'grilled_food', 26, 15, 7, 165, 18500, 30000, 6500, '2010-12-07'),
(269, 'Badrashin Industrial Supplies', 'retail', 'industrial_tools', 21, 11, 6, 135, 17000, 21000, 3900, '2016-06-18'),
(270, 'Badrashin Repair Hub', 'service', 'general_repair', 18, 10, 6, 80, 15500, 14500, 3000, '2015-09-26'),
(271, 'Badrashin Coffee Corner', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 15500, 14500, 3400, '2019-11-06'),

-- ('El Obour Industrial Grocery', 'grocery', 'food_items', 20, 14, 7, 160, 21000, 30000, 5800, '2014-03-20'),
-- ('El Obour Grill Station', 'restaurant', 'grilled_food', 22, 15, 7, 145, 22000, 27000, 6000, '2013-07-11'),
-- ('El Obour Electronics Hub', 'retail', 'electronics', 14, 11, 6, 110, 23000, 17000, 3800, '2018-08-23'),
-- ('El Obour Repair Center', 'service', 'general_repair', 17, 10, 6, 75, 20000, 15000, 3200, '2016-05-17'),
-- ('El Obour Coffee Stop', 'cafe', 'coffee_beverages', 8, 12, 7, 80, 21500, 14500, 3500, '2020-01-28'),

-- ('El Obour Commercial Grocery', 'grocery', 'food_items', 21, 14, 7, 170, 22000, 31000, 6000, '2013-06-04'),
-- ('El Obour BBQ Grill', 'restaurant', 'grilled_food', 24, 15, 7, 155, 22500, 29000, 6300, '2011-11-19'),
-- ('El Obour Hardware Tools', 'retail', 'hardware_tools', 26, 11, 6, 130, 21000, 22000, 4000, '2010-03-08'),
-- ('El Obour Phone Repair', 'service', 'mobile_repair', 19, 10, 6, 85, 19500, 15500, 3300, '2015-02-14'),
-- ('El Obour Coffee Lounge', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 21500, 15000, 3600, '2019-09-03'),

(143, 'Shorouk Central Grocery', 'grocery', 'food_items', 18, 14, 7, 150, 20000, 27000, 5400, '2015-05-26'),
(144, 'Shorouk Grill House', 'restaurant', 'grilled_food', 21, 15, 7, 135, 21000, 26000, 5800, '2014-08-17'),
(145, 'Shorouk Electronics', 'retail', 'electronics', 12, 11, 6, 100, 22000, 16000, 3500, '2019-12-05'),
(146, 'Shorouk Repair Services', 'service', 'general_repair', 16, 10, 6, 70, 19000, 14000, 3000, '2016-04-10'),
(147, 'Shorouk Coffee Stop', 'cafe', 'coffee_beverages', 7, 12, 7, 75, 20500, 14000, 3300, '2021-02-22'),

(148, 'Badr City Central Grocery', 'grocery', 'food_items', 17, 14, 7, 145, 19500, 26000, 5200, '2016-01-29'),
(149, 'Badr City Grill Station', 'restaurant', 'grilled_food', 20, 15, 7, 130, 20500, 25000, 5600, '2015-06-07'),
(150, 'Badr City Electronics Hub', 'retail', 'electronics', 11, 11, 6, 95, 21500, 15000, 3300, '2020-03-15'),
(151, 'Badr City Repair Center', 'service', 'general_repair', 15, 10, 6, 65, 18500, 13500, 2900, '2017-09-12'),
(152, 'Badr City Coffee Corner', 'cafe', 'coffee_beverages', 6, 12, 7, 70, 20000, 13500, 3200, '2022-06-01'),

(153, 'Madinaty Central Grocery', 'grocery', 'food_items', 14, 14, 7, 140, 24000, 25000, 5000, '2018-07-18'),
(154, 'Madinaty Grill & BBQ', 'restaurant', 'grilled_food', 17, 15, 7, 125, 26000, 24000, 5400, '2017-11-03'),
(155, 'Madinaty Electronics Store', 'retail', 'electronics', 10, 11, 6, 90, 27000, 16000, 3500, '2021-04-09'),
(156, 'Madinaty Repair Hub', 'service', 'general_repair', 14, 10, 6, 60, 23000, 14000, 3000, '2018-05-27'),
(157, 'Madinaty Coffee Lounge', 'cafe', 'coffee_beverages', 6, 12, 7, 75, 25000, 14500, 3300, '2022-01-19'),

(163, 'Rehab Central Grocery', 'grocery', 'food_items', 15, 14, 7, 145, 23000, 26000, 5200, '2017-04-22'),
(164, 'Rehab Grill Station', 'restaurant', 'grilled_food', 18, 15, 7, 130, 24000, 25000, 5600, '2016-09-14'),
(165, 'Rehab Electronics Hub', 'retail', 'electronics', 11, 11, 6, 95, 25500, 16500, 3600, '2020-08-06'),
(166, 'Rehab Repair Center', 'service', 'general_repair', 15, 10, 6, 65, 22500, 14500, 3100, '2018-12-02'),
(167, 'Rehab Coffee Stop', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 24000, 15000, 3400, '2021-10-28'),

(168, 'Nozha Central Grocery', 'grocery', 'food_items', 19, 14, 7, 165, 18500, 30000, 5800, '2014-06-09'),
(169, 'Nozha Grill House', 'restaurant', 'grilled_food', 22, 15, 7, 150, 19000, 28000, 6100, '2013-10-17'),
(170, 'Nozha Electronics', 'retail', 'electronics', 13, 11, 6, 105, 20000, 16500, 3600, '2018-02-26'),
(171, 'Nozha Repair Services', 'service', 'general_repair', 17, 10, 6, 75, 17500, 14500, 3000, '2016-08-05'),
(172, 'Nozha Coffee Point', 'cafe', 'coffee_beverages', 8, 12, 7, 80, 18500, 15000, 3300, '2020-11-11'),

(350, 'Ring Road Grocery Hub', 'grocery', 'food_items', 20, 14, 7, 170, 18000, 31000, 6000, '2013-03-28'),
(350, 'Ring Road Grill Corner', 'restaurant', 'grilled_food', 23, 15, 7, 155, 18500, 29000, 6300, '2012-07-13'),
(350, 'Ring Road Electronics Shop', 'retail', 'electronics', 14, 11, 6, 110, 19500, 17000, 3700, '2018-01-09'),
(350, 'Ring Road Repair Hub', 'service', 'general_repair', 18, 10, 6, 80, 17000, 15000, 3100, '2016-10-20'),
(350, 'Ring Road Coffee Lounge', 'cafe', 'coffee_beverages', 9, 12, 7, 85, 17500, 15000, 3400, '2019-05-02'),

(192, 'Fayoum Road Grocery', 'grocery', 'food_items', 21, 14, 7, 175, 17000, 32000, 6100, '2012-01-15'),
(193, 'Fayoum Road Grill', 'restaurant', 'grilled_food', 24, 15, 7, 160, 17500, 29000, 6300, '2011-09-04'),
(194, 'Fayoum Road Electronics', 'retail', 'electronics', 14, 11, 6, 105, 18500, 16500, 3600, '2017-12-29'),
(195, 'Fayoum Road Repair Center', 'service', 'general_repair', 18, 10, 6, 75, 16000, 14500, 3000, '2016-06-16'),
(196, 'Fayoum Road Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16500, 14500, 3300, '2019-09-20'),

(197, 'Maryotia Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17500, 33000, 6200, '2011-05-12'),
(198, 'Maryotia Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 165, 18000, 30000, 6500, '2010-10-01'),
(199, 'Maryotia Electronics Hub', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-04-18'),
(200, 'Maryotia Repair Services', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-11-29'),
(201, 'Maryotia Coffee Corner', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-03-08'),

(207, 'Abu Rawash Grocery Hub', 'grocery', 'food_items', 21, 14, 7, 175, 17000, 32000, 6100, '2012-02-20'),
(208, 'Abu Rawash Grill House', 'restaurant', 'grilled_food', 24, 15, 7, 160, 17500, 29000, 6300, '2011-08-15'),
(209, 'Abu Rawash Electronics', 'retail', 'electronics', 14, 11, 6, 105, 18500, 16500, 3600, '2017-10-09'),
(210, 'Abu Rawash Repair Hub', 'service', 'general_repair', 18, 10, 6, 75, 16000, 14500, 3000, '2016-07-24'),
(211, 'Abu Rawash Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16500, 14500, 3300, '2019-12-18'),

(212, 'Oseem Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17500, 33000, 6200, '2011-04-02'),
(213, 'Oseem Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 165, 18000, 30000, 6500, '2010-11-26'),
(214, 'Oseem Electronics Center', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-06-13'),
(215, 'Oseem Repair Services', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-12-08'),
(216, 'Oseem Coffee Lounge', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-10-25'),

(222, 'Qalyub Central Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 18000, 34000, 6400, '2011-01-23'),
(223, 'Qalyub Grill Corner', 'restaurant', 'grilled_food', 26, 15, 7, 170, 18500, 31000, 6700, '2010-06-12'),
(224, 'Qalyub Electronics Hub', 'retail', 'electronics', 15, 11, 6, 115, 19500, 17500, 3800, '2018-03-27'),
(225, 'Qalyub Repair Workshop', 'service', 'general_repair', 20, 10, 6, 85, 17000, 15500, 3200, '2016-09-16'),
(226, 'Qalyub Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 17500, 15500, 3500, '2019-07-04'),

(227, 'Ezbet El Nakhl Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 18000, 35000, 6600, '2011-02-28'),
(228, 'Ezbet El Nakhl Grill', 'restaurant', 'grilled_food', 27, 15, 7, 175, 18500, 32000, 6900, '2010-05-18'),
(229, 'Ezbet El Nakhl Electronics', 'retail', 'electronics', 16, 11, 6, 120, 20000, 18000, 3900, '2018-08-02'),
(230, 'Ezbet El Nakhl Repair Services', 'service', 'general_repair', 21, 10, 6, 90, 17500, 16000, 3300, '2016-03-11'),
(231, 'Ezbet El Nakhl Coffee Corner', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 18000, 16000, 3600, '2019-11-19'),

(183, 'Sheikh Zayed Central Grocery', 'grocery', 'food_items', 16, 14, 7, 150, 26000, 27000, 5200, '2017-04-18'),
(184, 'Sheikh Zayed Grill House', 'restaurant', 'grilled_food', 18, 15, 7, 135, 28000, 26000, 5600, '2016-09-02'),
(185, 'Sheikh Zayed Electronics', 'retail', 'electronics', 11, 11, 6, 100, 30000, 18000, 3800, '2020-02-11'),
(186, 'Sheikh Zayed Repair Hub', 'service', 'general_repair', 14, 10, 6, 70, 24000, 14000, 3000, '2018-06-07'),
(187, 'Sheikh Zayed Coffee Lounge', 'cafe', 'coffee_beverages', 7, 12, 7, 80, 29000, 15000, 3300, '2021-03-26'),

(178, '6th October Mega Grocery', 'grocery', 'food_items', 20, 14, 7, 180, 23000, 33000, 6100, '2014-02-15'),
(179, '6th October BBQ Grill', 'restaurant', 'grilled_food', 23, 15, 7, 165, 25000, 30000, 6500, '2013-07-09'),
(180, '6th October Electronics World', 'retail', 'electronics', 14, 11, 6, 115, 27000, 19000, 3900, '2018-05-22'),
(181, '6th October Repair Center', 'service', 'general_repair', 18, 10, 6, 85, 23000, 15500, 3200, '2016-11-13'),
(182, '6th October Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 90, 26000, 15000, 3400, '2019-08-01'),

(173, 'Haram Main Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 18000, 35000, 6600, '2012-03-05'),
(173, 'Haram Street Grill', 'restaurant', 'grilled_food', 26, 15, 7, 175, 18500, 32000, 6900, '2010-06-21'),
(173, 'Haram Electronics Center', 'retail', 'electronics', 16, 11, 6, 120, 20000, 18000, 3900, '2018-09-14'),
(173, 'Haram Repair Services', 'service', 'general_repair', 21, 10, 6, 90, 17500, 16000, 3300, '2016-02-08'),
(173, 'Haram Coffee Corner', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 18000, 16000, 3600, '2019-10-30'),

(174, 'Faisal Central Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 17500, 34000, 6400, '2011-01-27'),
(174, 'Faisal Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 170, 18000, 31000, 6700, '2010-04-19'),
(174, 'Faisal Electronics Hub', 'retail', 'electronics', 15, 11, 6, 115, 19500, 17500, 3800, '2018-06-03'),
(174, 'Faisal Repair Workshop', 'service', 'general_repair', 19, 10, 6, 85, 17000, 15500, 3200, '2016-12-14'),
(174, 'Faisal Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 90, 17500, 15500, 3500, '2019-09-17'),

(12, 'Dokki Central Grocery', 'grocery', 'food_items', 18, 14, 7, 160, 22000, 29000, 5600, '2015-05-06'),
(13, 'Dokki Grill House', 'restaurant', 'grilled_food', 21, 15, 7, 145, 24000, 27000, 5900, '2014-08-28'),
(14, 'Dokki Electronics', 'retail', 'electronics', 13, 11, 6, 105, 26000, 18000, 3800, '2019-01-11'),
(15, 'Dokki Repair Center', 'service', 'general_repair', 16, 10, 6, 75, 21000, 14500, 3100, '2017-07-03'),
(16, 'Dokki Coffee Lounge', 'cafe', 'coffee_beverages', 8, 12, 7, 85, 25000, 15000, 3400, '2021-02-14'),

(57, 'Agouza Central Grocery', 'grocery', 'food_items', 19, 14, 7, 165, 21000, 30000, 5800, '2014-03-18'),
(58, 'Agouza Grill Station', 'restaurant', 'grilled_food', 22, 15, 7, 150, 23000, 28000, 6100, '2013-10-07'),
(59, 'Agouza Electronics Center', 'retail', 'electronics', 14, 11, 6, 110, 24500, 17000, 3700, '2018-04-21'),
(60, 'Agouza Repair Hub', 'service', 'general_repair', 17, 10, 6, 80, 20500, 15000, 3200, '2016-09-12'),
(61, 'Agouza Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 90, 23500, 15000, 3500, '2019-12-05'),

(62, 'Imbaba Central Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 17000, 35000, 6600, '2011-02-09'),
(63, 'Imbaba Grill House', 'restaurant', 'grilled_food', 26, 15, 7, 175, 17500, 32000, 6900, '2010-07-18'),
(64, 'Imbaba Electronics', 'retail', 'electronics', 16, 11, 6, 120, 19000, 18000, 3900, '2018-03-14'),
(65, 'Imbaba Repair Services', 'service', 'general_repair', 21, 10, 6, 90, 16500, 16000, 3300, '2016-01-25'),
(66, 'Imbaba Coffee Corner', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 17000, 16000, 3600, '2019-11-21'),

(72, 'Bulaq Central Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 17500, 34000, 6400, '2011-05-16'),
(73, 'Bulaq Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 170, 18000, 31000, 6700, '2010-09-03'),
(74, 'Bulaq Electronics Hub', 'retail', 'electronics', 15, 11, 6, 115, 19500, 17500, 3800, '2018-07-28'),
(75, 'Bulaq Repair Workshop', 'service', 'general_repair', 19, 10, 6, 85, 17000, 15500, 3200, '2016-04-11'),
(76, 'Bulaq Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 90, 17500, 15500, 3500, '2019-08-23'),

(52, 'Helwan Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17000, 33000, 6200, '2012-01-06'),
(53, 'Helwan Grill House', 'restaurant', 'grilled_food', 25, 15, 7, 165, 17500, 30000, 6500, '2011-11-09'),
(54, 'Helwan Electronics', 'retail', 'electronics', 15, 11, 6, 110, 18500, 17000, 3700, '2018-05-02'),
(55, 'Helwan Repair Center', 'service', 'general_repair', 19, 10, 6, 80, 16000, 15000, 3100, '2016-08-19'),
(56, 'Helwan Coffee Lounge', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 16500, 15000, 3400, '2019-06-27'),

(87, 'Maadi Central Grocery', 'grocery', 'food_items', 17, 14, 7, 150, 26000, 27000, 5200, '2016-03-24'),
(88, 'Maadi Grill Station', 'restaurant', 'grilled_food', 19, 15, 7, 135, 28000, 26000, 5600, '2015-07-13'),
(89, 'Maadi Electronics Center', 'retail', 'electronics', 12, 11, 6, 100, 30000, 18000, 3800, '2020-01-08'),
(90, 'Maadi Repair Hub', 'service', 'general_repair', 15, 10, 6, 70, 24000, 14000, 3000, '2018-10-16'),
(91, 'Maadi Coffee Corner', 'cafe', 'coffee_beverages', 7, 12, 7, 80, 29000, 15000, 3300, '2021-05-29'),

-- ('Basateen Central Grocery', 'grocery', 'food_items', 21, 14, 7, 175, 18000, 32000, 6100, '2012-06-02'),
-- ('Basateen Grill House', 'restaurant', 'grilled_food', 24, 15, 7, 160, 18500, 29000, 6300, '2011-12-19'),
-- ('Basateen Electronics', 'retail', 'electronics', 14, 11, 6, 105, 19500, 16500, 3600, '2018-08-24'),
-- ('Basateen Repair Services', 'service', 'general_repair', 18, 10, 6, 75, 17000, 14500, 3000, '2016-04-06'),
-- ('Basateen Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 17500, 14500, 3300, '2019-09-14'),

-- ('Dar El Salam Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17500, 33000, 6200, '2012-02-17'),
-- ('Dar El Salam Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 165, 18000, 30000, 6500, '2011-06-21'),
-- ('Dar El Salam Electronics', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-01-12'),
-- ('Dar El Salam Repair Hub', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-10-03'),
-- ('Dar El Salam Coffee Lounge', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-04-25'),

-- ('Mokattam Central Grocery', 'grocery', 'food_items', 18, 14, 7, 155, 24000, 26000, 5200, '2016-01-09'),
-- ('Mokattam Grill House', 'restaurant', 'grilled_food', 20, 15, 7, 140, 26000, 25000, 5600, '2015-09-28'),
-- ('Mokattam Electronics Hub', 'retail', 'electronics', 12, 11, 6, 105, 28000, 17000, 3700, '2020-03-17'),
-- ('Mokattam Repair Services', 'service', 'general_repair', 15, 10, 6, 75, 23000, 14500, 3100, '2018-07-08'),
-- ('Mokattam Coffee Stop', 'cafe', 'coffee_beverages', 7, 12, 7, 85, 27000, 15000, 3400, '2021-11-04'),

(142, 'Ring Road Market Grocery', 'grocery', 'food_items', 20, 14, 7, 170, 18000, 31000, 6000, '2013-05-21'),
(142, 'Ring Road Grill Corner', 'restaurant', 'grilled_food', 23, 15, 7, 155, 18500, 29000, 6300, '2012-08-09'),
(142, 'Ring Road Electronics', 'retail', 'electronics', 14, 11, 6, 110, 19500, 17000, 3700, '2018-11-13'),
(142, 'Ring Road Repair Services', 'service', 'general_repair', 18, 10, 6, 80, 17000, 15000, 3100, '2016-06-27'),
(142, 'Ring Road Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 85, 17500, 15000, 3400, '2019-10-02'),

-- ('Hadayek El Ahram Grocery', 'grocery', 'food_items', 19, 14, 7, 160, 20000, 28000, 5600, '2015-04-12'),
-- ('Hadayek El Ahram Grill', 'restaurant', 'grilled_food', 22, 15, 7, 145, 22000, 27000, 5900, '2014-09-06'),
-- ('Hadayek El Ahram Electronics', 'retail', 'electronics', 13, 11, 6, 110, 24000, 18000, 3800, '2019-12-18'),
-- ('Hadayek El Ahram Repair Center', 'service', 'general_repair', 17, 10, 6, 80, 21000, 15000, 3200, '2017-02-03'),
-- ('Hadayek El Ahram Coffee Lounge', 'cafe', 'coffee_beverages', 8, 12, 7, 90, 25000, 15000, 3500, '2021-06-11'),

-- ('Saft El Laban Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 17000, 34000, 6400, '2011-07-14'),
-- ('Saft El Laban Grill Station', 'restaurant', 'grilled_food', 25, 15, 7, 170, 17500, 31000, 6700, '2010-12-28'),
-- ('Saft El Laban Electronics', 'retail', 'electronics', 15, 11, 6, 115, 19000, 17500, 3800, '2018-06-19'),
-- ('Saft El Laban Repair Hub', 'service', 'general_repair', 19, 10, 6, 85, 16500, 15500, 3200, '2016-03-10'),
-- ('Saft El Laban Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 90, 17000, 15500, 3500, '2019-11-07'),

-- ('Boulaq Dakrour Central Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 17500, 35000, 6600, '2011-03-09'),
-- ('Boulaq Dakrour Grill House', 'restaurant', 'grilled_food', 26, 15, 7, 175, 18000, 32000, 6900, '2010-05-27'),
-- ('Boulaq Dakrour Electronics', 'retail', 'electronics', 16, 11, 6, 120, 19500, 18000, 3900, '2018-10-14'),
-- ('Boulaq Dakrour Repair Services', 'service', 'general_repair', 21, 10, 6, 90, 17000, 16000, 3300, '2016-01-18'),
-- ('Boulaq Dakrour Coffee Corner', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 17500, 16000, 3600, '2019-09-29'),

(232, 'Kafr Tohormos Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17000, 33000, 6200, '2012-02-23'),
(233, 'Kafr Tohormos Grill', 'restaurant', 'grilled_food', 25, 15, 7, 165, 17500, 30000, 6500, '2011-07-16'),
(234, 'Kafr Tohormos Electronics', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-01-30'),
(235, 'Kafr Tohormos Repair Hub', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-09-04'),
(236, 'Kafr Tohormos Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-05-22'),

(242, 'Meet Halfa Grocery', 'grocery', 'food_items', 21, 14, 7, 175, 17000, 32000, 6100, '2012-01-19'),
(243, 'Meet Halfa Grill Station', 'restaurant', 'grilled_food', 24, 15, 7, 160, 17500, 29000, 6300, '2011-08-11'),
(244, 'Meet Halfa Electronics', 'retail', 'electronics', 14, 11, 6, 105, 18500, 16500, 3600, '2018-04-25'),
(245, 'Meet Halfa Repair Center', 'service', 'general_repair', 18, 10, 6, 75, 16000, 14500, 3000, '2016-06-02'),
(246, 'Meet Halfa Coffee Corner', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16500, 14500, 3300, '2019-10-09'),

(237, 'Bashtil Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17000, 33000, 6200, '2012-03-03'),
(238, 'Bashtil Grill House', 'restaurant', 'grilled_food', 25, 15, 7, 165, 17500, 30000, 6500, '2011-09-24'),
(239, 'Bashtil Electronics Hub', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-02-15'),
(240, 'Bashtil Repair Services', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-11-22'),
(241, 'Bashtil Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-07-16'),

(176, 'October Central Grocery', 'grocery', 'food_items', 19, 14, 7, 170, 21000, 30000, 5800, '2015-04-12'),
(177, 'October Grill House', 'restaurant', 'grilled_food', 22, 15, 7, 155, 23000, 28000, 6100, '2014-08-23'),
(178, 'October Electronics', 'retail', 'electronics', 13, 11, 6, 110, 26000, 17000, 3700, '2019-01-15'),
(179, 'October Repair Center', 'service', 'general_repair', 17, 10, 6, 80, 22000, 14500, 3200, '2017-06-04'),
(180, 'October Coffee Stop', 'cafe', 'coffee_beverages', 8, 12, 7, 85, 25000, 15000, 3400, '2021-03-09'),

(177, 'Mall of Arabia Mini Market', 'grocery', 'food_items', 12, 13, 7, 120, 32000, 22000, 4500, '2020-02-17'),
(177, 'Mall of Arabia Food Court Grill', 'restaurant', 'fast_food', 6, 16, 7, 95, 35000, 26000, 7200, '2022-07-01'),
(177, 'Mall of Arabia Electronics', 'retail', 'electronics', 9, 11, 6, 90, 34000, 16000, 3900, '2021-11-28'),
(177, 'Mall of Arabia Repair Kiosk', 'service', 'electronics_repair', 4, 10, 7, 35, 18000, 8000, 1900, '2023-05-14'),
(177, 'Mall of Arabia Coffee', 'cafe', 'coffee_beverages', 5, 14, 7, 60, 30000, 14000, 3600, '2022-09-06'),

(181, 'Sheikh Zayed Central Grocery', 'grocery', 'food_items', 18, 14, 7, 160, 27000, 29000, 5600, '2016-03-11'),
(182, 'Sheikh Zayed Grill House', 'restaurant', 'grilled_food', 21, 15, 7, 145, 29000, 27000, 6000, '2015-10-02'),
(183, 'Sheikh Zayed electronics', 'retail', 'electronics', 11, 11, 6, 105, 31000, 17500, 3800, '2020-01-19'),
(184, 'Sheikh Zayed Repair Hub', 'service', 'general_repair', 15, 10, 6, 75, 25000, 14500, 3100, '2018-07-07'),
(185, 'Sheikh Zayed Coffee Lounge', 'cafe', 'coffee_beverages', 7, 12, 7, 85, 30000, 15000, 3400, '2021-06-22'),

(186, 'Hawamdeya Main Grocery', 'grocery', 'food_items', 24, 14, 7, 185, 17000, 34000, 6400, '2012-02-18'),
(187, 'Hawamdeya Grill Station', 'restaurant', 'grilled_food', 27, 15, 7, 170, 17500, 31000, 6700, '2010-11-03'),
(188, 'Hawamdeya Electronics', 'retail', 'electronics', 16, 11, 6, 115, 19000, 17500, 3800, '2018-04-27'),
(189, 'Hawamdeya Repair Services', 'service', 'general_repair', 20, 10, 6, 85, 16500, 15500, 3200, '2016-09-15'),
(190, 'Hawamdeya Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 17000, 15500, 3500, '2019-12-01'),

(191, 'Fayoum Road Mega Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 18500, 33000, 6200, '2013-01-29'),
(192, 'Fayoum Road Grill', 'restaurant', 'grilled_food', 25, 15, 7, 165, 18000, 30000, 6500, '2011-08-14'),
(193, 'Fayoum Road Electronics', 'retail', 'electronics', 14, 11, 6, 110, 19500, 17000, 3700, '2018-02-06'),
(194, 'Fayoum Road Repair Hub', 'service', 'general_repair', 18, 10, 6, 80, 17000, 15000, 3100, '2016-11-20'),
(195, 'Fayoum Road Coffee Stop', 'cafe', 'coffee_beverages', 9, 12, 7, 85, 17500, 15000, 3400, '2019-07-05'),

(196, 'Maryotia Central Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 17500, 34000, 6400, '2012-03-17'),
(197, 'Maryotia Grill Station', 'restaurant', 'grilled_food', 26, 15, 7, 170, 18000, 31000, 6700, '2010-12-12'),
(198, 'Maryotia Electronics', 'retail', 'electronics', 15, 11, 6, 115, 19500, 17500, 3800, '2018-06-30'),
(199, 'Maryotia Repair Services', 'service', 'general_repair', 19, 10, 6, 85, 17000, 15500, 3200, '2016-04-19'),
(200, 'Maryotia Coffee Corner', 'cafe', 'coffee_beverages', 10, 12, 7, 90, 17500, 15500, 3500, '2019-10-27'),

-- ⚠ Intentional duplicate (same name + location, slightly different costs)
(176, 'October Central Grocery', 'grocery', 'food_items', 19, 14, 7, 170, 21500, 30500, 5900, '2015-04-12'),

-- ⚠ Intentional inconsistency: very low rent for large area
(224, 'Industrial Supplies Warehouse', 'retail', 'industrial_tools', 8, 10, 6, 220, 9000, 18000, 4100, '2021-09-03'),

(105, 'Ring Road Snack Kiosk', 'restaurant', 'snacks', 3, 12, 7, 30, 12000, 7000, 1800, '2023-02-11'),
(105, 'Ring Road Coffee Stop', 'cafe', 'coffee_beverages', 6, 12, 7, 65, 15000, 12000, 2900, '2022-06-08'),
(105, 'Ring Road Mini Market', 'grocery', 'food_items', 15, 13, 7, 95, 16000, 14000, 3100, '2019-11-16'),
(176, '6th October Student Cafe', 'cafe', 'coffee_snacks', 4, 14, 7, 60, 14000, 11000, 2700, '2022-10-01'),
(177, '6th October Copy Center', 'service', 'printing', 18, 10, 6, 55, 13000, 10000, 2500, '2015-05-21'),

(178, '6th October Pharmacy', 'pharmacy', 'medicines', 11, 12, 7, 65, 17000, 13000, 3000, '2018-08-09'),
(179, '6th October Phone Repair', 'service', 'mobile_repair', 9, 10, 6, 50, 14000, 9000, 2200, '2020-03-12'),
(180, '6th October Bakery', 'bakery', 'bread_pastries', 20, 13, 7, 80, 15000, 17000, 3600, '2014-12-27'),
(181, 'Sheikh Zayed Organic Market', 'grocery', 'organic_food', 6, 11, 6, 75, 32000, 15000, 3400, '2021-04-04'),
(182, 'Sheikh Zayed Sushi Bar', 'restaurant', 'asian_food', 5, 12, 6, 90, 35000, 22000, 5200, '2022-08-18'),

(183, 'Sheikh Zayed Pet Shop', 'retail', 'pet_supplies', 7, 10, 6, 70, 28000, 12000, 2600, '2020-12-02'),
(184, 'Sheikh Zayed Laundry', 'service', 'laundry', 14, 12, 7, 65, 20000, 11000, 2900, '2017-06-25'),
(185, 'Sheikh Zayed Dessert Cafe', 'cafe', 'desserts', 4, 12, 7, 80, 31000, 14000, 3600, '2022-05-10'),
(186, 'Hawamdeya Market Butchery', 'retail', 'meat_products', 29, 12, 7, 85, 14000, 18000, 3700, '2009-07-01'),
(187, 'Hawamdeya Fish Grill', 'restaurant', 'seafood', 24, 15, 7, 150, 16500, 26000, 6000, '2012-11-19'),

(188, 'Hawamdeya Spare Parts', 'retail', 'auto_parts', 17, 11, 6, 95, 15000, 16000, 3300, '2016-03-08'),
(189, 'Hawamdeya Juice Bar', 'cafe', 'fresh_juice', 6, 11, 7, 55, 13000, 9000, 2400, '2021-07-22'),
(190, 'Hawamdeya Grocery Express', 'grocery', 'food_items', 10, 13, 7, 100, 14500, 16000, 3200, '2019-10-14'),
(196, 'Maryotia Tourism Gifts', 'retail', 'souvenirs', 12, 10, 6, 85, 22000, 13000, 2800, '2018-04-03'),
(197, 'Maryotia Pizza Corner', 'restaurant', 'fast_food', 7, 14, 7, 75, 20000, 16000, 4200, '2020-09-09'),

(198, 'Maryotia Mini Market', 'grocery', 'food_items', 16, 14, 7, 110, 17000, 20000, 3900, '2016-12-01'),
(199, 'Maryotia Phone Repair', 'service', 'mobile_repair', 11, 10, 6, 55, 14000, 10000, 2400, '2019-02-18'),
(200, 'Maryotia Coffee Stop', 'cafe', 'coffee_beverages', 8, 12, 7, 70, 16000, 12000, 2900, '2021-01-27'),
(191, 'Fayoum Road Auto Service', 'service', 'auto_repair', 21, 10, 6, 120, 18000, 20000, 4500, '2014-06-06'),
(192, 'Fayoum Road Tyres Shop', 'retail', 'auto_parts', 18, 11, 6, 95, 16500, 15000, 3300, '2016-08-24'),

(193, 'Fayoum Road Coffee Kiosk', 'cafe', 'coffee_beverages', 3, 10, 7, 25, 9000, 6000, 1500, '2023-01-05'),
(194, 'Fayoum Road Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 90, 14000, 15000, 3000, '2020-05-17'),
(195, 'Fayoum Road Shawarma', 'restaurant', 'fast_food', 14, 14, 7, 85, 15500, 18000, 4200, '2017-09-29'),
(176, 'October Industrial Canteen', 'restaurant', 'egyptian_food', 0, 10, 6, 100, 12000, 16000, 3800, '2022-02-02'),
(177, 'October Tools Store', 'retail', 'hardware_tools', 19, 11, 6, 115, 18000, 17000, 3500, '2015-11-11'),

(178, 'October Workers Grocery', 'grocery', 'food_items', 12, 14, 7, 130, 15000, 20000, 4200, '2018-07-08'),
(179, 'October Phone Accessories', 'retail', 'mobile_accessories', 6, 11, 6, 60, 14000, 10000, 2300, '2021-06-19'),
(180, 'October Tea & Coffee', 'cafe', 'tea_drinks', 10, 12, 7, 75, 16000, 13000, 3000, '2019-03-14'),
(181, 'Sheikh Zayed Night Cafe', 'cafe', 'coffee_beverages', 8, 16, 7, 90, 32000, 17000, 5200, '2020-08-30'),
(182, 'Sheikh Zayed Late Snacks', 'restaurant', 'snacks', 5, 15, 7, 70, 28000, 15000, 4300, '2022-04-11'),

(183, 'Sheikh Zayed Convenience Store', 'grocery', 'food_items', 7, 13, 7, 95, 26000, 16000, 3600, '2021-01-03'),
(184, 'Sheikh Zayed Vape Shop', 'retail', 'vape_products', 4, 11, 6, 50, 24000, 9000, 2100, '2023-03-07'),
(185, 'Sheikh Zayed Car Wash', 'service', 'car_wash', 13, 12, 7, 140, 20000, 14000, 4000, '2017-05-22'),
(186, 'Hawamdeya Public Bakery', 'bakery', 'bread_pastries', 31, 14, 7, 90, 12000, 19000, 3600, '2008-09-01'),
(187, 'Hawamdeya Community Pharmacy', 'pharmacy', 'medicines', 16, 12, 7, 65, 14000, 13000, 3000, '2016-12-14'),

(188, 'Hawamdeya Phone Shop', 'retail', 'mobile_phones', 9, 11, 6, 55, 13500, 10000, 2300, '2019-04-27'),
(189, 'Hawamdeya Repair Corner', 'service', 'electronics_repair', 14, 10, 6, 60, 14500, 11000, 2600, '2017-08-06'),
(190, 'Hawamdeya Coffee Spot', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 15000, 12000, 2900, '2021-11-15'),
(196, 'Maryotia BBQ & Grill', 'restaurant', 'grilled_food', 23, 15, 7, 160, 18000, 30000, 6500, '2012-06-20'),
(197, 'Maryotia Grocery Plus', 'grocery', 'food_items', 18, 14, 7, 150, 16500, 26000, 5200, '2016-10-02'),

(198, 'Maryotia Electronics World', 'retail', 'electronics', 12, 11, 6, 100, 19000, 16000, 3500, '2019-02-14'),
(199, 'Maryotia Services Center', 'service', 'general_repair', 17, 10, 6, 75, 16000, 14500, 3000, '2016-05-09'),
(200, 'Maryotia Coffee Lounge', 'cafe', 'coffee_beverages', 9, 12, 7, 85, 17000, 15000, 3400, '2019-09-18'),
(191, 'Fayoum Road Fruits Market', 'retail', 'fruits_vegetables', 20, 13, 7, 120, 15000, 18000, 3600, '2015-04-26'),
(192, 'Fayoum Road Meat Shop', 'retail', 'meat_products', 27, 12, 7, 85, 14000, 19000, 3700, '2011-03-17'),

(193, 'Fayoum Road Mini Cafe', 'cafe', 'coffee_beverages', 6, 11, 7, 55, 13000, 10000, 2400, '2020-08-31'),
(194, 'Fayoum Road Repair Garage', 'service', 'auto_repair', 22, 10, 6, 130, 19000, 21000, 4600, '2013-12-04'),
(195, 'Fayoum Road Grocery Express', 'grocery', 'food_items', 11, 14, 7, 100, 14500, 16000, 3200, '2019-01-20'),
(176, 'October Roadside Cafe', 'cafe', 'coffee_beverages', 5, 12, 7, 60, 13500, 11000, 2600, '2022-06-13'),
(177, 'October Mini Market', 'grocery', 'food_items', 13, 14, 7, 110, 15000, 18000, 3500, '2018-10-08'),

(178, 'October Electronics Repair', 'service', 'electronics_repair', 10, 10, 6, 65, 14500, 12000, 2700, '2019-07-24'),
(179, 'October Shawarma', 'restaurant', 'fast_food', 16, 14, 7, 90, 16000, 19000, 4300, '2016-05-19'),
(180, 'October Coffee Corner', 'cafe', 'coffee_beverages', 8, 12, 7, 75, 15500, 13000, 3000, '2021-02-08'),
(181, 'Sheikh Zayed Central Grocery ', 'grocery', 'food_items', 18, 14, 7, 160, 27000, 29000, 5600, '2016-03-11'),
(182, 'Sheikh Zayed Burger Joint', 'restaurant', 'fast_food', 6, 14, 7, 85, 31000, 18000, 4600, '2022-12-04'),

(183, 'Sheikh Zayed Electronics Hub', 'retail', 'electronics', 11, 11, 6, 105, 30000, 17000, 3700, '2020-09-19'),
(184, 'Sheikh Zayed Repair Center', 'service', 'general_repair', 15, 10, 6, 75, 25000, 14500, 3100, '2018-07-07'),
(185, 'Sheikh Zayed Coffee Point', 'cafe', 'coffee_beverages', 7, 12, 7, 85, 29000, 15000, 3400, '2021-06-22'),
(186, 'Hawamdeya Night Cafe', 'cafe', 'coffee_beverages', 6, 16, 7, 80, 16000, 15000, 4800, '2020-08-14'),
(187, 'Hawamdeya Late Grill', 'restaurant', 'grilled_food', 19, 16, 7, 145, 17500, 28000, 6200, '2015-10-09'),

(188, 'Hawamdeya Mini Grocery', 'grocery', 'food_items', 8, 13, 7, 95, 14000, 15000, 3000, '2021-01-26'),
(189, 'Hawamdeya Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 13000, 9000, 2100, '2022-05-02'),
(190, 'Hawamdeya Coffee Corner', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 15000, 12000, 2900, '2021-11-15'),
(211, 'Oseem Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 17000, 33000, 6200, '2012-02-06'),
(212, 'Oseem Grill House', 'restaurant', 'grilled_food', 25, 15, 7, 165, 17500, 30000, 6500, '2011-09-19'),

(213, 'Oseem Electronics', 'retail', 'electronics', 15, 11, 6, 110, 19000, 17000, 3700, '2018-03-27'),
(214, 'Oseem Repair Services', 'service', 'general_repair', 19, 10, 6, 80, 16500, 15000, 3100, '2016-11-14'),
(215, 'Oseem Coffee Stop', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 17000, 15000, 3400, '2019-07-08'),
(216, 'Manshiyet El Qanater Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 16500, 34000, 6400, '2011-04-23'),
(217, 'Manshiyet El Qanater Grill', 'restaurant', 'grilled_food', 26, 15, 7, 170, 17000, 31000, 6700, '2010-12-01'),

(218, 'Manshiyet El Qanater Electronics', 'retail', 'electronics', 16, 11, 6, 115, 18500, 17500, 3800, '2018-05-11'),
(219, 'Manshiyet El Qanater Repair Hub', 'service', 'general_repair', 20, 10, 6, 85, 16000, 15500, 3200, '2016-08-18'),
(220, 'Manshiyet El Qanater Coffee', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 16500, 15500, 3500, '2019-10-03'),
(221, 'Qalyub Central Grocery', 'grocery', 'food_items', 24, 14, 7, 190, 18000, 35000, 6600, '2011-02-12'),
(222, 'Qalyub Grill Station', 'restaurant', 'grilled_food', 27, 15, 7, 175, 18500, 32000, 6900, '2010-05-27'),

(223, 'Qalyub Electronics Hub', 'retail', 'electronics', 16, 11, 6, 120, 20000, 18000, 3900, '2018-09-09'),
(224, 'Qalyub Repair Center', 'service', 'general_repair', 21, 10, 6, 90, 17500, 16000, 3300, '2016-01-21'),
(225, 'Qalyub Coffee Corner', 'cafe', 'coffee_beverages', 12, 12, 7, 95, 18000, 16000, 3600, '2019-12-11'),
(226, 'Ezbet El Nakhl Central Grocery', 'grocery', 'food_items', 25, 14, 7, 195, 17500, 36000, 6800, '2011-01-18'),
(227, 'Ezbet El Nakhl Grill House', 'restaurant', 'grilled_food', 28, 15, 7, 180, 18000, 33000, 7100, '2010-04-09'),

(228, 'Ezbet El Nakhl Electronics', 'retail', 'electronics', 17, 11, 6, 125, 20500, 18500, 4000, '2018-07-01'),
(229, 'Ezbet El Nakhl Repair Services', 'service', 'general_repair', 22, 10, 6, 95, 18000, 16500, 3400, '2016-03-05'),
(230, 'Ezbet El Nakhl Coffee Stop', 'cafe', 'coffee_beverages', 13, 12, 7, 100, 18500, 16500, 3700, '2019-09-17'),
(236, 'Bashtil Central Grocery', 'grocery', 'food_items', 23, 14, 7, 185, 17000, 34000, 6400, '2012-02-01'),
(237, 'Bashtil Grill Station', 'restaurant', 'grilled_food', 26, 15, 7, 170, 17500, 31000, 6700, '2011-06-22'),

(238, 'Bashtil Electronics Hub', 'retail', 'electronics', 16, 11, 6, 115, 19500, 17500, 3800, '2018-10-05'),
(239, 'Bashtil Repair Center', 'service', 'general_repair', 20, 10, 6, 85, 17000, 15500, 3200, '2016-07-14'),
(240, 'Bashtil Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 90, 17500, 15500, 3500, '2019-04-26'),
(241, 'Meet Halfa Central Grocery', 'grocery', 'food_items', 22, 14, 7, 180, 16500, 33000, 6200, '2012-03-09'),
(242, 'Meet Halfa Grill House', 'restaurant', 'grilled_food', 25, 15, 7, 165, 17000, 30000, 6500, '2011-10-01'),

(243, 'Meet Halfa Electronics', 'retail', 'electronics', 15, 11, 6, 110, 18500, 17000, 3700, '2018-04-12'),
(244, 'Meet Halfa Repair Hub', 'service', 'general_repair', 19, 10, 6, 80, 16000, 15000, 3100, '2016-12-18'),
(245, 'Meet Halfa Coffee Corner', 'cafe', 'coffee_beverages', 10, 12, 7, 85, 16500, 15000, 3400, '2019-08-02'),
-- ('Basateen El Qanater Grocery', 'grocery', 'food_items', 21, 14, 7, 175, 16000, 32000, 6000, '2013-01-11'),
-- ('Basateen El Qanater Grill', 'restaurant', 'grilled_food', 24, 15, 7, 160, 16500, 29000, 6300, '2012-08-27'),

-- ('Basateen El Qanater Electronics', 'retail', 'electronics', 14, 11, 6, 105, 18000, 16500, 3600, '2018-02-20'),
-- ('Basateen El Qanater Repair', 'service', 'general_repair', 18, 10, 6, 75, 15500, 14500, 3000, '2016-05-06'),
-- ('Basateen El Qanater Coffee', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16000, 14500, 3300, '2019-10-14'),
(221, 'Qalyub Station Mini Market', 'grocery', 'food_items', 14, 13, 7, 100, 15000, 16000, 3200, '2019-06-01'),
(221, 'Qalyub Station Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 14000, 12000, 2800, '2021-09-08'),

(221, 'Qalyub Station Coffee', 'cafe', 'coffee_beverages', 5, 12, 7, 50, 13500, 11000, 2600, '2022-04-16'),
(221, 'Qalyub Station Phone Repair', 'service', 'mobile_repair', 8, 10, 6, 45, 13000, 9500, 2200, '2020-01-22'),
(221, 'Qalyub Station Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 60, 15000, 13000, 3000, '2018-07-07'),

-- ⚠ intentional casing + spacing issue
(226, 'Ezbet El Nakhl Central Grocery', 'grocery', 'food_items', 25, 14, 7, 195, 17500, 36000, 6800, '2011-01-18'),

(227, 'Ezbet El Nakhl Fish Grill', 'restaurant', 'seafood', 26, 15, 7, 155, 17000, 28000, 6200, '2012-09-02'),
(228, 'Ezbet El Nakhl Phone Accessories', 'retail', 'mobile_accessories', 7, 11, 6, 60, 14500, 10000, 2300, '2021-03-19'),
(229, 'Ezbet El Nakhl Juice Bar', 'cafe', 'fresh_juice', 6, 11, 7, 55, 13500, 9500, 2400, '2021-08-23'),
(230, 'Ezbet El Nakhl Grocery Express', 'grocery', 'food_items', 11, 13, 7, 105, 15000, 17000, 3400, '2019-11-01'),
(236, 'Bashtil Roadside Cafe', 'cafe', 'coffee_beverages', 5, 12, 7, 60, 14000, 11500, 2700, '2022-02-14'),

(237, 'Bashtil Fruits Market', 'retail', 'fruits_vegetables', 18, 13, 7, 120, 15000, 18000, 3600, '2015-04-19'),
(238, 'Bashtil Meat Shop', 'retail', 'meat_products', 27, 12, 7, 85, 14500, 19500, 3800, '2011-07-03'),
(239, 'Bashtil Repair Garage', 'service', 'auto_repair', 22, 10, 6, 130, 18500, 21000, 4600, '2013-11-27'),
(240, 'Bashtil Coffee Lounge', 'cafe', 'coffee_beverages', 9, 12, 7, 80, 16500, 14500, 3300, '2019-05-21'),
(211, 'Oseem Market Bakery', 'bakery', 'bread_pastries', 24, 13, 7, 90, 14000, 18000, 3600, '2012-12-09'),

(212, 'Oseem Market Pharmacy', 'pharmacy', 'medicines', 16, 12, 7, 65, 15000, 13000, 3000, '2016-08-01'),
(213, 'Oseem Phone Shop', 'retail', 'mobile_phones', 9, 11, 6, 55, 13500, 10000, 2300, '2019-03-27'),
(214, 'Oseem Repair Corner', 'service', 'electronics_repair', 14, 10, 6, 60, 14500, 11000, 2600, '2017-07-15'),
(215, 'Oseem Coffee Spot', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 15000, 12000, 2900, '2021-10-06'),
(216, 'Manshiyet El Qanater Public Bakery', 'bakery', 'bread_pastries', 30, 14, 7, 95, 12000, 19000, 3600, '2008-08-22'),

(217, 'Manshiyet El Qanater Pharmacy', 'pharmacy', 'medicines', 15, 12, 7, 65, 14000, 13000, 3000, '2016-11-09'),
(218, 'Manshiyet El Qanater Phone Shop', 'retail', 'mobile_phones', 8, 11, 6, 55, 13500, 10000, 2300, '2019-05-14'),
(219, 'Manshiyet El Qanater Repair Corner', 'service', 'electronics_repair', 13, 10, 6, 60, 14500, 11000, 2600, '2017-09-21'),
(220, 'Manshiyet El Qanater Coffee Spot', 'cafe', 'coffee_beverages', 6, 12, 7, 70, 15000, 12000, 2900, '2021-12-12'),
(221, 'Qalyub Market Fruits', 'retail', 'fruits_vegetables', 21, 13, 7, 130, 15500, 18500, 3700, '2014-04-07'),

(222, 'Qalyub Market Meat', 'retail', 'meat_products', 28, 12, 7, 90, 14500, 20000, 3900, '2011-02-19'),
(223, 'Qalyub Auto Service', 'service', 'auto_repair', 23, 10, 6, 140, 19000, 22000, 4800, '2013-10-28'),
(224, 'Qalyub Mini Cafe', 'cafe', 'coffee_beverages', 6, 11, 7, 55, 13500, 10500, 2400, '2020-07-02'),
(225, 'Qalyub Grocery Express', 'grocery', 'food_items', 12, 14, 7, 105, 15000, 17000, 3400, '2019-01-18'),
(226, 'Ezbet El Nakhl Auto Parts', 'retail', 'auto_parts', 18, 11, 6, 95, 15000, 16000, 3300, '2016-06-11'),

(227, 'Ezbet El Nakhl Tyres Shop', 'retail', 'auto_parts', 20, 11, 6, 100, 15500, 16500, 3400, '2015-08-24'),
(228, 'Ezbet El Nakhl Garage', 'service', 'auto_repair', 24, 10, 6, 150, 19500, 23000, 5200, '2012-05-17'),
(229, 'Ezbet El Nakhl Fuel Snacks', 'restaurant', 'snacks', 5, 12, 7, 45, 12000, 9000, 2300, '2022-03-05'),
(230, 'Ezbet El Nakhl Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 11000, 8500, 2000, '2023-01-11'),
(236, 'Bashtil Station Grocery', 'grocery', 'food_items', 14, 13, 7, 110, 14500, 16000, 3200, '2019-09-19'),

(237, 'Bashtil Station Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 14000, 12000, 2800, '2021-04-08'),
(238, 'Bashtil Station Coffee', 'cafe', 'coffee_beverages', 5, 12, 7, 50, 13500, 11000, 2600, '2022-02-27'),
(239, 'Bashtil Station Phone Repair', 'service', 'mobile_repair', 8, 10, 6, 45, 13000, 9500, 2200, '2020-10-06'),
(240, 'Bashtil Station Pharmacy', 'pharmacy', 'medicines', 12, 12, 7, 60, 15000, 13000, 3000, '2018-12-03'),
(241, 'Meet Halfa Auto Service', 'service', 'auto_repair', 21, 10, 6, 135, 18500, 21500, 4700, '2014-11-12'),

(242, 'Meet Halfa Tyres Shop', 'retail', 'auto_parts', 19, 11, 6, 100, 16000, 16500, 3400, '2016-04-21'),
(243, 'Meet Halfa Fuel Snacks', 'restaurant', 'snacks', 5, 12, 7, 45, 12000, 9000, 2300, '2022-07-09'),
(244, 'Meet Halfa Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 11000, 8500, 2000, '2023-03-18'),
(245, 'Meet Halfa Grocery Express', 'grocery', 'food_items', 11, 14, 7, 100, 14500, 16000, 3200, '2019-05-28'),
(211, 'Oseem Night Cafe', 'cafe', 'coffee_beverages', 7, 16, 7, 85, 16500, 15500, 4800, '2020-08-13'),

(212, 'Oseem Late Grill', 'restaurant', 'grilled_food', 20, 16, 7, 150, 17500, 29000, 6200, '2015-09-06'),
(213, 'Oseem Convenience Store', 'grocery', 'food_items', 9, 13, 7, 95, 14500, 15500, 3000, '2021-02-22'),
(214, 'Oseem Vape Shop', 'retail', 'vape_products', 4, 11, 6, 50, 13500, 9000, 2100, '2023-04-02'),
(215, 'Oseem Car Wash', 'service', 'car_wash', 14, 12, 7, 140, 19500, 14000, 4000, '2017-07-31'),
(216, 'Manshiyet El Qanater Night Cafe', 'cafe', 'coffee_beverages', 6, 16, 7, 80, 15500, 14500, 4700, '2020-09-19'),

(217, 'Manshiyet El Qanater Late Grill', 'restaurant', 'grilled_food', 19, 16, 7, 145, 16500, 28000, 6100, '2015-11-11'),
(218, 'Manshiyet El Qanater Mini Grocery', 'grocery', 'food_items', 8, 13, 7, 95, 14000, 15000, 3000, '2021-03-03'),
(219, 'Manshiyet El Qanater Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 13000, 9000, 2100, '2022-06-14'),
(220, 'Manshiyet El Qanater Coffee Corner', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 15000, 12000, 2900, '2021-11-29'),
(221, 'Qalyub Night Cafe', 'cafe', 'coffee_beverages', 6, 16, 7, 85, 17000, 15500, 4900, '2020-10-07'),

(222, 'Qalyub Late Grill', 'restaurant', 'grilled_food', 21, 16, 7, 150, 18000, 30000, 6400, '2015-12-19'),
(223, 'Qalyub Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 95, 14500, 15500, 3000, '2021-01-16'),
(224, 'Qalyub Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 13000, 9000, 2100, '2022-05-21'),
(225, 'Qalyub Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 11000, 8500, 2000, '2023-02-08'),
(226, 'Ezbet El Nakhl Night Cafe', 'cafe', 'coffee_beverages', 6, 16, 7, 85, 16500, 15500, 4800, '2020-09-27'),

(227, 'Ezbet El Nakhl Late Grill', 'restaurant', 'grilled_food', 20, 16, 7, 150, 17500, 29000, 6200, '2015-10-23'),
(228, 'Ezbet El Nakhl Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 95, 14000, 15000, 3000, '2021-02-11'),
(229, 'Ezbet El Nakhl Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 13000, 9000, 2100, '2022-04-16'),
(230, 'Ezbet El Nakhl Coffee Corner', 'cafe', 'coffee_beverages', 7, 12, 7, 70, 15000, 12000, 2900, '2021-10-20'),
(6, 'Shubra Central Grocery', 'grocery', 'food_items', 26, 14, 7, 195, 18500, 36000, 6800, '2011-01-14'),

(7, 'Shubra Grill House', 'restaurant', 'grilled_food', 29, 15, 7, 180, 19000, 33000, 7100, '2010-03-22'),
(8, 'Shubra Electronics', 'retail', 'electronics', 18, 11, 6, 130, 21000, 18500, 4100, '2017-09-11'),
(9, 'Shubra Repair Services', 'service', 'general_repair', 23, 10, 6, 95, 18500, 16500, 3400, '2015-05-19'),
(10, 'Shubra Coffee Stop', 'cafe', 'coffee_beverages', 14, 12, 7, 105, 19000, 16500, 3700, '2019-08-07'),
(7, 'Rod El Farag Main Grocery', 'grocery', 'food_items', 25, 14, 7, 190, 17500, 35000, 6600, '2012-02-09'),

(7, 'Rod El Farag Grill Station', 'restaurant', 'grilled_food', 27, 15, 7, 175, 18000, 32000, 6900, '2010-11-15'),
(7, 'Rod El Farag Electronics Hub', 'retail', 'electronics', 17, 11, 6, 120, 20000, 18000, 3900, '2018-04-04'),
(7, 'Rod El Farag Repair Center', 'service', 'general_repair', 22, 10, 6, 90, 17500, 16000, 3300, '2016-01-28'),
(7, 'Rod El Farag Coffee Corner', 'cafe', 'coffee_beverages', 13, 12, 7, 95, 18000, 16000, 3600, '2019-11-03'),
-- ('Abbasiya Central Grocery', 'grocery', 'food_items', 24, 14, 7, 185, 19500, 34000, 6500, '2012-05-06'),

-- ('Abbasiya Grill House', 'restaurant', 'grilled_food', 26, 15, 7, 170, 20000, 31000, 6700, '2011-08-24'),
-- ('Abbasiya Electronics', 'retail', 'electronics', 16, 11, 6, 125, 22000, 18500, 4000, '2018-01-19'),
-- ('Abbasiya Repair Workshop', 'service', 'general_repair', 21, 10, 6, 95, 19000, 16500, 3400, '2016-10-02'),
-- ('Abbasiya Coffee Lounge', 'cafe', 'coffee_beverages', 12, 12, 7, 100, 21000, 17000, 3700, '2020-02-14'),
-- ('Hadayek El Kobba Grocery', 'grocery', 'food_items', 23, 14, 7, 180, 20000, 33000, 6300, '2013-03-17'),

-- ('Hadayek El Kobba Grill', 'restaurant', 'grilled_food', 25, 15, 7, 165, 21000, 30000, 6600, '2012-07-29'),
-- ('Hadayek El Kobba Electronics', 'retail', 'electronics', 15, 11, 6, 120, 23000, 18000, 3900, '2019-04-06'),
-- ('Hadayek El Kobba Repair Hub', 'service', 'general_repair', 20, 10, 6, 90, 20000, 16000, 3300, '2016-09-18'),
-- ('Hadayek El Kobba Coffee Stop', 'cafe', 'coffee_beverages', 11, 12, 7, 95, 22000, 16500, 3600, '2019-12-22'),
(46, 'Nasr City Zone 1 Grocery', 'grocery', 'food_items', 18, 14, 7, 160, 26000, 28000, 5400, '2016-01-25'),

(47, 'Nasr City Zone 1 Grill', 'restaurant', 'grilled_food', 20, 15, 7, 145, 28000, 27000, 5800, '2015-05-11'),
(48, 'Nasr City Zone 1 Electronics', 'retail', 'electronics', 13, 11, 6, 110, 30000, 19000, 3900, '2020-03-08'),
(49, 'Nasr City Zone 1 Repair', 'service', 'general_repair', 16, 10, 6, 80, 24000, 15000, 3100, '2017-08-19'),
(50, 'Nasr City Zone 1 Coffee', 'cafe', 'coffee_beverages', 8, 12, 7, 90, 27000, 15000, 3400, '2021-06-04'),
(21, 'Nasr City Abbas El Akkad Grocery', 'grocery', 'food_items', 17, 14, 7, 155, 30000, 26000, 5200, '2017-02-03'),

(21, 'Nasr City Abbas El Akkad Grill', 'restaurant', 'grilled_food', 19, 15, 7, 140, 32000, 25000, 5600, '2016-09-09'),
(21, 'Nasr City Abbas El Akkad Electronics', 'retail', 'electronics', 12, 11, 6, 105, 34000, 18000, 3800, '2021-01-16'),
(21, 'Nasr City Abbas El Akkad Repair Center', 'service', 'general_repair', 15, 10, 6, 75, 28000, 14500, 3000, '2018-04-27'),
(21, 'Nasr City Abbas El Akkad Coffee', 'cafe', 'coffee_beverages', 7, 12, 7, 85, 31000, 15000, 3300, '2022-05-19'),
(22, 'Nasr City Makram Ebeid Grocery', 'grocery', 'food_items', 16, 14, 7, 150, 29000, 25000, 5100, '2018-06-11'),

(22, 'Nasr City Makram Ebeid Grill', 'restaurant', 'grilled_food', 18, 15, 7, 135, 31000, 24000, 5500, '2017-11-23'),
(22, 'Nasr City Makram Ebeid Electronics', 'retail', 'electronics', 11, 11, 6, 100, 33000, 17000, 3700, '2021-09-14'),
(22, 'Nasr City Makram Ebeid Repair Hub', 'service', 'general_repair', 14, 10, 6, 70, 27000, 14000, 2900, '2018-08-05'),
(22, 'Nasr City Makram Ebeid Coffee', 'cafe', 'coffee_beverages', 6, 12, 7, 80, 30000, 14500, 3200, '2022-03-07'),
(340, 'Nasr City Stadium Grocery', 'grocery', 'food_items', 15, 14, 7, 145, 27000, 24000, 5000, '2019-01-09'),

(340, 'Nasr City Stadium Grill', 'restaurant', 'grilled_food', 17, 15, 7, 130, 29000, 23000, 5400, '2018-07-02'),
(340, 'Nasr City Stadium Electronics', 'retail', 'electronics', 10, 11, 6, 95, 31000, 16000, 3600, '2022-02-18'),
(340, 'Nasr City Stadium Repair', 'service', 'general_repair', 13, 10, 6, 65, 25000, 13500, 2800, '2019-10-12'),
(340, 'Nasr City Stadium Coffee', 'cafe', 'coffee_beverages', 5, 12, 7, 75, 28000, 14000, 3100, '2023-01-21'),
-- ('Shubra El Kheima Market Grocery', 'grocery', 'food_items', 27, 14, 7, 200, 18500, 37000, 6900, '2011-01-31'),

-- ('Shubra El Kheima Market Grill', 'restaurant', 'grilled_food', 29, 15, 7, 185, 19000, 34000, 7200, '2010-06-17'),
-- ('Shubra El Kheima Electronics', 'retail', 'electronics', 18, 11, 6, 140, 21500, 19000, 4200, '2017-12-09'),
-- ('Shubra El Kheima Repair Services', 'service', 'general_repair', 23, 10, 6, 105, 19000, 17000, 3500, '2015-03-04'),
-- ('Shubra El Kheima Coffee Stop', 'cafe', 'coffee_beverages', 14, 12, 7, 110, 19500, 17000, 3800, '2019-09-26'),
(7, 'Rod El Farag Station Grocery', 'grocery', 'food_items', 15, 13, 7, 115, 15500, 17000, 3300, '2019-05-14'),

(7, 'Rod El Farag Station Snacks', 'restaurant', 'snacks', 7, 12, 7, 60, 14500, 12500, 2900, '2021-06-30'),
(7, 'Rod El Farag Station Coffee', 'cafe', 'coffee_beverages', 6, 12, 7, 55, 14000, 11500, 2700, '2022-11-08'),
(7, 'Rod El Farag Station Phone Repair', 'service', 'mobile_repair', 9, 10, 6, 50, 13500, 10000, 2300, '2020-04-15'),
(7, 'Rod El Farag Station Pharmacy', 'pharmacy', 'medicines', 13, 12, 7, 65, 15500, 13500, 3100, '2018-08-12'),
-- ('Abbasiya Market Fruits', 'retail', 'fruits_vegetables', 22, 13, 7, 135, 17500, 19000, 3800, '2014-04-19'),

-- ('Abbasiya Market Meat', 'retail', 'meat_products', 28, 12, 7, 95, 16500, 20500, 4000, '2011-06-07'),
-- ('Abbasiya Auto Service', 'service', 'auto_repair', 24, 10, 6, 145, 21000, 23000, 4900, '2013-09-22'),
-- ('Abbasiya Mini Cafe', 'cafe', 'coffee_beverages', 7, 11, 7, 60, 14500, 11000, 2600, '2020-07-14'),
-- ('Abbasiya Grocery Express', 'grocery', 'food_items', 13, 14, 7, 110, 16000, 17500, 3500, '2019-02-26'),
-- ('Hadayek El Kobba Night Cafe', 'cafe', 'coffee_beverages', 6, 16, 7, 85, 22000, 16000, 5000, '2020-09-11'),

-- ('Hadayek El Kobba Late Grill', 'restaurant', 'grilled_food', 21, 16, 7, 150, 23000, 30000, 6400, '2015-12-02'),
-- ('Hadayek El Kobba Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 95, 20000, 15500, 3100, '2021-03-09'),
-- ('Hadayek El Kobba Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 18500, 9000, 2100, '2022-06-21'),
-- ('Hadayek El Kobba Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 17000, 8500, 2000, '2023-02-02'),

-- ⚠ intentional duplicate + spacing issue
(336, 'Nasr City Zone 1 Grocery ', 'grocery', 'food_items', 18, 14, 7, 160, 26000, 28000, 5400, '2016-01-25'),

(337, 'Nasr City Auto Service', 'service', 'auto_repair', 22, 10, 6, 150, 29000, 23000, 5000, '2014-10-18'),
(338, 'Nasr City Tyres Shop', 'retail', 'auto_parts', 20, 11, 6, 110, 27000, 18000, 3600, '2016-03-05'),
(339, 'Nasr City Fuel Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 25000, 12000, 2900, '2022-08-09'),
(340, 'Nasr City Coffee Kiosk', 'cafe', 'coffee_beverages', 5, 11, 7, 50, 24000, 10000, 2400, '2023-04-13'),
(6, 'Shubra Night Cafe', 'cafe', 'coffee_beverages', 8, 16, 7, 95, 18500, 17000, 5200, '2020-10-03'),

(7, 'Shubra Late Grill', 'restaurant', 'grilled_food', 22, 16, 7, 165, 19500, 31000, 6600, '2015-09-27'),
(8, 'Shubra Mini Grocery', 'grocery', 'food_items', 10, 13, 7, 105, 16000, 17000, 3300, '2021-01-05'),
(9, 'Shubra Phone Accessories', 'retail', 'mobile_accessories', 6, 11, 6, 55, 15000, 9500, 2200, '2022-05-16'),
(10, 'Shubra Coffee Kiosk', 'cafe', 'coffee_beverages', 5, 11, 7, 45, 14500, 9000, 2100, '2023-03-19'),
(7, 'Rod El Farag Night Cafe', 'cafe', 'coffee_beverages', 7, 16, 7, 90, 17500, 16000, 5000, '2020-08-25'),

(7, 'Rod El Farag Late Grill', 'restaurant', 'grilled_food', 21, 16, 7, 155, 18500, 30000, 6400, '2015-11-06'),
(7, 'Rod El Farag Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 95, 15000, 15500, 3000, '2021-02-13'),
(7, 'Rod El Farag Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 14000, 9000, 2100, '2022-04-27'),
(7, 'Rod El Farag Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 13500, 8500, 2000, '2023-01-29'),
-- ('Abbasiya Night Cafe', 'cafe', 'coffee_beverages', 7, 16, 7, 90, 20000, 16500, 5100, '2020-09-18'),

-- ('Abbasiya Late Grill', 'restaurant', 'grilled_food', 22, 16, 7, 160, 21000, 31000, 6600, '2015-12-11'),
-- ('Abbasiya Mini Grocery', 'grocery', 'food_items', 10, 13, 7, 100, 18000, 16000, 3200, '2021-03-17'),
-- ('Abbasiya Phone Accessories', 'retail', 'mobile_accessories', 6, 11, 6, 55, 17000, 9500, 2200, '2022-06-02'),
-- ('Abbasiya Coffee Kiosk', 'cafe', 'coffee_beverages', 5, 11, 7, 45, 16500, 9000, 2100, '2023-02-21'),
-- ('Hadayek El Kobba Auto Parts', 'retail', 'auto_parts', 19, 11, 6, 110, 22000, 17500, 3600, '2016-07-04'),

-- ('Hadayek El Kobba Tyres Shop', 'retail', 'auto_parts', 21, 11, 6, 115, 22500, 18000, 3700, '2015-09-13'),
-- ('Hadayek El Kobba Garage', 'service', 'auto_repair', 25, 10, 6, 160, 26000, 24000, 5200, '2012-05-25'),
-- ('Hadayek El Kobba Fuel Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 21000, 12000, 2900, '2022-09-15'),
-- ('Hadayek El Kobba Grocery Express', 'grocery', 'food_items', 11, 14, 7, 105, 20500, 17000, 3400, '2019-06-11'),
(21, 'Nasr City Night Cafe', 'cafe', 'coffee_beverages', 7, 16, 7, 95, 28000, 16500, 5200, '2020-10-29'),

(22, 'Nasr City Late Grill', 'restaurant', 'grilled_food', 21, 16, 7, 165, 30000, 31000, 6600, '2015-11-22'),
(23, 'Nasr City Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 100, 26000, 16000, 3200, '2021-02-04'),
(24, 'Nasr City Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 55, 24000, 9500, 2200, '2022-05-30'),
(25, 'Nasr City Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 45, 23000, 9000, 2100, '2023-03-11'),
(6, 'Shubra Auto Service', 'service', 'auto_repair', 24, 10, 6, 150, 20000, 23000, 4900, '2013-10-08'),

(7, 'Shubra Tyres Shop', 'retail', 'auto_parts', 22, 11, 6, 115, 19000, 18000, 3700, '2015-02-16'),
(8, 'Shubra Fuel Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 17500, 12000, 2900, '2022-07-18'),
(9, 'Shubra Grocery Express', 'grocery', 'food_items', 12, 14, 7, 110, 16500, 17500, 3500, '2019-01-29'),
(10, 'Shubra Coffee Express', 'cafe', 'coffee_beverages', 8, 12, 7, 75, 17000, 14500, 3300, '2021-08-13'),
(7, 'Rod El Farag Auto Service', 'service', 'auto_repair', 23, 10, 6, 145, 19000, 22000, 4800, '2013-09-16'),

(7, 'Rod El Farag Tyres Shop', 'retail', 'auto_parts', 21, 11, 6, 110, 18000, 17500, 3600, '2015-06-24'),
(7, 'Rod El Farag Fuel Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 16500, 12000, 2900, '2022-08-03'),
(7, 'Rod El Farag Grocery Express', 'grocery', 'food_items', 12, 14, 7, 110, 15500, 17000, 3400, '2019-02-11'),
(7, 'Rod El Farag Coffee Express', 'cafe', 'coffee_beverages', 8, 12, 7, 75, 16000, 14500, 3300, '2021-09-04'),
(33, 'Ain Shams Central Grocery', 'grocery', 'food_items', 27, 14, 7, 200, 17000, 37000, 6900, '2011-01-10'),

(33, 'Ain Shams Grill House', 'restaurant', 'grilled_food', 29, 15, 7, 185, 17500, 34000, 7200, '2010-02-18'),
(34, 'Ain Shams Electronics', 'retail', 'electronics', 18, 11, 6, 140, 19500, 19000, 4200, '2017-08-21'),
(34, 'Ain Shams Repair Services', 'service', 'general_repair', 23, 10, 6, 105, 17500, 17000, 3500, '2015-04-09'),
(35, 'Ain Shams Coffee Stop', 'cafe', 'coffee_beverages', 14, 12, 7, 115, 18000, 17000, 3800, '2019-09-03'),
(31, 'El Matareya Main Grocery', 'grocery', 'food_items', 26, 14, 7, 195, 16500, 36000, 6800, '2012-03-14'),

(32, 'El Matareya Grill Station', 'restaurant', 'grilled_food', 28, 15, 7, 180, 17000, 33000, 7100, '2010-10-05'),
(31, 'El Matareya Electronics Hub', 'retail', 'electronics', 17, 11, 6, 135, 19000, 18500, 4100, '2018-05-27'),
(32, 'El Matareya Repair Center', 'service', 'general_repair', 22, 10, 6, 100, 17000, 16500, 3400, '2016-01-16'),
(31, 'El Matareya Coffee Corner', 'cafe', 'coffee_beverages', 13, 12, 7, 110, 17500, 16500, 3700, '2019-12-08'),
(33, 'Ain Shams University Grocery', 'grocery', 'food_items', 15, 14, 7, 150, 21000, 25000, 5200, '2016-02-11'),

(33, 'Ain Shams University Grill', 'restaurant', 'fast_food', 12, 15, 7, 120, 22000, 24000, 5600, '2017-10-07'),
(33, 'Ain Shams University Electronics', 'retail', 'electronics', 10, 11, 6, 100, 24000, 17000, 3800, '2020-04-22'),
(33, 'Ain Shams University Repair', 'service', 'general_repair', 14, 10, 6, 75, 20000, 14500, 3100, '2018-06-30'),
(33, 'Ain Shams University Coffee', 'cafe', 'coffee_beverages', 8, 12, 7, 90, 23000, 15000, 3400, '2021-11-19'),
(26, 'Heliopolis Fringe Grocery', 'grocery', 'food_items', 19, 14, 7, 165, 26000, 28000, 5400, '2016-04-03'),

(27, 'Heliopolis Fringe Grill', 'restaurant', 'grilled_food', 21, 15, 7, 150, 28000, 27000, 5800, '2015-09-12'),
(28, 'Heliopolis Fringe Electronics', 'retail', 'electronics', 13, 11, 6, 115, 30000, 19000, 3900, '2020-02-08'),
(29, 'Heliopolis Fringe Repair Hub', 'service', 'general_repair', 16, 10, 6, 85, 24000, 15000, 3100, '2017-07-26'),
(30, 'Heliopolis Fringe Coffee', 'cafe', 'coffee_beverages', 9, 12, 7, 95, 27000, 15000, 3400, '2022-01-14'),
(337, 'Nasr City East Grocery', 'grocery', 'food_items', 18, 14, 7, 160, 29000, 26000, 5200, '2017-03-21'),

(337, 'Nasr City East Grill', 'restaurant', 'grilled_food', 20, 15, 7, 145, 31000, 25000, 5600, '2016-11-09'),
(337, 'Nasr City East Electronics', 'retail', 'electronics', 12, 11, 6, 110, 33000, 18000, 3800, '2021-05-18'),
(337, 'Nasr City East Repair Center', 'service', 'general_repair', 15, 10, 6, 80, 27000, 14500, 3000, '2018-08-04'),
(337, 'Nasr City East Coffee Stop', 'cafe', 'coffee_beverages', 7, 12, 7, 90, 30000, 15000, 3300, '2022-06-02'),
(33, 'Ain Shams Market Fruits', 'retail', 'fruits_vegetables', 23, 13, 7, 140, 16500, 19000, 3800, '2014-05-17'),

(33, 'Ain Shams Market Meat', 'retail', 'meat_products', 30, 12, 7, 100, 15500, 20500, 4000, '2010-12-02'),
(33, 'Ain Shams Auto Service', 'service', 'auto_repair', 25, 10, 6, 150, 20000, 23000, 4900, '2013-09-28'),
(33, 'Ain Shams Mini Cafe', 'cafe', 'coffee_beverages', 7, 11, 7, 60, 14500, 11000, 2600, '2020-08-15'),
(33, 'Ain Shams Grocery Express', 'grocery', 'food_items', 13, 14, 7, 110, 16000, 17500, 3500, '2019-01-31'),
(31, 'El Matareya Market Fruits', 'retail', 'fruits_vegetables', 22, 13, 7, 135, 16000, 18500, 3700, '2014-03-09'),

(31, 'El Matareya Market Meat', 'retail', 'meat_products', 29, 12, 7, 95, 15000, 20000, 3900, '2011-05-27'),
(31, 'El Matareya Auto Service', 'service', 'auto_repair', 24, 10, 6, 145, 19500, 22500, 4800, '2013-11-13'),
(31, 'El Matareya Mini Cafe', 'cafe', 'coffee_beverages', 6, 11, 7, 55, 14000, 10500, 2400, '2020-09-24'),
(31, 'El Matareya Grocery Express', 'grocery', 'food_items', 12, 14, 7, 105, 15500, 17000, 3400, '2019-02-19'),
(34, 'Ain Shams Night Cafe', 'cafe', 'coffee_beverages', 8, 16, 7, 95, 17500, 16500, 5200, '2020-10-11'),

(34, 'Ain Shams Late Grill', 'restaurant', 'grilled_food', 22, 16, 7, 165, 18500, 31000, 6600, '2015-12-06'),
(34, 'Ain Shams Mini Grocery', 'grocery', 'food_items', 10, 13, 7, 105, 15000, 17000, 3300, '2021-02-09'),
(34, 'Ain Shams Phone Accessories', 'retail', 'mobile_accessories', 6, 11, 6, 55, 14500, 9500, 2200, '2022-04-18'),
(34, 'Ain Shams Coffee Kiosk', 'cafe', 'coffee_beverages', 5, 11, 7, 45, 14000, 9000, 2100, '2023-03-02'),
(32, 'El Matareya Night Cafe', 'cafe', 'coffee_beverages', 7, 16, 7, 90, 16500, 15500, 5000, '2020-09-29'),

(32, 'El Matareya Late Grill', 'restaurant', 'grilled_food', 21, 16, 7, 155, 17500, 30000, 6400, '2015-11-19'),
(32, 'El Matareya Mini Grocery', 'grocery', 'food_items', 9, 13, 7, 95, 14500, 15500, 3000, '2021-01-22'),
(32, 'El Matareya Phone Accessories', 'retail', 'mobile_accessories', 5, 11, 6, 50, 14000, 9000, 2100, '2022-06-06'),
(32, 'El Matareya Coffee Kiosk', 'cafe', 'coffee_beverages', 4, 11, 7, 40, 13500, 8500, 2000, '2023-01-25'),
(26, 'Heliopolis Fringe Auto Parts', 'retail', 'auto_parts', 20, 11, 6, 120, 26000, 18000, 3600, '2016-08-17'),

(27, 'Heliopolis Fringe Tyres Shop', 'retail', 'auto_parts', 22, 11, 6, 125, 27000, 18500, 3700, '2015-10-29'),
(28, 'Heliopolis Fringe Garage', 'service', 'auto_repair', 26, 10, 6, 170, 30000, 25000, 5200, '2012-06-13'),
(29, 'Heliopolis Fringe Fuel Snacks', 'restaurant', 'snacks', 6, 12, 7, 55, 25000, 12000, 2900, '2022-09-01'),
(30, 'Heliopolis Fringe Grocery Express', 'grocery', 'food_items', 11, 14, 7, 105, 24500, 17000, 3400, '2019-07-07');

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Business_Profile inserted successfully ✅';
END $$;
