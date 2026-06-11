-- ============================================================
-- Aggregated Data: operational_efficiency
-- Purpose:
--   Compute daily operational efficiency indicators at the
--   business level by combining revenue performance with
--   normalized fixed operating costs.
--
-- Description:
--   This INSERT-SELECT derives cost ratios and an estimated
--   profit margin by allocating monthly operating expenses
--   (staff, rent, utilities) to a daily basis and comparing
--   them against actual daily revenue.
--
-- Metrics Calculated:
--   • staff_cost_ratio
--       → (Daily staff cost) / (Daily revenue)
--       → Indicates labor cost efficiency
--
--   • rent_cost_ratio
--       → (Daily rent cost) / (Daily revenue)
--       → Measures location and fixed-asset burden
--
--   • profit_margin_est
--       → (Revenue − daily staff − daily rent − daily utilities)
--         / Revenue
--       → Approximated daily operating margin
--
-- Analytical Notes:
--   - Monthly costs are normalized using a 30-day convention
--     for consistency across months.
--   - NULLIF safeguards prevent division-by-zero errors on
--     zero-revenue days.
--   - Rounded to 4 decimals for stable downstream analytics
--     and dashboard presentation.
--
-- Intended Use:
--   - Daily profitability monitoring
--   - Cost-structure benchmarking across businesses
--   - Feature generation for profit and risk prediction models
--   - Decision support for pricing, staffing, and expansion
--
-- Assumptions:
--   • Costs are evenly distributed across days of the month
--   • Revenue figures are net daily totals
--   • Margin is an estimate (excludes COGS and taxes)
-- ============================================================


TRUNCATE TABLE operational_efficiency RESTART IDENTITY CASCADE;

INSERT INTO operational_efficiency
(business_id, date, staff_cost_ratio, rent_cost_ratio, profit_margin_est)
SELECT
  dr.business_id,
  dr.date,

  ROUND((bp.staff_cost_monthly / 30.0) / NULLIF(dr.total_revenue,0), 4)
    AS staff_cost_ratio,

  ROUND((bp.rent_monthly / 30.0) / NULLIF(dr.total_revenue,0), 4)
    AS rent_cost_ratio,

  ROUND(
    (
      dr.total_revenue
      - (bp.staff_cost_monthly / 30.0)
      - (bp.rent_monthly / 30.0)
      - (bp.utilities_cost_monthly / 30.0)
    ) / NULLIF(dr.total_revenue,0),
    4
  ) AS profit_margin_est

FROM Daily_Revenue dr
JOIN Business_Profile bp
  ON dr.business_id = bp.business_id;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Operational Efficiency table inserted successfully ✅';
END $$;