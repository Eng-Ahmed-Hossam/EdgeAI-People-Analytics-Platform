-- ============================================================
-- Prediction_Output
-- Purpose:
--   Refresh and populate the Prediction_Output table with
--   model-generated revenue forecasts, derived profit
--   estimates, and composite feasibility scores for
--   business-level decision support.
--
-- Description:
--   This script clears prior prediction results, joins the
--   feature store with ML inference outputs, operational
--   efficiency metrics, and behavioral trends, then computes
--   interpretable KPIs used for feasibility and confidence
--   assessment.
--
-- Key Outputs:
--   • predicted_revenue   → ML-predicted daily revenue
--   • predicted_profit    → Revenue × estimated profit margin
--   • feasibility_score   → Weighted composite score (0–100)
--   • confidence_score    → Model-provided confidence metric
--   • model_version       → Inference model identifier
--
-- Feasibility Score Composition:
--   - 40% Revenue normalization (market potential)
--   - 30% Estimated profit margin (financial health)
--   - 20% Traffic stability (1 − volatility index)
--   - 10% Promotion/Event uplift signal
-- ============================================================

TRUNCATE TABLE Prediction_Output RESTART IDENTITY CASCADE;

INSERT INTO Prediction_Output
(business_id, location_id, prediction_date,
 predicted_revenue, predicted_profit,
 feasibility_score, confidence_score, model_version)
SELECT
  fs.business_id,
  fs.location_id,
  fs.date AS prediction_date,

  -- Predicted revenue from ML service (example join)
  pr.predicted_revenue,

  -- Predicted profit derived using estimated margin
  ROUND(
    pr.predicted_revenue * oe.profit_margin_est,
    2
  ) AS predicted_profit,

  -- Feasibility score (0–100)
  ROUND(
    LEAST(
      100,
      GREATEST(
        0,
        (
          0.40 * pr.revenue_norm
        + 0.30 * oe.profit_margin_est
        + 0.20 * (1 - bt.traffic_volatility_index)
        + 0.10 * COALESCE(pr.promo_event_boost, 0)
        ) * 100
      )
    ),
    2
  ) AS feasibility_score,

  pr.confidence_score,
  pr.model_version

FROM feature_store fs

-- Mocked prediction results table from ML service
JOIN ml_revenue_predictions pr
  ON pr.feature_vector_hash = fs.feature_vector_hash

JOIN operational_efficiency oe
  ON oe.business_id = fs.business_id
 AND oe.date = fs.date

LEFT JOIN behavior_trends bt
  ON bt.location_id = fs.location_id
 AND bt.date = fs.date;

-- ==========================================
-- ✅ Confirmation Message
-- ==========================================
DO $$
BEGIN
    RAISE NOTICE 'Prediction Output inserted successfully ✅';
END $$;