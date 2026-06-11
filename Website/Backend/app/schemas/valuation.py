"""
Valuation request / response schemas.

Serialisation convention (matches the rest of the backend):
  - Request bodies accept camelCase JSON (alias_generator=to_camel + populate_by_name=True)
  - Response bodies emit camelCase JSON (serialize_by_alias=True)
  - Python attributes are snake_case throughout

ValuationResponse field names must exactly match the (extended) ValuationOutput
TypeScript interface in frontend/src/types/assessment-v2.ts. The `mode` field
distinguishes the two output types so the frontend renders them differently:
  - "listing_evaluation"   → Mode A (price fairness): priceDelta + valuationGapPct present
  - "opportunity_discovery"→ Mode B (commercial potential): priceDelta + valuationGapPct null
"""

from pydantic import BaseModel, ConfigDict, Field


# ---------------------------------------------------------------------------
# Request (inbound camelCase JSON → snake_case Python attributes)
# ---------------------------------------------------------------------------

class ValuationRequest(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    asset_id: str = Field(alias="assetId")
    # Optional: when supplied the feasibility signal is concept-specific (richest);
    # when omitted the engine runs a generic proxy at slightly reduced confidence.
    concept: str | None = None


# ---------------------------------------------------------------------------
# Response (snake_case Python → camelCase JSON)
# ---------------------------------------------------------------------------

class ValuationRangeOut(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    low: int
    high: int


class ValuationDriverOut(BaseModel):
    model_config = ConfigDict(populate_by_name=True)

    label: str
    detail: str
    direction: str  # "positive" | "negative" | "neutral"


class ValuationResponse(BaseModel):
    """
    Response shape — must match the extended ValuationOutput in assessment-v2.ts.

    Field aliases (camelCase JSON):
      mode                → mode
      verdict             → verdict
      fair_price_estimate → fairPriceEstimate
      fair_value_range    → fairValueRange
      expected_value      → expectedValue
      currency            → currency
      confidence          → confidence
      price_delta         → priceDelta            (null in Mode B)
      valuation_gap_pct   → valuationGapPct       (null in Mode B)
      drivers             → drivers
      valuation_narrative → valuationNarrative
      generated_at        → generatedAt

    Explicit per-field aliases (not alias_generator) so the null-able fields and
    the nested models serialise predictably. populate_by_name=True lets the router
    construct with snake_case kwargs (Session H lesson).
    """

    model_config = ConfigDict(populate_by_name=True, serialize_by_alias=True)

    mode: str = Field(alias="mode")  # "listing_evaluation" | "opportunity_discovery"
    verdict: str = Field(alias="verdict")
    fair_price_estimate: int = Field(alias="fairPriceEstimate")
    fair_value_range: ValuationRangeOut = Field(alias="fairValueRange")
    expected_value: int = Field(alias="expectedValue")
    currency: str = Field(alias="currency")
    confidence: str = Field(alias="confidence")  # "high" | "medium" | "low"
    price_delta: str | None = Field(default=None, alias="priceDelta")
    valuation_gap_pct: float | None = Field(default=None, alias="valuationGapPct")
    drivers: list[ValuationDriverOut] = Field(alias="drivers")
    valuation_narrative: str = Field(alias="valuationNarrative")
    generated_at: str = Field(alias="generatedAt")
