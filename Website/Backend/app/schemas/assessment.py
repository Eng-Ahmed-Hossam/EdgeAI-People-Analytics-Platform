"""
Assessment request / response schemas.

Serialisation convention (matches the rest of the backend):
  - Request bodies accept camelCase JSON (alias_generator=to_camel + populate_by_name=True)
  - Response bodies emit camelCase JSON (serialize_by_alias=True)
  - Python attributes are snake_case throughout

FeasibilityResponse field names must exactly match the FeasibilityOutput TypeScript
interface in frontend/src/types/assessment-v2.ts.
"""

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel


# ---------------------------------------------------------------------------
# Request schemas (inbound camelCase JSON → snake_case Python attributes)
# ---------------------------------------------------------------------------

class OperatingHoursIn(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    open: str = "09:00"
    close: str = "21:00"
    days_per_week: int = 6


class BusinessParametersIn(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    domain: str
    operating_hours: OperatingHoursIn | None = None
    staff_count: int = 0
    budget: float = 0.0
    currency: str = "EGP"
    additional_notes: str | None = None


class FeasibilityRequest(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    asset_id: str
    concept: str
    parameters: BusinessParametersIn


# ---------------------------------------------------------------------------
# Response schemas (snake_case Python → camelCase JSON)
# ---------------------------------------------------------------------------

class ConfidenceIntervalOut(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, serialize_by_alias=True, populate_by_name=True)

    low: int
    high: int


class ContributingFactorOut(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, serialize_by_alias=True, populate_by_name=True)

    label: str
    value: str
    direction: str  # "positive" | "negative" | "neutral"
    weight: float = Field(ge=0.0, le=1.0)


class FeasibilityResponse(BaseModel):
    """
    Response shape — must match FeasibilityOutput in assessment-v2.ts exactly.

    camelCase aliases produced by serialize_by_alias=True:
      feasibility_score     → feasibilityScore
      success_likelihood    → successLikelihood
      confidence_interval   → confidenceInterval
      score_band            → scoreBand
      contributing_factors  → contributingFactors
      label_quality         → labelQuality
      data_sources          → dataSources
      data_point_count      → dataPointCount
      generated_at          → generatedAt
    """

    model_config = ConfigDict(alias_generator=to_camel, serialize_by_alias=True, populate_by_name=True)

    feasibility_score: int = Field(ge=0, le=100)
    success_likelihood: float = Field(ge=0.0, le=1.0)
    confidence_interval: ConfidenceIntervalOut
    score_band: str  # "strong" | "moderate" | "weak"
    contributing_factors: list[ContributingFactorOut]
    narrative: str
    label_quality: str  # "high" | "medium" | "low"
    data_sources: list[str]
    data_point_count: int
    generated_at: str  # ISO 8601 timestamp


# ---------------------------------------------------------------------------
# Persistence schemas — save / retrieve / list a completed assessment.
#
# A SaveAssessmentRequest carries a completed FeasibilityOutput (the `feasibility`
# field reuses FeasibilityResponse). The backend stamps engine_version and reads
# coverage_score from the asset; the client does not supply those.
#
# A StoredAssessmentResponse returns the persistence metadata plus the
# reassembled FeasibilityOutput in `feasibility` — this MUST match a live engine
# run field-for-field (the round-trip contract).
# ---------------------------------------------------------------------------

class SaveAssessmentRequest(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    asset_id: str
    concept: str
    feasibility: FeasibilityResponse
    # owner_id is NOT accepted from the client — the server stamps current_user.id.


class StoredAssessmentResponse(BaseModel):
    """
    A persisted assessment as returned by GET/POST/list.

    `feasibility` is the reassembled FeasibilityOutput — identical in shape to a
    live engine run. Everything else is persistence metadata.
    """

    model_config = ConfigDict(alias_generator=to_camel, serialize_by_alias=True, populate_by_name=True)

    assessment_id: str
    asset_id: str
    owner_id: str | None
    concept: str
    engine_version: str
    coverage_score: int
    created_at: str  # ISO 8601 — when the record was persisted
    feasibility: FeasibilityResponse
