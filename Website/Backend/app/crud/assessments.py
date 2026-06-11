"""
Assessment persistence — repository layer.

All DB logic for persisted assessments lives here; the router stays thin.

Serialisation strategy (see app/models/assessment.py for the rationale):
  - `feasibility_json` stores the COMPLETE FeasibilityOutput verbatim (camelCase).
    It is the authoritative source for round-trip reconstruction.
  - drivers_json / risks_json / explanation_json and the scalar columns are a
    denormalised, queryable projection written at save time. Records are
    immutable, so the projection cannot drift from `feasibility_json`.

Round-trip guarantee: assessment_to_response() reconstructs the FeasibilityOutput
from `feasibility_json` ONLY, so the returned shape is byte-for-byte the engine's
output (including contributingFactors order, neutral factors, and every scalar).
"""

import uuid
from datetime import datetime

from sqlalchemy import delete as sa_delete
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.assessment import Assessment as AssessmentModel
from app.schemas.assessment import (
    FeasibilityResponse,
    StoredAssessmentResponse,
)

# 'local-dev' records written before Auth Session 3 remain in the database as
# orphaned rows. They are excluded from all user-scoped list/get/delete queries
# because no real user has owner_id == 'local-dev'. They are intentionally NOT
# migrated or deleted — leaving them untouched is the safest choice.
_LEGACY_OWNER_ID = "local-dev"  # kept for reference only; no code path sets this now


def _iso(dt: datetime | None) -> str | None:
    if dt is None:
        return None
    return dt.isoformat()


def _decompose(feasibility: FeasibilityResponse) -> tuple[list, list, dict]:
    """
    Split a FeasibilityOutput into the requested queryable projection.

    Returns (drivers_json, risks_json, explanation_json):
      drivers      — contributing factors with direction == "positive"
      risks        — contributing factors with direction == "negative"
      explanation  — { narrative, dataSources, neutralFactors }

    NOTE: this projection is for querying/semantics only. It is NEVER used to
    reconstruct the FeasibilityOutput (that comes from feasibility_json), so the
    by-direction split here cannot break contributingFactors ordering.
    """
    factors = [f.model_dump(by_alias=True) for f in feasibility.contributing_factors]
    drivers = [f for f in factors if f["direction"] == "positive"]
    risks = [f for f in factors if f["direction"] == "negative"]
    neutral = [f for f in factors if f["direction"] == "neutral"]
    explanation = {
        "narrative": feasibility.narrative,
        "dataSources": list(feasibility.data_sources),
        "neutralFactors": neutral,
    }
    return drivers, risks, explanation


async def save_assessment(
    db: AsyncSession,
    *,
    asset_id: str,
    concept: str,
    feasibility: FeasibilityResponse,
    coverage_score: int,
    engine_version: str,
    owner_id: str,
) -> AssessmentModel:
    """
    Persist one completed feasibility assessment. Returns the stored ORM row.

    owner_id is required and must be the authenticated user's real UUID — the
    router always passes current_user.id. No fallback to 'local-dev'.

    The complete FeasibilityOutput is stored verbatim in feasibility_json;
    scalar and decomposed columns are derived from it for querying.
    """
    drivers, risks, explanation = _decompose(feasibility)

    row = AssessmentModel(
        id=str(uuid.uuid4()),
        asset_id=asset_id,
        owner_id=owner_id,
        concept_type=concept,
        engine_version=engine_version,
        # Denormalised scalars
        score=feasibility.feasibility_score,
        success_likelihood=feasibility.success_likelihood,
        score_band=feasibility.score_band,
        confidence=feasibility.label_quality,
        confidence_range_low=feasibility.confidence_interval.low,
        confidence_range_high=feasibility.confidence_interval.high,
        coverage_score=coverage_score,
        data_point_count=feasibility.data_point_count,
        # Structured JSON projection
        drivers_json=drivers,
        risks_json=risks,
        explanation_json=explanation,
        # Authoritative complete output (round-trip source)
        feasibility_json=feasibility.model_dump(by_alias=True),
        # Engine timestamp (created_at is set by the DB server_default)
        generated_at=_parse_iso(feasibility.generated_at),
    )
    db.add(row)
    await db.flush()      # populate server defaults (created_at) without ending the txn
    await db.refresh(row)
    return row


def _parse_iso(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        # Accept trailing "Z"
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


async def get_assessment_by_id(
    db: AsyncSession,
    assessment_id: str,
    *,
    owner_id: str,
) -> AssessmentModel | None:
    """
    Retrieve one assessment by id, scoped to owner_id.

    Returns None (→ 404) if the assessment doesn't exist OR belongs to a
    different owner. This prevents confirming the record's existence to
    unauthorized callers (same no-enumeration principle as auth).
    """
    stmt = (
        select(AssessmentModel)
        .where(AssessmentModel.id == assessment_id)
        .where(AssessmentModel.owner_id == owner_id)
    )
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def list_assessments(
    db: AsyncSession,
    *,
    asset_id: str | None = None,
    owner_id: str | None = None,
) -> list[AssessmentModel]:
    """
    List assessments, newest first. Optional filters by asset_id and/or owner_id
    support both the "history" view (by owner) and the "asset-grouped" view.
    """
    stmt = select(AssessmentModel)
    if asset_id is not None:
        stmt = stmt.where(AssessmentModel.asset_id == asset_id)
    if owner_id is not None:
        stmt = stmt.where(AssessmentModel.owner_id == owner_id)
    stmt = stmt.order_by(AssessmentModel.created_at.desc())
    result = await db.execute(stmt)
    return list(result.scalars().all())


async def delete_assessment(
    db: AsyncSession,
    assessment_id: str,
    *,
    owner_id: str,
) -> bool:
    """
    Delete by id, scoped to owner_id.

    Returns True if a row was deleted, False if none matched (id absent OR
    owned by a different user). The router returns 404 in both cases — do not
    reveal whether the record exists to an unauthorized caller.
    """
    stmt = (
        sa_delete(AssessmentModel)
        .where(AssessmentModel.id == assessment_id)
        .where(AssessmentModel.owner_id == owner_id)
    )
    result = await db.execute(stmt)
    return (result.rowcount or 0) > 0


def assessment_to_response(row: AssessmentModel) -> StoredAssessmentResponse:
    """
    Reassemble a stored row into the API response.

    The FeasibilityOutput is reconstructed from feasibility_json ONLY, via
    model_validate — guaranteeing the returned shape matches a live engine run
    field-for-field (camelCase keys accepted by populate_by_name).
    """
    feasibility = FeasibilityResponse.model_validate(row.feasibility_json)
    return StoredAssessmentResponse(
        assessment_id=row.id,
        asset_id=row.asset_id,
        owner_id=row.owner_id,
        concept=row.concept_type,
        engine_version=row.engine_version,
        coverage_score=row.coverage_score,
        created_at=_iso(row.created_at) or "",
        feasibility=feasibility,
    )
