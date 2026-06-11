"""
Assessments router.

Compute:
  POST   /api/assessments/feasibility   — run the rule engine, return a result (unauthenticated)

Persistence (all require a valid JWT):
  POST   /api/assessments               — save a completed assessment; owner = current_user
  GET    /api/assessments               — list current user's assessments (filter by assetId)
  GET    /api/assessments/{id}          — retrieve one (404 if absent or owned by another user)
  DELETE /api/assessments/{id}          — delete one (404 if absent or owned by another user)

Ownership policy (Auth Session 3):
  - Every saved assessment is stamped with current_user.id server-side. The
    client never supplies owner_id — it is ignored even if sent.
  - List, retrieve, and delete are scoped to current_user.id. A record owned by
    another user returns 404 (not 403 — do not confirm the record exists).
  - 'local-dev' records from the pre-auth phase remain in the database as
    orphaned rows. No real user has owner_id='local-dev' so they never appear
    in any list or GET. They are not deleted — see crud/assessments.py.

The engine is Phase 1 (rule-based reasoned priors). Phase 2 will replace the
weight constants when sufficient labelled outcome data exists. Nothing in this
router changes for Phase 2 — only the service layer.
"""

from fastapi import APIRouter, Depends, HTTPException, Query, Response, status
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.security import get_current_user
from app.crud.assessments import (
    assessment_to_response,
    delete_assessment,
    get_assessment_by_id,
    list_assessments,
    save_assessment,
)
from app.crud.assets import get_asset_by_id
from app.database import get_db
from app.models.user import User
from app.schemas.assessment import (
    FeasibilityRequest,
    FeasibilityResponse,
    SaveAssessmentRequest,
    StoredAssessmentResponse,
)
from app.services.feasibility import ENGINE_VERSION, compute_feasibility

router = APIRouter(prefix="/assessments", tags=["assessments"])


@router.post("/feasibility", response_model=FeasibilityResponse)
async def get_feasibility(
    body: FeasibilityRequest,
    db: AsyncSession = Depends(get_db),
) -> FeasibilityResponse:
    """
    Compute a feasibility score for the given asset + business concept.

    Returns a FeasibilityResponse whose camelCase JSON keys match the
    FeasibilityOutput TypeScript interface exactly.
    """
    asset = await get_asset_by_id(db, body.asset_id)
    if asset is None:
        raise HTTPException(status_code=404, detail=f"Asset '{body.asset_id}' not found.")

    result = compute_feasibility(
        asset_id=asset.id,
        area=asset.area,
        size=asset.size,
        monthly_rent=asset.monthly_rent,
        property_type=asset.property_type,
        coverage_score=asset.coverage_score,
        concept=body.concept,
    )

    return FeasibilityResponse(
        feasibility_score=result["feasibility_score"],
        success_likelihood=result["success_likelihood"],
        confidence_interval=result["confidence_interval"],
        score_band=result["score_band"],
        contributing_factors=result["contributing_factors"],
        narrative=result["narrative"],
        label_quality=result["label_quality"],
        data_sources=result["data_sources"],
        data_point_count=result["data_point_count"],
        generated_at=result["generated_at"],
    )


# ─────────────────────────────────────────────────────────────────────────────
# Persistence endpoints
# ─────────────────────────────────────────────────────────────────────────────

@router.post("", response_model=StoredAssessmentResponse, status_code=status.HTTP_201_CREATED)
async def create_assessment(
    body: SaveAssessmentRequest,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user),
) -> StoredAssessmentResponse:
    """
    Save a completed feasibility assessment. Requires a valid JWT.

    owner_id is always set to current_user.id server-side — the client cannot
    influence it. The asset must exist (FK provides coverage_score).
    engine_version is stamped server-side.
    """
    asset = await get_asset_by_id(db, body.asset_id)
    if asset is None:
        raise HTTPException(status_code=404, detail=f"Asset '{body.asset_id}' not found.")

    row = await save_assessment(
        db,
        asset_id=body.asset_id,
        concept=body.concept,
        feasibility=body.feasibility,
        coverage_score=asset.coverage_score,
        engine_version=ENGINE_VERSION,
        owner_id=current_user.id,
    )
    return assessment_to_response(row)


@router.get("", response_model=list[StoredAssessmentResponse])
async def list_saved_assessments(
    asset_id: str | None = Query(default=None, alias="assetId"),
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user),
) -> list[StoredAssessmentResponse]:
    """
    List the current user's persisted assessments, newest first. Requires a valid JWT.

    Optionally filter by assetId. Results are always scoped to current_user —
    a user never sees another user's assessments. 'local-dev' orphan records
    from the pre-auth phase are excluded automatically.
    """
    rows = await list_assessments(db, asset_id=asset_id, owner_id=current_user.id)
    return [assessment_to_response(r) for r in rows]


@router.get("/{assessment_id}", response_model=StoredAssessmentResponse)
async def retrieve_assessment(
    assessment_id: str,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user),
) -> StoredAssessmentResponse:
    """
    Retrieve one assessment by id. Requires a valid JWT.

    404 if the assessment doesn't exist OR belongs to a different user — the
    caller is not told which case applied (no-enumeration principle).
    """
    row = await get_assessment_by_id(db, assessment_id, owner_id=current_user.id)
    if row is None:
        raise HTTPException(status_code=404, detail=f"Assessment '{assessment_id}' not found.")
    return assessment_to_response(row)


@router.delete("/{assessment_id}", status_code=status.HTTP_204_NO_CONTENT)
async def remove_assessment(
    assessment_id: str,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user),
) -> Response:
    """
    Delete one assessment by id. Requires a valid JWT.

    404 if absent or owned by a different user. 204 on success.
    """
    deleted = await delete_assessment(db, assessment_id, owner_id=current_user.id)
    if not deleted:
        raise HTTPException(status_code=404, detail=f"Assessment '{assessment_id}' not found.")
    return Response(status_code=status.HTTP_204_NO_CONTENT)
