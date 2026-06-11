"""
Valuation router.

  POST /api/valuation   — run the valuation engine for an asset (+ optional concept)

Protected by get_current_user (requires a valid session cookie): valuation is a
higher-value intelligence output than the unauthenticated feasibility compute, and
the brief specifies it sit behind auth. The asset is fetched from the DB so the
engine reasons against the stored commercial terms and coverage score; the client
cannot influence those.

The engine is Phase 1 (rule-based reasoned priors); it consumes the feasibility
signal as a first-class input. Nothing in this router changes for Phase 2 — only
the service layer (real comparables replace the priors).
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.security import get_current_user
from app.crud.assets import get_asset_by_id
from app.database import get_db
from app.models.user import User
from app.schemas.valuation import ValuationResponse
from app.schemas.valuation import ValuationRequest
from app.services.valuation import compute_valuation

router = APIRouter(prefix="/valuation", tags=["valuation"])


@router.post("", response_model=ValuationResponse)
async def get_valuation(
    body: ValuationRequest,
    db: AsyncSession = Depends(get_db),
    _current_user: User = Depends(get_current_user),
) -> ValuationResponse:
    """
    Compute a valuation for the given asset (and optional business concept).

    Mode is chosen by the engine from the asset's listed price:
      · has monthly_rent / sale_price → Mode A (listing_evaluation)
      · no listed price               → Mode B (opportunity_discovery)

    Returns a ValuationResponse whose camelCase JSON matches the extended
    ValuationOutput TypeScript interface exactly.
    """
    asset = await get_asset_by_id(db, body.asset_id)
    if asset is None:
        raise HTTPException(status_code=404, detail=f"Asset '{body.asset_id}' not found.")

    result = compute_valuation(
        asset_id=asset.id,
        area=asset.area,
        size=asset.size,
        monthly_rent=asset.monthly_rent,
        sale_price=asset.sale_price,
        property_type=asset.property_type,
        coverage_score=asset.coverage_score,
        currency=asset.currency,
        concept=body.concept,
    )

    return ValuationResponse(
        mode=result["mode"],
        verdict=result["verdict"],
        fair_price_estimate=result["fair_price_estimate"],
        fair_value_range=result["fair_value_range"],
        expected_value=result["expected_value"],
        currency=result["currency"],
        confidence=result["confidence"],
        price_delta=result["price_delta"],
        valuation_gap_pct=result["valuation_gap_pct"],
        drivers=result["drivers"],
        valuation_narrative=result["valuation_narrative"],
        generated_at=result["generated_at"],
    )
