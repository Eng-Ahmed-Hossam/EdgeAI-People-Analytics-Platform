from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from app.crud.assets import get_all_assets, get_asset_by_id
from app.database import get_db
from app.schemas.asset import AssetResponse

router = APIRouter(prefix="/assets", tags=["assets"])


@router.get(
    "",
    response_model=list[AssetResponse],
    summary="List all market-visible assets",
    description=(
        "Returns all active assets with availability_status of 'available' or 'reserved', "
        "ordered by coverage_score descending. This is the primary endpoint consumed by "
        "the frontend map workspace."
    ),
)
async def list_assets(db: AsyncSession = Depends(get_db)) -> list[AssetResponse]:
    return await get_all_assets(db)


@router.get(
    "/{asset_id}",
    response_model=AssetResponse,
    summary="Get a single asset by ID",
    responses={
        status.HTTP_404_NOT_FOUND: {"description": "Asset not found or inactive"},
    },
)
async def get_asset(
    asset_id: str,
    db: AsyncSession = Depends(get_db),
) -> AssetResponse:
    asset = await get_asset_by_id(db, asset_id)
    if asset is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Asset '{asset_id}' not found.",
        )
    return asset
