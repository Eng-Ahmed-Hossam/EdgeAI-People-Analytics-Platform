from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.asset import Asset as AssetModel
from app.schemas.asset import AssetCoordinates, AssetResponse


def asset_to_response(db_asset: AssetModel) -> AssetResponse:
    """Convert an ORM Asset row to the camelCase API response schema."""
    return AssetResponse(
        id=db_asset.id,
        coordinates=AssetCoordinates(lat=db_asset.latitude, lng=db_asset.longitude),
        address=db_asset.address,
        area=db_asset.area,
        size=db_asset.size,
        monthly_rent=db_asset.monthly_rent,
        sale_price=db_asset.sale_price,
        currency=db_asset.currency,
        property_type=db_asset.property_type,
        availability_status=db_asset.availability_status,
        listing_source=db_asset.listing_source,
        coverage_score=db_asset.coverage_score,
    )


async def get_all_assets(db: AsyncSession) -> list[AssetResponse]:
    stmt = (
        select(AssetModel)
        .where(
            AssetModel.is_active.is_(True),
            AssetModel.availability_status.in_(["available", "reserved"]),
        )
        .order_by(AssetModel.coverage_score.desc())
    )
    result = await db.execute(stmt)
    rows = result.scalars().all()
    return [asset_to_response(row) for row in rows]


async def get_asset_by_id(db: AsyncSession, asset_id: str) -> AssetResponse | None:
    stmt = select(AssetModel).where(
        AssetModel.id == asset_id,
        AssetModel.is_active.is_(True),
    )
    result = await db.execute(stmt)
    row = result.scalar_one_or_none()
    if row is None:
        return None
    return asset_to_response(row)
