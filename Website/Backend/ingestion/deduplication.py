"""
Deduplication layer.

Three complementary strategies, applied in this order:

  1. Source + listing ID match (exact)
     — (source, source_listing_id) unique constraint in the DB catches this at
       insert time; we check it here so we can update instead of skip.

  2. Coordinate proximity (spatial)
     — Any existing asset within 50 metres is a potential duplicate.
     — Uses PostGIS ST_DWithin on geography (metres), not geometry (degrees).

  3. Address similarity (fuzzy text)
     — Applied after coordinate proximity as a secondary confirmation.
     — Uses difflib.SequenceMatcher (no extra library dependency).
     — Threshold: 0.75 similarity ratio.

Strategy: if strategy 1 hits → update the existing record (price may have changed).
          If strategy 2+3 hit → skip the new listing (it's a duplicate from another
          source or a re-scraped version with a different listing ID).
          If nothing hits → insert as a new asset.
"""

import logging
from difflib import SequenceMatcher

from geoalchemy2.functions import ST_DWithin, ST_MakePoint, ST_SetSRID
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.asset import Asset as AssetModel
from ingestion.normalizer import NormalizedAsset

logger = logging.getLogger(__name__)

COORDINATE_RADIUS_METRES = 50
ADDRESS_SIMILARITY_THRESHOLD = 0.75


def _address_similarity(a: str, b: str) -> float:
    return SequenceMatcher(None, a.lower().strip(), b.lower().strip()).ratio()


async def find_by_source(
    db: AsyncSession,
    source: str,
    source_listing_id: str,
) -> AssetModel | None:
    stmt = select(AssetModel).where(
        AssetModel.source == source,
        AssetModel.source_listing_id == source_listing_id,
    )
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def find_nearby(
    db: AsyncSession,
    lat: float,
    lng: float,
    radius_metres: int = COORDINATE_RADIUS_METRES,
) -> list[AssetModel]:
    """
    Return all active assets within `radius_metres` of (lat, lng).
    Uses ST_DWithin on geography (true-metre distance, not degree distance).
    """
    # Cast to geography (true-metre distance). PostGIS uses the geography()
    # type constructor — there is no ST_Geography() function in PostGIS 3.x.
    target_geog = func.geography(
        func.ST_SetSRID(func.ST_MakePoint(lng, lat), 4326)
    )
    asset_geog = func.geography(AssetModel.location)

    stmt = select(AssetModel).where(
        AssetModel.is_active.is_(True),
        func.ST_DWithin(asset_geog, target_geog, radius_metres),
    )
    result = await db.execute(stmt)
    return list(result.scalars().all())


async def check_duplicate(
    db: AsyncSession,
    normalized: NormalizedAsset,
) -> tuple[AssetModel | None, str]:
    """
    Check if a NormalizedAsset is a duplicate of an existing record.

    Returns:
        (existing_asset, reason) where reason is one of:
          "same_source"       — exact source+listing_id match → update candidate
          "coordinate_match"  — spatial duplicate from a different source → skip
          "not_duplicate"     — new listing → insert
    """
    # Strategy 1: exact source + listing ID
    existing = await find_by_source(db, normalized.source, normalized.source_listing_id)
    if existing is not None:
        return existing, "same_source"

    # Strategy 2+3: spatial proximity + address similarity
    nearby = await find_nearby(db, normalized.lat, normalized.lng)
    for candidate in nearby:
        similarity = _address_similarity(normalized.address, candidate.address)
        if similarity >= ADDRESS_SIMILARITY_THRESHOLD:
            logger.debug(
                "Coordinate+address duplicate detected: new='%s' matches existing='%s' (%.2f similarity, %s source)",
                normalized.address,
                candidate.address,
                similarity,
                candidate.source,
            )
            return candidate, "coordinate_match"

    return None, "not_duplicate"
