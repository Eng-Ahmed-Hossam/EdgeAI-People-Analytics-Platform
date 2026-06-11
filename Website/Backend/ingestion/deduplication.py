"""
Deduplication layer.

Three complementary strategies, applied in this order:

  1. Source + listing ID match (exact)
  2. Coordinate proximity — Python-side Haversine; any asset within 50 m is a candidate.
  3. Address similarity (fuzzy text) — difflib.SequenceMatcher, threshold 0.75.

Strategy: same_source → update; coordinate_match → skip; not_duplicate → insert.
"""

import logging
import math
from difflib import SequenceMatcher

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.asset import Asset as AssetModel
from ingestion.normalizer import NormalizedAsset

logger = logging.getLogger(__name__)

COORDINATE_RADIUS_METRES = 50
ADDRESS_SIMILARITY_THRESHOLD = 0.75
_EARTH_RADIUS_M = 6_371_000


def _haversine_metres(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lng2 - lng1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))


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
    """Return active assets within radius_metres of (lat, lng) using Haversine."""
    # Bounding-box pre-filter: 50 m ≈ 0.00045° latitude; longitude degree shrinks with cos(lat)
    lat_delta = radius_metres / _EARTH_RADIUS_M * (180 / math.pi)
    lng_delta = lat_delta / max(math.cos(math.radians(lat)), 1e-6)

    stmt = select(AssetModel).where(
        AssetModel.is_active.is_(True),
        AssetModel.latitude.between(lat - lat_delta, lat + lat_delta),
        AssetModel.longitude.between(lng - lng_delta, lng + lng_delta),
    )
    result = await db.execute(stmt)
    candidates = list(result.scalars().all())

    return [
        c for c in candidates
        if _haversine_metres(lat, lng, c.latitude, c.longitude) <= radius_metres
    ]


async def check_duplicate(
    db: AsyncSession,
    normalized: NormalizedAsset,
) -> tuple[AssetModel | None, str]:
    existing = await find_by_source(db, normalized.source, normalized.source_listing_id)
    if existing is not None:
        return existing, "same_source"

    nearby = await find_nearby(db, normalized.lat, normalized.lng)
    for candidate in nearby:
        similarity = _address_similarity(normalized.address, candidate.address)
        if similarity >= ADDRESS_SIMILARITY_THRESHOLD:
            return candidate, "coordinate_match"

    return None, "not_duplicate"
