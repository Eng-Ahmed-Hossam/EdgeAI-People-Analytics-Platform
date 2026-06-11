"""
SyncJob — orchestrates one full run of a source connector through the pipeline.

Pipeline:
  Source Connector  →  Normalizer  →  Deduplication  →  Assets Table
  ^^^^^^^^^^^^^^^^     ^^^^^^^^^^     ^^^^^^^^^^^^^^     ^^^^^^^^^^^^
  yields RawListing    NormalizedAsset  check+strategy   upsert / skip

Asset lifecycle management:
  - same_source hit  → UPDATE (price/status may have changed)
  - coordinate_match → SKIP (spatial+address duplicate from another source)
  - not_duplicate    → INSERT as new asset

The SyncJob knows nothing about the connector's internals and nothing about
what HTTP calls or scraping happened. It only sees NormalizedAsset objects.
"""

import logging
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone

from geoalchemy2.elements import WKTElement
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.asset import Asset as AssetModel
from ingestion.base import BaseSourceConnector
from ingestion.deduplication import check_duplicate
from ingestion.normalizer import ListingNormalizer, NormalizedAsset

logger = logging.getLogger(__name__)


@dataclass
class SyncReport:
    source: str
    started_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    finished_at: datetime | None = None
    inserted: int = 0
    updated: int = 0
    skipped: int = 0
    errors: int = 0

    @property
    def total_processed(self) -> int:
        return self.inserted + self.updated + self.skipped + self.errors

    def finish(self) -> None:
        self.finished_at = datetime.now(timezone.utc)

    def __str__(self) -> str:
        duration = (
            f"{(self.finished_at - self.started_at).total_seconds():.1f}s"
            if self.finished_at
            else "in progress"
        )
        return (
            f"[{self.source}] {duration} | "
            f"inserted={self.inserted} updated={self.updated} "
            f"skipped={self.skipped} errors={self.errors}"
        )


def _make_point(lat: float, lng: float) -> WKTElement:
    """Create a PostGIS WKT geometry from lat/lng. WKT order is POINT(lng lat)."""
    return WKTElement(f"POINT({lng} {lat})", srid=4326)


def _generate_id(normalized: NormalizedAsset) -> str:
    """
    Generate a stable asset ID.
    Seeded assets carry their own ID in source_listing_id.
    Other sources get a deterministic UUID based on (source, source_listing_id).
    """
    if normalized.source == "seeded":
        return normalized.source_listing_id
    # Deterministic UUID-5 (namespace + name) — same input always produces same ID
    ns = uuid.UUID("6ba7b810-9dad-11d1-80b4-00c04fd430c8")  # URL namespace
    return str(uuid.uuid5(ns, f"{normalized.source}:{normalized.source_listing_id}"))


class SyncJob:
    def __init__(self, db: AsyncSession, connector: BaseSourceConnector) -> None:
        self._db = db
        self._connector = connector
        self._normalizer = ListingNormalizer()

    async def run(self) -> SyncReport:
        report = SyncReport(source=self._connector.source_name)
        logger.info("SyncJob starting for source: %s", self._connector.source_name)

        async for raw_listing in self._connector.fetch_listings():
            try:
                normalized = self._normalizer.normalize(raw_listing)
                if normalized is None:
                    report.skipped += 1
                    continue

                existing, reason = await check_duplicate(self._db, normalized)

                if reason == "same_source" and existing is not None:
                    await self._update_asset(existing, normalized)
                    report.updated += 1

                elif reason == "coordinate_match":
                    logger.debug("Skipping spatial duplicate: %s", normalized.address)
                    report.skipped += 1

                else:  # not_duplicate
                    await self._insert_asset(normalized)
                    report.inserted += 1

            except Exception as exc:
                logger.error(
                    "Error processing listing %s/%s: %s",
                    raw_listing.source,
                    raw_listing.source_listing_id,
                    exc,
                )
                report.errors += 1

        await self._db.flush()
        report.finish()
        logger.info("SyncJob complete: %s", report)
        return report

    async def _insert_asset(self, normalized: NormalizedAsset) -> AssetModel:
        now = datetime.now(timezone.utc)
        asset = AssetModel(
            id=_generate_id(normalized),
            location=_make_point(normalized.lat, normalized.lng),
            address=normalized.address,
            area=normalized.area,
            size=normalized.size,
            monthly_rent=normalized.monthly_rent,
            sale_price=normalized.sale_price,
            currency=normalized.currency,
            property_type=normalized.property_type,
            availability_status=normalized.availability_status,
            listing_source=normalized.listing_source,
            coverage_score=normalized.coverage_score,
            source=normalized.source,
            source_listing_id=normalized.source_listing_id,
            source_url=normalized.source_url,
            first_seen_at=now,
            last_seen_at=now,
            ingestion_timestamp=now,
            is_active=True,
        )
        self._db.add(asset)
        logger.debug("Inserted asset: %s (%s)", asset.id, normalized.address)
        return asset

    async def _update_asset(self, existing: AssetModel, normalized: NormalizedAsset) -> None:
        now = datetime.now(timezone.utc)
        # Only update fields that legitimately change between listing refreshes
        existing.monthly_rent = normalized.monthly_rent
        existing.sale_price = normalized.sale_price
        existing.availability_status = normalized.availability_status
        existing.size = normalized.size or existing.size
        existing.last_seen_at = now
        existing.ingestion_timestamp = now
        logger.debug("Updated asset: %s (%s)", existing.id, existing.address)
