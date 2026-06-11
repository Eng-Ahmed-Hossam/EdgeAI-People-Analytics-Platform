"""
CLI: run the asset ingestion pipeline.

Usage:
  python run_ingestion.py                        # seeded only (safe, always works)
  python run_ingestion.py --all                  # seeded + all live connectors
  python run_ingestion.py --source openstreetmap # specific connector only
  python run_ingestion.py --dry-run              # parse + normalise without writing to DB

Run from the backend/ directory:
  cd backend && python run_ingestion.py

Registered connectors:
  seeded          — pre-verified fixture assets (offline, always safe)
  openstreetmap   — commercial POIs from OpenStreetMap via Overpass API
                    (one bulk HTTP request; ODbL-compliant; no web scraping)

Adding a new connector:
  1. Create ingestion/sources/<name>.py implementing BaseSourceConnector.
  2. Add a normalizer method in ingestion/normalizer.py.
  3. Register the connector in CONNECTORS below.
  No other files change.
"""

import argparse
import asyncio
import logging
import sys

from app.config import settings
from app.database import AsyncSessionLocal
from ingestion.jobs.sync import SyncJob
from ingestion.sources.openstreetmap import OpenStreetMapConnector
from ingestion.sources.seeded import SeededConnector

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("run_ingestion")


CONNECTORS = {
    "seeded": lambda: SeededConnector(seed_path=settings.seed_data_path),
    "openstreetmap": lambda: OpenStreetMapConnector(
        user_agent=settings.geocoding_user_agent,
    ),
    # Future connectors registered here:
    # "aqarmap":         lambda: AqarmapConnector(...),
    # "property_finder": lambda: PropertyFinderConnector(...),
    # "partner_xyz":     lambda: PartnerXYZConnector(...),
}


async def run(sources: list[str], dry_run: bool) -> None:
    reports = []

    async with AsyncSessionLocal() as db:
        for source_key in sources:
            factory = CONNECTORS.get(source_key)
            if factory is None:
                logger.error(
                    "Unknown source: '%s'. Available: %s",
                    source_key,
                    list(CONNECTORS),
                )
                continue

            connector = factory()
            logger.info("Starting sync: %s (dry_run=%s)", source_key, dry_run)

            if dry_run:
                # Parse and normalise only; print results without writing to DB
                from ingestion.normalizer import ListingNormalizer
                normalizer = ListingNormalizer()
                count = 0
                async for raw in connector.fetch_listings():
                    normalized = normalizer.normalize(raw)
                    if normalized:
                        logger.info(
                            "  DRY-RUN | %-40s | %.4f,%.4f | rent=%-8s | type=%s",
                            normalized.address[:40],
                            normalized.lat,
                            normalized.lng,
                            str(normalized.monthly_rent),
                            normalized.property_type,
                        )
                        count += 1
                logger.info("DRY-RUN complete: %d listings normalised (not written)", count)
            else:
                job = SyncJob(db, connector)
                report = await job.run()
                await db.commit()
                reports.append(report)
                print(f"  OK {report}")

    if reports:
        total_inserted = sum(r.inserted for r in reports)
        total_updated = sum(r.updated for r in reports)
        total_skipped = sum(r.skipped for r in reports)
        print(
            f"\nSync complete — "
            f"inserted={total_inserted} updated={total_updated} skipped={total_skipped}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run EdgeAI asset ingestion pipeline")
    parser.add_argument(
        "--source",
        choices=list(CONNECTORS),
        help="Run a specific source connector only",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run all registered connectors (seeded + live sources)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and normalise listings without writing to the database",
    )
    args = parser.parse_args()

    if args.source:
        sources = [args.source]
    elif args.all:
        sources = list(CONNECTORS)
    else:
        # Default: seeded only — safe, offline, fast
        sources = ["seeded"]

    asyncio.run(run(sources, dry_run=args.dry_run))


if __name__ == "__main__":
    main()
