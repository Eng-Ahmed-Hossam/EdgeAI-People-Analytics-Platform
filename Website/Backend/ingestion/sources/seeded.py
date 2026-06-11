"""
SeededConnector — loads pre-verified assets from a JSON fixture file.

This is the guaranteed-to-work fallback connector. It runs first in every
sync so the platform always has a baseline dataset even when live scrapers
fail or are offline.

The seed file (ingestion/seed_data.json) uses the canonical Asset field names.
The normalizer's _normalize_seeded() handles it with minimal mapping.
"""

import json
import logging
from collections.abc import AsyncIterator
from pathlib import Path

from ingestion.base import BaseSourceConnector, RawListing

logger = logging.getLogger(__name__)


class SeededConnector(BaseSourceConnector):
    source_name = "seeded"

    def __init__(self, seed_path: str | Path | None = None) -> None:
        if seed_path is None:
            seed_path = Path(__file__).parent.parent / "seed_data.json"
        self._seed_path = Path(seed_path)

    async def fetch_listings(self) -> AsyncIterator[RawListing]:
        if not self._seed_path.exists():
            logger.error("Seed data file not found: %s", self._seed_path)
            return

        with open(self._seed_path, encoding="utf-8") as f:
            data = json.load(f)

        assets = data.get("assets", [])
        logger.info("SeededConnector: loading %d assets from %s", len(assets), self._seed_path)

        for asset in assets:
            asset_id = asset.get("id")
            if not asset_id:
                logger.warning("Seeded asset missing 'id'; skipping: %s", asset)
                continue

            yield RawListing(
                source=self.source_name,
                source_listing_id=asset_id,
                source_url=f"seeded://{asset_id}",
                raw_data=asset,
            )
