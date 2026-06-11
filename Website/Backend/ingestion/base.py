"""
Abstract base for all source connectors.

Every connector — seeded fixtures, OpenStreetMap, agency feeds, partner APIs —
implements BaseSourceConnector and yields RawListing objects. The ingestion
pipeline knows nothing about the source beyond this interface.

Architecture:
  External Sources → Source Connector → Normalization → Deduplication → Assets Table
                     ^^^^^^^^^^^^^^^^
                     This file defines the Connector contract.
"""

from abc import ABC, abstractmethod
from collections.abc import AsyncIterator
from dataclasses import dataclass, field


@dataclass
class RawListing:
    """
    Provider-specific listing data as returned by a connector.

    `raw_data` holds whatever the provider returned (HTML-parsed dict, API JSON,
    CSV row, etc.). The normalizer translates it into NormalizedAsset; nothing
    downstream knows the raw_data structure.
    """

    source: str           # connector name, e.g. "openstreetmap"
    source_listing_id: str  # provider's own unique ID for this listing
    source_url: str         # canonical URL for this listing
    raw_data: dict = field(default_factory=dict)


class BaseSourceConnector(ABC):
    """
    Abstract source connector interface.

    Implement fetch_listings() as an async generator that yields RawListing
    objects. Each connector is responsible for its own HTTP session, pagination,
    rate limiting, and authentication.

    Required class attribute:
        source_name: str — must be unique across all connectors; used as the
                           `source` field in the assets table.

    Design invariant:
        Connectors MUST NOT write to the database. They yield raw data; the
        SyncJob + normalizer + deduplication pipeline handles persistence.
    """

    source_name: str  # override in subclass

    @abstractmethod
    async def fetch_listings(self) -> AsyncIterator[RawListing]:
        """
        Yield raw listings from this source.

        Implementations should handle:
        - Pagination (yield all pages)
        - Rate limiting (sleep between requests)
        - Transient errors (retry with backoff; skip on repeated failure)
        - Filtering to commercial-only listings where the source requires it
        """
        ...

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        pass
