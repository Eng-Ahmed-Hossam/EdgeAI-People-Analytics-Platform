"""
OpenStreetMapConnector — queries commercial POIs from the Overpass API.

Data source:
    OpenStreetMap via Overpass API (https://overpass-api.de/api/interpreter)
    © OpenStreetMap contributors — https://www.openstreetmap.org/copyright
    License: Open Database License (ODbL) — https://opendatacommons.org/licenses/odbl/

Coverage:
    Greater Cairo bounding box:
        south=29.85, west=31.00, north=30.25, east=31.65
    Includes: Maadi, Zamalek, Heliopolis, Nasr City, New Cairo, Giza,
              Imbaba, Shubra el-Kheima, Downtown Cairo, Dokki, Mohandeseen.

Query strategy:
    One bulk Overpass QL query per run — efficient, single HTTP call, minimal
    server load. Results are capped at RESULT_LIMIT to keep the first-run
    dataset manageable. Increase RESULT_LIMIT (or split by district) as the
    platform matures.

OSM tags queried (commercial signals):
    shop=*             → retail units, supermarkets, boutiques, pharmacies, …
    office=*           → office units, coworking spaces, professional services, …
    building=commercial → commercial buildings (ground-floor or multi-storey)
    building=retail    → purpose-built retail buildings

Rate limiting:
    One Overpass query per connector run. Overpass fair-use guidelines request
    that clients avoid high-frequency polling. This connector queries once per
    scheduled sync and includes a descriptive User-Agent for identification.

Pricing:
    OSM does NOT provide rental or sale prices.
    monthly_rent = None, sale_price = None — always.
    No estimation, inference, or fabrication.

Availability:
    OSM does NOT track listing availability.
    All ingested assets use availability_status = "available" as a documented
    default. This field reflects "the platform has data about this location,"
    not "this space is actively listed for lease."

Attribution (ODbL compliance):
    All records ingested from this connector carry source="openstreetmap".
    source_url points to the canonical OSM element page.
    The platform must attribute OSM data if rendered publicly.
"""

import logging
from collections.abc import AsyncIterator
from typing import Optional

import httpx

from ingestion.base import BaseSourceConnector, RawListing

logger = logging.getLogger(__name__)

# ── Configuration constants ───────────────────────────────────────────────────

OVERPASS_URL = "https://overpass-api.de/api/interpreter"

# Greater Cairo bounding box: (south, west, north, east)
# Covers Maadi → New Cairo (E), Zamalek/Imbaba (W), Heliopolis/Nasr City (N), Maadi (S)
CAIRO_BBOX = (29.85, 31.00, 30.25, 31.65)

# Hard cap on results returned in a single query.
# ~400 commercial POIs is a strong first dataset for the demo region.
# Increase to 1000+ as the pipeline and DB are proven stable.
RESULT_LIMIT = 400

# Timeout for the Overpass HTTP request (seconds).
# Overpass queries over Greater Cairo may take 20–40 s depending on server load.
REQUEST_TIMEOUT = 90.0

# Overpass QL query template.
# {bbox} is replaced with "(south,west,north,east)" before sending.
# {limit} is replaced with RESULT_LIMIT.
# "out center tags {limit};" returns:
#   - nodes:  lat, lon, tags  (center modifier is a no-op for nodes)
#   - ways:   center.lat, center.lon, tags  (no node-ref arrays — keeps response compact)
_OVERPASS_QUERY_TEMPLATE = """\
[out:json][timeout:60];
(
  node["shop"]{bbox};
  way["shop"]{bbox};
  node["office"]{bbox};
  way["office"]{bbox};
  node["building"="commercial"]{bbox};
  way["building"="commercial"]{bbox};
  node["building"="retail"]{bbox};
  way["building"="retail"]{bbox};
);
out center tags {limit};
"""


# ── Connector ─────────────────────────────────────────────────────────────────

class OpenStreetMapConnector(BaseSourceConnector):
    """
    Fetches commercial POIs from OpenStreetMap via the Overpass API.

    Yields one RawListing per OSM element. The normalizer
    (_normalize_openstreetmap in ingestion/normalizer.py) converts the raw
    Overpass element into a NormalizedAsset.

    source_name = "openstreetmap"
    source_listing_id = str(osm_element_id)   (globally unique within OSM)
    source_url = "https://www.openstreetmap.org/{type}/{id}"
    """

    source_name = "openstreetmap"

    def __init__(
        self,
        user_agent: str = "edgeai-location-intelligence/1.0 (educational; contact: eng.ahmed.hossam144@gmail.com)",
        bbox: tuple[float, float, float, float] = CAIRO_BBOX,
        result_limit: int = RESULT_LIMIT,
    ) -> None:
        self._user_agent = user_agent
        self._bbox = bbox
        self._result_limit = result_limit

    # ── Public interface ──────────────────────────────────────────────────────

    async def fetch_listings(self) -> AsyncIterator[RawListing]:
        """
        Issue one Overpass QL query and yield a RawListing for each element.

        Elements without extractable coordinates are silently skipped here;
        the normalizer is the appropriate place for semantic filtering.
        """
        query = self._build_query()
        logger.info(
            "OpenStreetMapConnector: querying Overpass API (bbox=%s, limit=%d)",
            self._bbox,
            self._result_limit,
        )

        elements = await self._fetch_overpass(query)
        logger.info("OpenStreetMapConnector: received %d elements from Overpass", len(elements))

        yielded = 0
        skipped_no_coords = 0

        for element in elements:
            raw = self._element_to_raw_listing(element)
            if raw is None:
                skipped_no_coords += 1
                continue
            yielded += 1
            yield raw

        logger.info(
            "OpenStreetMapConnector: yielded %d listings (%d skipped — no coordinates)",
            yielded,
            skipped_no_coords,
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _build_query(self) -> str:
        """Inject bbox and limit into the Overpass QL template."""
        south, west, north, east = self._bbox
        bbox_str = f"({south},{west},{north},{east})"
        return _OVERPASS_QUERY_TEMPLATE.format(bbox=bbox_str, limit=self._result_limit)

    async def _fetch_overpass(self, query: str) -> list[dict]:
        """
        POST the Overpass QL query and return the `elements` array.
        Returns an empty list on HTTP or parsing failure (logged as error).
        """
        headers = {
            "User-Agent": self._user_agent,
            "Accept": "application/json",
            "Content-Type": "application/x-www-form-urlencoded",
        }
        try:
            async with httpx.AsyncClient(timeout=REQUEST_TIMEOUT) as client:
                response = await client.post(
                    OVERPASS_URL,
                    data={"data": query},
                    headers=headers,
                )
                response.raise_for_status()
                payload = response.json()
                return payload.get("elements", [])

        except httpx.HTTPStatusError as exc:
            logger.error(
                "Overpass API returned HTTP %d: %s",
                exc.response.status_code,
                exc.response.text[:500],
            )
            return []
        except httpx.TimeoutException:
            logger.error(
                "Overpass API request timed out after %.0fs. "
                "Try reducing RESULT_LIMIT or splitting the bounding box.",
                REQUEST_TIMEOUT,
            )
            return []
        except Exception as exc:
            logger.error("Unexpected error querying Overpass API: %s", exc)
            return []

    def _element_to_raw_listing(self, element: dict) -> Optional[RawListing]:
        """
        Convert one Overpass API element dict into a RawListing.

        Coordinate extraction:
          - nodes:  element["lat"] / element["lon"]  (always present for nodes)
          - ways:   element["center"]["lat"] / element["center"]["lon"]
                    (present when query includes "out center")
          - Any element without extractable coordinates is skipped.

        raw_data structure passed to the normalizer:
          {
            "lat": float,
            "lon": float,
            "osm_type": "node" | "way",
            "osm_id": int,
            "tags": dict,   # all OSM key/value tags on this element
          }
        """
        osm_type = element.get("type", "")
        osm_id = element.get("id")

        if osm_type == "node":
            lat = element.get("lat")
            lon = element.get("lon")
        elif osm_type == "way":
            center = element.get("center", {})
            lat = center.get("lat")
            lon = center.get("lon")
        else:
            # Relations: skip (complex geometry, rare for commercial POIs)
            return None

        if lat is None or lon is None:
            return None

        source_url = f"https://www.openstreetmap.org/{osm_type}/{osm_id}"

        return RawListing(
            source=self.source_name,
            source_listing_id=str(osm_id),
            source_url=source_url,
            raw_data={
                "lat": lat,
                "lon": lon,
                "osm_type": osm_type,
                "osm_id": osm_id,
                "tags": element.get("tags", {}),
            },
        )
