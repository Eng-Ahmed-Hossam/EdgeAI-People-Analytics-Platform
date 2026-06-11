"""
Normalization layer: converts provider-specific RawListing into NormalizedAsset.

Every provider names fields differently:

  Seeded fixtures:   canonical field names (minimal mapping)
  OpenStreetMap:     lat/lon, tags dict (addr:*, shop, office, building, name, …)
  Agency feeds:      provider-specific JSON / CSV
  Partner APIs:      TBD per partner contract

All normalize into the platform's canonical Asset contract before entering the DB.
The normalizer knows about provider formats; nothing else does.

Adding a new source:
  1. Add a method `_normalize_<source_name>(self, raw: RawListing) -> NormalizedAsset | None`
  2. Register it in the dispatch table in normalize().
"""

import logging
import re
from dataclasses import dataclass
from typing import Optional

from ingestion.base import RawListing

logger = logging.getLogger(__name__)


# ── Canonical NormalizedAsset ─────────────────────────────────────────────────

@dataclass
class NormalizedAsset:
    """
    Provider-agnostic asset data ready for deduplication and DB insertion.
    Field names mirror the ORM Asset model (snake_case).
    """

    # Required
    source: str
    source_listing_id: str
    source_url: str
    lat: float
    lng: float
    address: str
    area: str
    currency: str
    listing_source: str

    # Optional commercial terms
    monthly_rent: Optional[float] = None
    sale_price: Optional[float] = None
    size: Optional[float] = None

    # Classification (defaults match frontend defaults)
    property_type: str = "retail_unit"
    availability_status: str = "available"

    # Coverage score — real computation is Coverage Engine (future session)
    coverage_score: int = 0


# ── Provider-specific normalization helpers ───────────────────────────────────

def _clean_price(raw: object) -> Optional[float]:
    """Parse a price string or number into a float, returning None if unparseable."""
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return float(raw) if raw > 0 else None
    s = str(raw).replace(",", "").strip()
    s = re.sub(r"[^\d.]", "", s)
    try:
        value = float(s)
        return value if value > 0 else None
    except ValueError:
        return None


def _clean_size(raw: object) -> Optional[float]:
    """Parse size in sq metres. Handles 'sqft' by converting (×0.0929)."""
    if raw is None:
        return None
    s = str(raw).lower()
    sqft = "sqft" in s or "sq ft" in s
    s = re.sub(r"[^\d.]", "", s)
    try:
        value = float(s)
        if sqft:
            value *= 0.0929
        return round(value, 1) if value > 0 else None
    except ValueError:
        return None


def _normalize_property_type(raw: object) -> str:
    """Map provider-specific property type strings to the frontend PropertyType enum."""
    s = str(raw).lower().replace(" ", "_") if raw else ""
    mapping = {
        "retail": "retail_unit",
        "retail_unit": "retail_unit",
        "shop": "retail_unit",
        "store": "retail_unit",
        "commercial": "ground_floor_commercial",
        "ground_floor": "ground_floor_commercial",
        "office": "office_unit",
        "offices": "office_unit",
        "kiosk": "kiosk",
        "warehouse": "warehouse",
        "storage": "warehouse",
        "mixed": "mixed_use",
        "mixed_use": "mixed_use",
    }
    for key, val in mapping.items():
        if key in s:
            return val
    return "other"


# ── Normalizer class ──────────────────────────────────────────────────────────

class ListingNormalizer:
    """
    Dispatches normalization to the correct per-source method.

    Adding a new source:
      1. Add `_normalize_<source_name>(self, raw: RawListing) -> NormalizedAsset | None`
      2. Register it in the dispatch table inside normalize().
    """

    def normalize(self, raw: RawListing) -> Optional[NormalizedAsset]:
        """
        Return NormalizedAsset or None if the listing should be skipped
        (missing coordinates, no useful identity, non-commercial, etc.).
        """
        dispatch = {
            "seeded": self._normalize_seeded,
            "openstreetmap": self._normalize_openstreetmap,
            # Future connectors registered here:
            # "aqarmap": self._normalize_aqarmap,
            # "property_finder": self._normalize_property_finder,
            # "partner_xyz": self._normalize_partner_xyz,
        }
        handler = dispatch.get(raw.source)
        if handler is None:
            logger.warning(
                "No normalizer for source '%s'; skipping listing %s",
                raw.source,
                raw.source_listing_id,
            )
            return None
        try:
            return handler(raw)
        except Exception as exc:
            logger.error(
                "Normalization failed for %s / %s: %s",
                raw.source,
                raw.source_listing_id,
                exc,
            )
            return None

    # ── Seeded connector normalizer ───────────────────────────────────────────

    def _normalize_seeded(self, raw: RawListing) -> Optional[NormalizedAsset]:
        """
        Seeded assets already use the canonical field names — minimal mapping.
        Required fields in raw_data: lat, lng, address, area, currency.
        """
        d = raw.raw_data
        lat = d.get("lat")
        lng = d.get("lng")
        if lat is None or lng is None:
            return None

        return NormalizedAsset(
            source=raw.source,
            source_listing_id=raw.source_listing_id,
            source_url=raw.source_url,
            lat=float(lat),
            lng=float(lng),
            address=d.get("address", ""),
            area=d.get("area", ""),
            currency=d.get("currency", "EGP"),
            listing_source="seeded",
            monthly_rent=_clean_price(d.get("monthly_rent")),
            sale_price=_clean_price(d.get("sale_price")),
            size=_clean_size(d.get("size")),
            property_type=_normalize_property_type(d.get("property_type", "retail_unit")),
            availability_status=d.get("availability_status", "available"),
            coverage_score=int(d.get("coverage_score", 0)),
        )

    # ── OpenStreetMap normalizer ──────────────────────────────────────────────

    def _normalize_openstreetmap(self, raw: RawListing) -> Optional[NormalizedAsset]:
        """
        OpenStreetMap element (from Overpass API):
          raw_data keys: lat, lon, osm_type, osm_id, tags (dict of OSM k/v tags)

        Pricing:
          Always None — OSM does not provide rental or sale prices.
          The honest-absence design ensures the frontend renders correctly.

        Availability:
          OSM does not track listing availability. All OSM assets are ingested
          as "available" (documented default). Real availability must come from
          a listing source or operator confirmation.

        ListingSource:
          "other" — OSM is crowd-sourced, not an official government registry
          or commercial listing partner.
        """
        d = raw.raw_data
        lat = d.get("lat")
        lon = d.get("lon")
        if lat is None or lon is None:
            return None

        tags = d.get("tags", {})

        # Compose address from OSM tags; skip elements with no usable identity
        address = self._osm_address(tags)
        if not address:
            logger.debug(
                "OSM element %s has no usable address or name; skipping",
                raw.source_listing_id,
            )
            return None

        area = self._osm_area(tags)

        return NormalizedAsset(
            source=raw.source,
            source_listing_id=raw.source_listing_id,
            source_url=raw.source_url,
            lat=float(lat),
            lng=float(lon),
            address=address,
            area=area,
            currency="EGP",
            listing_source="other",      # OSM is crowd-sourced, not a registry
            monthly_rent=None,           # OSM provides no pricing data
            sale_price=None,             # OSM provides no pricing data
            size=_clean_size(tags.get("building:floor_area") or tags.get("floor_area")),
            property_type=self._osm_property_type(tags),
            availability_status="available",  # OSM has no availability data; documented default
            coverage_score=0,            # Coverage Engine computes the real value (future session)
        )

    @staticmethod
    def _osm_address(tags: dict) -> str:
        """
        Compose a human-readable address from OSM addr:* tags and name.

        Priority:
          1. housenumber + street (+ suburb if available)
          2. street only (+ suburb)
          3. name (+ suburb / city)

        Returns empty string if no useful identity is available.
        Elements with no name and no address tags are skipped by the caller.
        """
        housenumber = tags.get("addr:housenumber", "").strip()
        street = tags.get("addr:street", "").strip()
        name = tags.get("name", "").strip()
        suburb = (
            tags.get("addr:suburb")
            or tags.get("addr:neighbourhood")
            or tags.get("addr:quarter")
            or ""
        ).strip()
        city = tags.get("addr:city", "").strip()

        # Build base (most specific available)
        if housenumber and street:
            base = f"{housenumber} {street}"
        elif street:
            base = street
        elif name:
            base = name
        else:
            return ""  # no useful identity

        # Append district qualifier if distinct from the base
        if suburb and suburb.lower() not in base.lower():
            return f"{base}, {suburb}"
        if city and city.lower() not in base.lower():
            return f"{base}, {city}"
        return base

    @staticmethod
    def _osm_area(tags: dict) -> str:
        """
        Extract the district / neighbourhood label from OSM tags.
        Falls back to "Greater Cairo" when no area tag is present.
        """
        return (
            tags.get("addr:suburb")
            or tags.get("addr:neighbourhood")
            or tags.get("addr:quarter")
            or tags.get("addr:district")
            or tags.get("is_in:suburb")
            or tags.get("addr:city")
            or "Greater Cairo"
        )

    @staticmethod
    def _osm_property_type(tags: dict) -> str:
        """
        Derive frontend PropertyType from OSM primary tags.

        Mapping:
          shop=*           → retail_unit
          office=*         → office_unit
          building=retail  → retail_unit
          building=commercial → ground_floor_commercial
          craft=*          → other
          (default)        → other
        """
        if tags.get("shop"):
            return "retail_unit"
        if tags.get("office"):
            return "office_unit"
        building = tags.get("building", "")
        if building == "retail":
            return "retail_unit"
        if building == "commercial":
            return "ground_floor_commercial"
        return "other"
