"""
Pydantic response schemas for the Asset API.

Naming convention boundary (SYSTEM_DESIGN.md §5.2):
  DB / Python: snake_case  (monthly_rent, sale_price, …)
  JSON output: camelCase   (monthlyRent, salePrice, …)

Resolution: alias_generator=to_camel + serialize_by_alias=True.
Every field is named in Python snake_case; the serialised JSON uses camelCase
automatically. No field-level aliases needed — the generator handles them all.

The frontend Asset type in src/types/asset.ts is the authority.
This schema must match it exactly; never change the frontend type to match here.
"""

from typing import Optional

from pydantic import BaseModel, ConfigDict
from pydantic.alias_generators import to_camel


class AssetCoordinates(BaseModel):
    """{ lat, lng } — both names are already single-word so to_camel leaves them unchanged."""

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,
        serialize_by_alias=True,
    )

    lat: float
    lng: float


class AssetResponse(BaseModel):
    """
    Serialised representation of an Asset for GET /assets and GET /assets/{id}.

    Field names use Python snake_case; JSON output is camelCase via alias_generator.
    Matches frontend Asset interface in src/types/asset.ts exactly.

    PropertyType values:
      "retail_unit" | "ground_floor_commercial" | "office_unit" |
      "kiosk" | "warehouse" | "mixed_use" | "other"

    AvailabilityStatus values:
      "available" | "reserved" | "occupied" | "sold"

    ListingSource values:
      "seeded" | "public_registry" | "partner" | "direct" | "other"
    """

    model_config = ConfigDict(
        alias_generator=to_camel,
        populate_by_name=True,
        serialize_by_alias=True,
    )

    # Identity & location — always present
    id: str
    coordinates: AssetCoordinates
    address: str
    area: str
    size: Optional[float] = None

    # Commercial terms — either or both may be absent
    monthly_rent: Optional[float] = None
    sale_price: Optional[float] = None
    currency: str

    # Classification — always present
    property_type: str
    availability_status: str

    # Provenance — always present
    listing_source: str

    # Intelligence readiness — always present (0–100)
    coverage_score: int
