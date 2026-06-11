import uuid
from datetime import datetime

from geoalchemy2 import Geometry
from sqlalchemy import (
    Boolean,
    Column,
    DateTime,
    Float,
    Index,
    Integer,
    String,
    Text,
    UniqueConstraint,
    func,
)

from app.database import Base


class Asset(Base):
    """
    Central domain object — a real commercial opportunity (rent or purchase).

    The location column is a PostGIS geometry point (SRID 4326) enabling
    spatial queries (radius search, bounding box, distance ordering). All
    other fields conform to the frontend Asset contract in src/types/asset.ts.

    Provenance fields (source, source_listing_id, source_url, *_at) are
    populated by the ingestion pipeline and must not be set by the API layer.
    """

    __tablename__ = "assets"

    # ── Identity ─────────────────────────────────────────────────────────────
    id = Column(String(255), primary_key=True, default=lambda: str(uuid.uuid4()))

    # ── Spatial ──────────────────────────────────────────────────────────────
    # POINT geometry in WGS84 (SRID 4326). Insert via ST_SetSRID(ST_MakePoint(lng, lat), 4326).
    location = Column(Geometry("POINT", srid=4326, spatial_index=True), nullable=False)

    # ── Location descriptors ──────────────────────────────────────────────────
    address = Column(String(500), nullable=False)
    area = Column(String(200), nullable=False)       # district / neighbourhood label
    size = Column(Float, nullable=True)              # sq metres; optional

    # ── Commercial terms ──────────────────────────────────────────────────────
    # Listed prices the valuation service reasons against (SYSTEM_DESIGN.md §2.1)
    monthly_rent = Column(Float, nullable=True)      # asking monthly rent
    sale_price = Column(Float, nullable=True)        # asking purchase price
    currency = Column(String(10), nullable=False, default="EGP")

    # ── Classification ────────────────────────────────────────────────────────
    # Values must match frontend PropertyType union
    property_type = Column(
        String(50), nullable=False, default="retail_unit"
    )
    # Values must match frontend AvailabilityStatus union
    # DB also supports "expired" for lifecycle tracking; API only exposes the 4 frontend values
    availability_status = Column(
        String(20), nullable=False, default="available"
    )

    # ── Provenance ────────────────────────────────────────────────────────────
    # ListingSource enum values match frontend contract
    listing_source = Column(String(30), nullable=False, default="other")

    # Ingestion provenance fields — the API layer never reads these
    source = Column(String(100), nullable=True)          # connector name, e.g. "openstreetmap"
    source_listing_id = Column(String(255), nullable=True)  # provider's own listing ID
    source_url = Column(Text, nullable=True)                # canonical listing URL
    first_seen_at = Column(DateTime(timezone=True), nullable=True)
    last_seen_at = Column(DateTime(timezone=True), nullable=True)
    ingestion_timestamp = Column(
        DateTime(timezone=True), nullable=True, server_default=func.now()
    )

    # ── Intelligence readiness ────────────────────────────────────────────────
    # Stored in DB; real computation is the Coverage Engine (future session).
    coverage_score = Column(Integer, nullable=False, default=0)

    # ── Soft-delete / active flag ─────────────────────────────────────────────
    is_active = Column(Boolean, nullable=False, default=True)

    # ── Constraints ───────────────────────────────────────────────────────────
    __table_args__ = (
        # Prevents re-inserting the same listing from the same source
        UniqueConstraint("source", "source_listing_id", name="uq_asset_source_listing"),
        # Spatial index is created by GeoAlchemy2 via spatial_index=True above.
        # Extra composite index for common API query pattern.
        Index("ix_assets_status_active", "availability_status", "is_active"),
    )
