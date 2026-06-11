import uuid
from datetime import datetime

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
    __tablename__ = "assets"

    # ── Identity ─────────────────────────────────────────────────────────────
    id = Column(String(255), primary_key=True, default=lambda: str(uuid.uuid4()))

    # ── Spatial ──────────────────────────────────────────────────────────────
    # Plain floats — no PostGIS dependency; sufficient for all current queries.
    latitude = Column(Float, nullable=False)
    longitude = Column(Float, nullable=False)

    # ── Location descriptors ──────────────────────────────────────────────────
    address = Column(String(500), nullable=False)
    area = Column(String(200), nullable=False)
    size = Column(Float, nullable=True)

    # ── Commercial terms ──────────────────────────────────────────────────────
    monthly_rent = Column(Float, nullable=True)
    sale_price = Column(Float, nullable=True)
    currency = Column(String(10), nullable=False, default="EGP")

    # ── Classification ────────────────────────────────────────────────────────
    property_type = Column(String(50), nullable=False, default="retail_unit")
    availability_status = Column(String(20), nullable=False, default="available")

    # ── Provenance ────────────────────────────────────────────────────────────
    listing_source = Column(String(30), nullable=False, default="other")
    source = Column(String(100), nullable=True)
    source_listing_id = Column(String(255), nullable=True)
    source_url = Column(Text, nullable=True)
    first_seen_at = Column(DateTime(timezone=True), nullable=True)
    last_seen_at = Column(DateTime(timezone=True), nullable=True)
    ingestion_timestamp = Column(
        DateTime(timezone=True), nullable=True, server_default=func.now()
    )

    # ── Intelligence readiness ────────────────────────────────────────────────
    coverage_score = Column(Integer, nullable=False, default=0)

    # ── Soft-delete ───────────────────────────────────────────────────────────
    is_active = Column(Boolean, nullable=False, default=True)

    # ── Constraints ───────────────────────────────────────────────────────────
    __table_args__ = (
        UniqueConstraint("source", "source_listing_id", name="uq_asset_source_listing"),
        Index("ix_assets_status_active", "availability_status", "is_active"),
        Index("ix_assets_lat_lng", "latitude", "longitude"),
    )
