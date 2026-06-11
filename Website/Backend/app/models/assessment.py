"""
Assessment ORM model — one persisted ConceptAssessment (a single business
concept scored against a single asset).

Persistence unit (SYSTEM_DESIGN.md §2.2): one concept-against-an-asset is stored
independently, groupable by asset later. We do NOT store an asset-with-its-whole-
collection as one blob.

Immutability: an assessment record is written once and never UPDATEd. This is
why denormalised scalar columns (score, confidence, …) cannot drift from the
authoritative `feasibility_json` — there is no code path that mutates a row.

Round-trip guarantee: `feasibility_json` holds the complete FeasibilityOutput
verbatim. On retrieval the API reassembles the exact same FeasibilityOutput a
live engine run produces, field-for-field, including contributingFactors order.
The decomposed columns (drivers_json / risks_json / explanation_json) and the
scalar columns are a queryable projection, NOT the reconstruction source.
"""

import uuid

from sqlalchemy import (
    Column,
    DateTime,
    Float,
    ForeignKey,
    Index,
    Integer,
    String,
    func,
)
from sqlalchemy.dialects.postgresql import JSONB

from app.database import Base


class Assessment(Base):
    __tablename__ = "assessments"

    # ── Identity ─────────────────────────────────────────────────────────────
    id = Column(String(255), primary_key=True, default=lambda: str(uuid.uuid4()))

    # ── Relationships ─────────────────────────────────────────────────────────
    # FK to the asset this concept was scored against.
    asset_id = Column(
        String(255),
        ForeignKey("assets.id", ondelete="CASCADE"),
        nullable=False,
    )

    # Ownership-ready from day one. Nullable during the no-auth phase; auth later
    # just populates this column — no schema redesign required. Functionally
    # unscoped for now.
    owner_id = Column(String(255), nullable=True)

    # ── Concept ───────────────────────────────────────────────────────────────
    concept_type = Column(String(50), nullable=False)

    # ── Reproducibility ───────────────────────────────────────────────────────
    # The engine version that produced this record. Required, not optional, so a
    # reopened assessment is interpretable in its own context after the engine
    # evolves.
    engine_version = Column(String(50), nullable=False)

    # ── Denormalised queryable scalars (projection of FeasibilityOutput) ───────
    # score                ← feasibilityScore
    # confidence           ← labelQuality (the high/medium/low confidence descriptor)
    # confidence_range_low ← confidenceInterval.low
    # confidence_range_high← confidenceInterval.high
    # coverage_score       ← asset.coverage_score at save time (the gating input)
    score = Column(Integer, nullable=False)
    success_likelihood = Column(Float, nullable=False)
    score_band = Column(String(20), nullable=False)
    confidence = Column(String(20), nullable=False)
    confidence_range_low = Column(Integer, nullable=False)
    confidence_range_high = Column(Integer, nullable=False)
    coverage_score = Column(Integer, nullable=False)
    data_point_count = Column(Integer, nullable=False)

    # ── Structured engine outputs as JSON (requested decomposition) ────────────
    # drivers_json     — contributing factors with direction == "positive"
    # risks_json       — contributing factors with direction == "negative"
    # explanation_json — { narrative, dataSources, neutralFactors }
    drivers_json = Column(JSONB, nullable=False)
    risks_json = Column(JSONB, nullable=False)
    explanation_json = Column(JSONB, nullable=False)

    # ── Authoritative complete output (lossless round-trip source) ─────────────
    # The entire FeasibilityOutput, verbatim. Reassembled on GET to guarantee the
    # stored shape == live engine shape, field-for-field.
    feasibility_json = Column(JSONB, nullable=False)

    # ── Timestamps ────────────────────────────────────────────────────────────
    # generated_at — when the ENGINE produced the output (FeasibilityOutput.generatedAt)
    # created_at   — when the record was PERSISTED
    generated_at = Column(DateTime(timezone=True), nullable=True)
    created_at = Column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    # ── Indexes for retrieval patterns ─────────────────────────────────────────
    __table_args__ = (
        Index("ix_assessments_asset_id", "asset_id"),
        Index("ix_assessments_owner_id", "owner_id"),
        Index("ix_assessments_created_at", "created_at"),
        # History view: a user's assessments newest-first.
        Index("ix_assessments_owner_created", "owner_id", "created_at"),
    )
