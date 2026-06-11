// ─────────────────────────────────────────────────────────────────────────────
// Assessment V2 — three-output model (SYSTEM_DESIGN.md §2.3)
//
// A ConceptAssessment is the result of running one business concept against one
// Asset. It holds three intelligence faces:
//   1. Feasibility  — always present  (Foundation)
//   2. Valuation    — may be null     (Growth; data model is Foundation)
//   3. Temporal     — always present as a struct, but activity data may be null
//                     (Growth; gated by coverage/sensor density)
//
// The null / sparse states are first-class — not error states. The frontend
// renders them honestly ("valuation data not yet available for this asset").
// ─────────────────────────────────────────────────────────────────────────────

import type { BusinessDomain, BusinessParameters } from "./parameters";
import type { ContributingFactor, ScoreBand } from "./report";

// BusinessConcept is the production-architecture name for what the existing code
// calls BusinessDomain. They describe the same set of values. New code should
// reference BusinessConcept; the underlying string literals are identical.
export type BusinessConcept = BusinessDomain;

// ── Face 1: Feasibility intelligence (Foundation) ─────────────────────────────
// "Would my business work here?"
// Always present on every ConceptAssessment.

export interface FeasibilityOutput {
  feasibilityScore: number;                    // 0–100
  successLikelihood: number;                   // 0–1 probability; e.g. 0.72
  confidenceInterval: { low: number; high: number }; // always shown; never a bare point estimate
  scoreBand: ScoreBand;
  contributingFactors: ContributingFactor[];
  narrative: string;                           // plain-language explanation
  labelQuality: "high" | "medium" | "low";
  dataSources: string[];
  dataPointCount: number;
  generatedAt: string;                         // ISO timestamp
}

// ── Face 2: Valuation intelligence (Growth; data model Foundation) ─────────────
// "Is this asset fairly priced, and what is it worth?"
// null means the valuation service has insufficient data for this asset.
// The Asset carries monthlyRent / salePrice from day one so the service always
// has a listed price to reason against; the valuation OUTPUT matures over time.

export type ValuationConfidence = "high" | "medium" | "low";
export type PriceDelta = "above_market" | "at_market" | "below_market";

// The valuation engine runs in two modes, chosen by whether the asset has a
// listed price. The `mode` field tells the UI which face to render.
export type ValuationMode = "listing_evaluation" | "opportunity_discovery";

// Mode A (listing_evaluation): is the listed price fair?
export type ListingVerdict = "underpriced" | "fairly_priced" | "overpriced";
// Mode B (opportunity_discovery): how strong is the commercial potential? (no price)
export type PotentialVerdict =
  | "high_potential"
  | "moderate_potential"
  | "low_potential"
  | "insufficient_data";
export type ValuationVerdict = ListingVerdict | PotentialVerdict;

export interface ValuationRange {
  low: number;
  high: number;
}

// A single traceable reason behind the valuation verdict (transparent rule engine).
export interface ValuationDriver {
  label: string;
  detail: string;
  direction: "positive" | "negative" | "neutral";
}

export interface ValuationOutput {
  mode: ValuationMode;           // distinguishes the two output types
  verdict: ValuationVerdict;     // ListingVerdict (Mode A) | PotentialVerdict (Mode B)
  fairPriceEstimate: number;     // centre of the fair-value range (indicative in Mode B)
  fairValueRange: ValuationRange; // estimated fair range — never a bare point estimate
  expectedValue: number;         // expected value given commercial-potential signal
  currency: string;
  confidence: ValuationConfidence; // always more conservative than feasibility confidence
  priceDelta: PriceDelta | null; // vs. listed price; null in Mode B (no listed price)
  valuationGapPct: number | null; // listed vs. fair centre, signed %; null in Mode B
  drivers: ValuationDriver[];    // traceable reasons behind the verdict
  valuationNarrative: string;
  generatedAt: string;
}

// ── Face 3: Temporal / coverage intelligence (Growth; gated by coverage) ────────
// "When is this place valuable, and how well do we understand it?"
//
// coverageScore and sourceChecklist are ALWAYS present — coverage is always
// knowable (we always know what data sources we have or don't have).
//
// activityProfile, peakWindows, forecastHorizon, and temporalConfidence are
// CONDITIONAL — only populated where sensor coverage permits. When temporalConfidence
// is null the platform has no temporal data for this asset; that is an honest,
// renderable state, not an error.

export type CoverageSourceKey =
  | "demographics"
  | "gis"
  | "traffic"
  | "competition"
  | "commercial_activity"
  | "edgeai_telemetry";

export const COVERAGE_SOURCE_LABELS: Record<CoverageSourceKey, string> = {
  demographics:        "Demographic model",
  gis:                 "GIS / land-use layers",
  traffic:             "Traffic & footfall data",
  competition:         "Competitive registry",
  commercial_activity: "Commercial activity (POI)",
  edgeai_telemetry:    "EdgeAI sensor telemetry",
};

export interface CoverageSourceStatus {
  source: CoverageSourceKey;
  label: string;    // human-readable label from COVERAGE_SOURCE_LABELS
  present: boolean; // true = data is available; false = missing for this asset
}

export type TemporalConfidence = "high" | "medium" | "low";

export interface ActivityDataPoint {
  hour: number;              // 0–23
  dayOfWeek: number;         // 0 = Sunday … 6 = Saturday
  relativeActivity: number;  // 0–1 normalized activity level for that slot
}

export interface PeakWindow {
  label: string;             // e.g. "Weekday lunch (12:00–14:00)"
  dayPattern: string;        // e.g. "Monday–Friday"
  peakHours: [number, number]; // [start, end] in 24h
  intensity: "high" | "moderate";
}

export interface TemporalOutput {
  // Always present ─────────────────────────────────
  coverageScore: number;            // 0–100; same value surfaced on the Asset
  sourceChecklist: CoverageSourceStatus[]; // one entry per of the six sources

  // Conditional: null when no sensor coverage ───────
  activityProfile: ActivityDataPoint[] | null;  // null = no temporal data
  peakWindows: PeakWindow[] | null;             // null = no temporal data
  forecastHorizon: string | null;               // e.g. "7 days"; null = no temporal data
  temporalConfidence: TemporalConfidence | null; // null = no temporal data
}

// ── ConceptAssessment: the three-output container ─────────────────────────────
// The fundamental unit of the one-to-many model: one Asset, one BusinessConcept,
// one ConceptAssessment holding all three intelligence faces.

export interface ConceptAssessment {
  assetId: string;
  concept: BusinessConcept;
  parameters: BusinessParameters;
  generatedAt: string; // ISO timestamp of when this assessment was generated

  // The three intelligence faces (SYSTEM_DESIGN.md §2.3):
  feasibility: FeasibilityOutput;        // always present (Foundation)
  valuation: ValuationOutput | null;     // null when insufficient data (Growth)
  temporal: TemporalOutput;              // struct always present; activity data may be null
}

// ── Collection types used by the store ────────────────────────────────────────

// All concept-assessments for one asset — keyed by BusinessConcept string value.
// Partial because not every concept has been assessed against every asset.
export type ConceptAssessmentRecord = Partial<Record<BusinessConcept, ConceptAssessment>>;

// All concept-assessments across all assets in the session — keyed by assetId.
// Switching assets does NOT clear this map; it accumulates across the session.
export type AssetConceptAssessmentMap = Record<string, ConceptAssessmentRecord>;
