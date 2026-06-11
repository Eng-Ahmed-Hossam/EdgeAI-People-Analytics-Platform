import type { Asset } from "@/types/asset";
import type {
  BusinessConcept,
  ConceptAssessment,
  CoverageSourceKey,
  CoverageSourceStatus,
} from "@/types/assessment-v2";
import { COVERAGE_SOURCE_LABELS } from "@/types/assessment-v2";

// ── Shared helpers ────────────────────────────────────────────────────────────

function buildChecklist(present: CoverageSourceKey[]): CoverageSourceStatus[] {
  const ALL: CoverageSourceKey[] = [
    "demographics",
    "gis",
    "traffic",
    "competition",
    "commercial_activity",
    "edgeai_telemetry",
  ];
  return ALL.map((source) => ({
    source,
    label: COVERAGE_SOURCE_LABELS[source],
    present: present.includes(source),
  }));
}

const DEFAULT_PARAMS = {
  domain: "specialty_coffee" as BusinessConcept,
  operatingHours: { open: "08:00", close: "22:00", daysPerWeek: 6 },
  staffCount: 5,
  budget: 500000,
  currency: "EGP",
};

// ── New type ──────────────────────────────────────────────────────────────────

export interface ReportScenario {
  assessment: ConceptAssessment;
  asset: Asset;
}

// ── Strong scenario — Nasr City, Specialty Coffee ─────────────────────────────
// Valuation: present (below market, high confidence)
// Temporal: 6/6 sources, full activity data, 14-day forecast horizon

const nasrCityAsset: Asset = {
  id: "asset-nasr-city-coffee-11a",
  coordinates: { lat: 30.0677, lng: 31.3391 },
  address: "Abbas El Akkad St, Nasr City",
  area: "Nasr City",
  size: 72,
  monthlyRent: 32000,
  currency: "EGP",
  propertyType: "retail_unit",
  availabilityStatus: "available",
  listingSource: "seeded",
  coverageScore: 88,
};

export const strongScenario: ReportScenario = {
  asset: nasrCityAsset,
  assessment: {
    assetId: "asset-nasr-city-coffee-11a",
    concept: "specialty_coffee",
    parameters: DEFAULT_PARAMS,
    generatedAt: "2026-06-03T08:05:00Z",
    feasibility: {
      feasibilityScore: 82,
      successLikelihood: 0.74,
      confidenceInterval: { low: 76, high: 88 },
      scoreBand: "strong",
      contributingFactors: [
        { label: "Pedestrian footfall", value: "14,800 / day", direction: "positive", weight: 0.94 },
        { label: "Evening activity density", value: "High", direction: "positive", weight: 0.86 },
        { label: "Demographic match", value: "Index 0.88", direction: "positive", weight: 0.82 },
        { label: "Direct competitor density", value: "3 within 400 m", direction: "neutral", weight: 0.64 },
        { label: "Rent index", value: "EGP 340 / m²", direction: "neutral", weight: 0.56 },
        { label: "Parking availability", value: "Moderate", direction: "positive", weight: 0.44 },
      ],
      narrative:
        "Nasr City is the strongest current specialty coffee candidate. Activity density is high across daytime and evening windows, the customer profile fits a premium repeat-purchase format, and competition is present but not saturated. The main diligence item is rent sensitivity on Abbas El Akkad frontage.",
      labelQuality: "high",
      dataSources: [
        "Aggregate mobile movement data (Q1–Q2 2026)",
        "Business registry – Cairo Governorate",
        "OSM point-of-interest layer (rev. May 2026)",
        "Rental index – Cairo retail corridors Q1 2026",
        "Demographic model – CAPMAS 2024 census interpolation",
      ],
      dataPointCount: 1024,
      generatedAt: "2026-06-03T08:05:00Z",
    },
    valuation: {
      mode: "listing_evaluation",
      verdict: "underpriced",
      fairPriceEstimate: 35000,
      fairValueRange: { low: 32500, high: 37500 },
      expectedValue: 36500,
      currency: "EGP",
      confidence: "high",
      priceDelta: "below_market",
      valuationGapPct: -8.6,
      drivers: [
        { label: "Commercial potential", detail: "Feasibility 82/100 (strong) — raises fair value", direction: "positive" },
        { label: "Price vs district benchmark", detail: "Listed EGP 444/m² vs benchmark EGP 380/m²", direction: "positive" },
        { label: "District quality", detail: "Prime commercial district", direction: "positive" },
      ],
      valuationNarrative:
        "The listed EGP 32,000/mo rent sits below the estimated fair range of EGP 32,500–37,500/mo: the Abbas El Akkad corridor's prime positioning and strong commercial potential (feasibility 82/100) support a fair value above the asking rent. This is a reasoned-prior estimate (Phase 1), not a transaction-calibrated valuation.",
      generatedAt: "2026-06-03T08:10:00Z",
    },
    temporal: {
      coverageScore: 88,
      sourceChecklist: buildChecklist([
        "demographics", "gis", "traffic", "competition", "commercial_activity", "edgeai_telemetry",
      ]),
      activityProfile: [
        { hour: 9,  dayOfWeek: 1, relativeActivity: 0.55 },
        { hour: 12, dayOfWeek: 1, relativeActivity: 0.82 },
        { hour: 15, dayOfWeek: 1, relativeActivity: 0.71 },
        { hour: 18, dayOfWeek: 1, relativeActivity: 0.94 },
        { hour: 20, dayOfWeek: 1, relativeActivity: 0.88 },
        { hour: 13, dayOfWeek: 5, relativeActivity: 0.96 },
        { hour: 19, dayOfWeek: 5, relativeActivity: 0.99 },
        { hour: 14, dayOfWeek: 6, relativeActivity: 0.91 },
      ],
      peakWindows: [
        { label: "Friday evening (18:00–21:00)", dayPattern: "Friday", peakHours: [18, 21], intensity: "high" },
        { label: "Weekday lunch (12:00–14:00)", dayPattern: "Monday–Thursday", peakHours: [12, 14], intensity: "moderate" },
      ],
      forecastHorizon: "14 days",
      temporalConfidence: "high",
    },
  },
};

// ── Moderate scenario — Heliopolis, Specialty Coffee ──────────────────────────
// Valuation: present but low confidence (low source count)
// Temporal: 5/6 sources (no EdgeAI), null activityProfile

const heliopolisAsset: Asset = {
  id: "asset-heliopolis-2b",
  coordinates: { lat: 30.0936, lng: 31.3219 },
  address: "8 Al-Ahrar St, Heliopolis",
  area: "Heliopolis",
  size: 68,
  monthlyRent: 24000,
  currency: "EGP",
  propertyType: "ground_floor_commercial",
  availabilityStatus: "available",
  listingSource: "seeded",
  coverageScore: 74,
};

export const moderateScenario: ReportScenario = {
  asset: heliopolisAsset,
  assessment: {
    assetId: "asset-heliopolis-2b",
    concept: "specialty_coffee",
    parameters: DEFAULT_PARAMS,
    generatedAt: "2026-06-02T11:15:00Z",
    feasibility: {
      feasibilityScore: 54,
      successLikelihood: 0.48,
      confidenceInterval: { low: 46, high: 62 },
      scoreBand: "moderate",
      contributingFactors: [
        { label: "Pedestrian footfall", value: "7,100 / day", direction: "positive", weight: 0.88 },
        { label: "Direct competitor density", value: "5 within 400 m", direction: "negative", weight: 0.81 },
        { label: "Demographic match", value: "Index 0.71", direction: "positive", weight: 0.64 },
        { label: "Rent index", value: "EGP 280 / m²", direction: "neutral", weight: 0.49 },
        { label: "Transit proximity", value: "Metro: 1.1 km", direction: "negative", weight: 0.33 },
      ],
      narrative:
        "This location presents a moderate opportunity. Footfall is solid but competition is denser than ideal — five comparable outlets within 400 metres dilute the available customer base. The demographic fit is good; the transit gap (no metro within 1 km) limits lunchtime commuter traffic. A specialty operator could differentiate on quality, but the margin for error is narrower here than in Maadi.",
      labelQuality: "high",
      dataSources: [
        "Aggregate mobile movement data (Q1–Q2 2026)",
        "Business registry – Cairo Governorate",
        "OSM point-of-interest layer (rev. May 2026)",
      ],
      dataPointCount: 612,
      generatedAt: "2026-06-02T11:15:00Z",
    },
    valuation: {
      mode: "listing_evaluation",
      verdict: "fairly_priced",
      fairPriceEstimate: 23000,
      fairValueRange: { low: 19500, high: 26500 },
      expectedValue: 24000,
      currency: "EGP",
      confidence: "low",
      priceDelta: "at_market",
      valuationGapPct: 4.3,
      drivers: [
        { label: "Commercial potential", detail: "Feasibility 54/100 (moderate) — neutral to fair value", direction: "neutral" },
        { label: "Price vs district benchmark", detail: "Listed EGP 353/m² vs benchmark EGP 380/m²", direction: "neutral" },
        { label: "Competitor density", detail: "5 within 400 m", direction: "negative" },
      ],
      valuationNarrative:
        "The listed EGP 24,000/mo rent falls within the estimated fair range of EGP 19,500–26,500/mo. Confidence is low: rental comparables in the Al-Ahrar corridor are sparse, so the range is wide and the verdict indicative. This is a reasoned-prior estimate (Phase 1), not a transaction-calibrated valuation.",
      generatedAt: "2026-06-02T11:20:00Z",
    },
    temporal: {
      coverageScore: 74,
      sourceChecklist: buildChecklist([
        "demographics", "gis", "traffic", "competition", "commercial_activity",
        // edgeai_telemetry: absent
      ]),
      activityProfile: null,
      peakWindows: null,
      forecastHorizon: null,
      temporalConfidence: null,
    },
  },
};

// ── Weak scenario — Imbaba, Specialty Coffee ──────────────────────────────────
// Valuation: null (insufficient data — exercises honest-absence state)
// Temporal: 2/6 sources, null activityProfile

const imbabaAsset: Asset = {
  id: "asset-imbaba-3f",
  coordinates: { lat: 30.0808, lng: 31.2139 },
  address: "Teraa al-Zumor, Imbaba",
  area: "Imbaba",
  size: 55,
  monthlyRent: 11000,
  currency: "EGP",
  propertyType: "retail_unit",
  availabilityStatus: "available",
  listingSource: "seeded",
  coverageScore: 52,
};

export const weakScenario: ReportScenario = {
  asset: imbabaAsset,
  assessment: {
    assetId: "asset-imbaba-3f",
    concept: "specialty_coffee",
    parameters: DEFAULT_PARAMS,
    generatedAt: "2026-06-02T09:44:00Z",
    feasibility: {
      feasibilityScore: 31,
      successLikelihood: 0.24,
      confidenceInterval: { low: 24, high: 40 },
      scoreBand: "weak",
      contributingFactors: [
        { label: "Pedestrian footfall", value: "3,200 / day", direction: "negative", weight: 0.90 },
        { label: "Demographic match", value: "Index 0.38", direction: "negative", weight: 0.85 },
        { label: "Direct competitor density", value: "1 within 400 m", direction: "positive", weight: 0.60 },
        { label: "Rent index", value: "EGP 180 / m²", direction: "positive", weight: 0.50 },
        { label: "Transit proximity", value: "Metro: 2.3 km", direction: "negative", weight: 0.44 },
      ],
      narrative:
        "This location scores weakly for a Specialty Coffee & Café. Footfall is low relative to the format's minimum viable threshold, and the demographic profile does not match the format's typical customer base. The low rent softens the economic downside but does not offset the structural demand gap. Not recommended for this format at this location.",
      labelQuality: "medium",
      dataSources: [
        "Aggregate mobile movement data (Q1–Q2 2026)",
        "Demographic model – CAPMAS 2024 census interpolation",
      ],
      dataPointCount: 389,
      generatedAt: "2026-06-02T09:44:00Z",
    },
    valuation: null,
    temporal: {
      coverageScore: 52,
      sourceChecklist: buildChecklist([
        "demographics", "traffic",
        // gis, competition, commercial_activity, edgeai_telemetry: all absent
      ]),
      activityProfile: null,
      peakWindows: null,
      forecastHorizon: null,
      temporalConfidence: null,
    },
  },
};

