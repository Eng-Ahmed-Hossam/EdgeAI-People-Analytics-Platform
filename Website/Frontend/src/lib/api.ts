// ─────────────────────────────────────────────────────────────────────────────
// BACKEND INTEGRATION SEAM (SYSTEM_DESIGN.md §5.2)
//
// This module is the ONLY place in the frontend that talks to the backend.
// Everything the production architecture describes — intelligence services, data
// platform, confidence engine, streaming infrastructure — is invisible to the
// frontend and lives entirely behind this seam.
//
// Backend integration status (Session 1 — Asset Service):
//   getAssets()      → REAL  — GET /api/assets from FastAPI + PostgreSQL/PostGIS
//   getAssetById()   → REAL  — GET /api/assets/{id}
//   All other functions remain mock (assessment, valuation, temporal — future sessions)
//
// Configure: set NEXT_PUBLIC_API_URL in frontend/.env.local
//   NEXT_PUBLIC_API_URL=http://localhost:8000
//
// Every consumer fetches through this module via TanStack Query — never inline.
// ─────────────────────────────────────────────────────────────────────────────

import type { Site } from "@/types/site";
import type { BusinessParameters } from "@/types/parameters";
import type { FeasibilityReport } from "@/types/report";

// New imports for the asset-centric model
import type { Asset } from "@/types/asset";
import type { SavedAssessmentRecord } from "@/types/saved-assessment";
import type {
  BusinessConcept,
  ConceptAssessment,
  CoverageSourceKey,
  CoverageSourceStatus,
  FeasibilityOutput,
  ValuationOutput,
  TemporalOutput,
} from "@/types/assessment-v2";
import { COVERAGE_SOURCE_LABELS } from "@/types/assessment-v2";
import type {
  AssessmentPipelineItem,
  ComparativeIntelligenceData,
  ConfidenceLevel,
  ExecutiveDashboardData,
} from "@/types/assessment";

// ── Backend base URL (configurable; falls back to local FastAPI dev server) ──
// Set NEXT_PUBLIC_API_URL in frontend/.env.local for any non-default host.
const API_BASE = process.env.NEXT_PUBLIC_API_URL ?? "http://localhost:8000";

// Simulated network delay for mock functions (removed when each function goes real)
const delay = (ms: number) =>
  new Promise<void>((resolve) => setTimeout(resolve, ms));

// ── Fixture data covering all three score bands ───────────────────────────────

const SITES: Site[] = [
  {
    id: "site-maadi-corniche-4a",
    name: "12 Corniche el-Maadi, Maadi",
    coordinates: { lat: 29.9602, lng: 31.2569 },
    previewScore: 78,
    status: "scored",
  },
  {
    id: "site-nasr-city-coffee-11a",
    name: "Abbas El Akkad Corridor, Nasr City",
    coordinates: { lat: 30.0606, lng: 31.3373 },
    previewScore: 82,
    status: "scored",
  },
  {
    id: "site-new-cairo-restaurant-5e",
    name: "South 90 St, New Cairo",
    coordinates: { lat: 30.0131, lng: 31.4934 },
    previewScore: 74,
    status: "scored",
  },
  {
    id: "site-october-pharmacy-6p",
    name: "Central Axis, 6th of October",
    coordinates: { lat: 29.9723, lng: 30.9361 },
    previewScore: 69,
    status: "scored",
  },
  {
    id: "site-sheikh-zayed-retail-8s",
    name: "Beverly Hills Gate, Sheikh Zayed",
    coordinates: { lat: 30.0496, lng: 30.9769 },
    previewScore: 63,
    status: "scored",
  },
  {
    id: "site-heliopolis-2b",
    name: "8 Al-Ahrar St, Heliopolis",
    coordinates: { lat: 30.0936, lng: 31.3219 },
    previewScore: 54,
    status: "scored",
  },
  {
    id: "site-imbaba-3f",
    name: "Teraa al-Zumor, Imbaba",
    coordinates: { lat: 30.0808, lng: 31.2139 },
    previewScore: 31,
    status: "scored",
  },
  {
    id: "site-fayoum-new-dist-7c",
    name: "Block 7, New Fayoum District",
    coordinates: { lat: 29.3084, lng: 30.8428 },
    status: "insufficient",
  },
  {
    id: "site-zamalek-9d",
    name: "26 July St, Zamalek",
    coordinates: { lat: 30.0619, lng: 31.2235 },
    status: "pending",
  },
];

const RESULTS: Record<string, FeasibilityReport> = {
  "site-maadi-corniche-4a": {
    siteId: "site-maadi-corniche-4a",
    siteName: "12 Corniche el-Maadi, Maadi",
    siteType: "Specialty Coffee & Café",
    coordinates: { lat: 29.9602, lng: 31.2569 },
    generatedAt: "2026-06-02T14:32:00Z",
    feasibilityScore: 78,
    scoreBand: "strong",
    confidenceInterval: { low: 72, high: 84 },
    revenueRange: {
      low: 85000,
      high: 140000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 97500,
      benchmarkLabel: "Maadi café median",
    },
    contributingFactors: [
      { label: "Pedestrian footfall", value: "12,300 / day", direction: "positive", weight: 0.92 },
      { label: "Direct competitor density", value: "2 within 400 m", direction: "positive", weight: 0.78 },
      { label: "Demographic match", value: "Index 0.84", direction: "positive", weight: 0.71 },
      { label: "Rent index", value: "EGP 320 / m²", direction: "neutral", weight: 0.55 },
      { label: "Parking availability", value: "Limited", direction: "negative", weight: 0.42 },
      { label: "Transit proximity", value: "Metro: 640 m", direction: "neutral", weight: 0.38 },
    ],
    narrative:
      "This location scores strongly against the Specialty Coffee & Café profile. Pedestrian counts on the Corniche el-Maadi strip are among the district's highest. The competitive set is thin — two established cafés within 400 metres, neither operating a specialty-roasting model. The primary drag is parking: limited short-term stopping may constrain weekend revenue from outer-district visitors.",
    labelQuality: "high",
    dataSources: [
      "Aggregate mobile movement data (Q1–Q2 2026)",
      "Business registry – Cairo Governorate",
      "OSM point-of-interest layer (rev. May 2026)",
      "Rental index – JLL Egypt Q1 2026",
      "Demographic model – CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 847,
  },

  "site-nasr-city-coffee-11a": {
    siteId: "site-nasr-city-coffee-11a",
    siteName: "Abbas El Akkad Corridor, Nasr City",
    siteType: "Specialty Coffee & Cafe",
    coordinates: { lat: 30.0606, lng: 31.3373 },
    generatedAt: "2026-06-03T08:05:00Z",
    feasibilityScore: 82,
    scoreBand: "strong",
    confidenceInterval: { low: 76, high: 88 },
    revenueRange: {
      low: 96000,
      high: 158000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 103000,
      benchmarkLabel: "Nasr City cafe median",
    },
    contributingFactors: [
      { label: "Pedestrian footfall", value: "14,800 / day", direction: "positive", weight: 0.94 },
      { label: "Evening activity density", value: "High", direction: "positive", weight: 0.86 },
      { label: "Demographic match", value: "Index 0.88", direction: "positive", weight: 0.82 },
      { label: "Direct competitor density", value: "3 within 400 m", direction: "neutral", weight: 0.64 },
      { label: "Rent index", value: "EGP 340 / m2", direction: "neutral", weight: 0.56 },
      { label: "Parking availability", value: "Moderate", direction: "positive", weight: 0.44 },
    ],
    narrative:
      "Nasr City is the strongest current specialty coffee candidate. Activity density is high across daytime and evening windows, the customer profile fits a premium repeat-purchase format, and competition is present but not saturated. The main diligence item is rent sensitivity on Abbas El Akkad frontage.",
    labelQuality: "high",
    dataSources: [
      "Aggregate mobile movement data (Q1-Q2 2026)",
      "Business registry - Cairo Governorate",
      "OSM point-of-interest layer (rev. May 2026)",
      "Rental index - Cairo retail corridors Q1 2026",
      "Demographic model - CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 1024,
  },

  "site-new-cairo-restaurant-5e": {
    siteId: "site-new-cairo-restaurant-5e",
    siteName: "South 90 St, New Cairo",
    siteType: "Fast Casual Restaurant",
    coordinates: { lat: 30.0131, lng: 31.4934 },
    generatedAt: "2026-06-03T07:42:00Z",
    feasibilityScore: 74,
    scoreBand: "strong",
    confidenceInterval: { low: 67, high: 81 },
    revenueRange: {
      low: 135000,
      high: 220000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 162000,
      benchmarkLabel: "New Cairo fast casual median",
    },
    contributingFactors: [
      { label: "Evening destination traffic", value: "Strong", direction: "positive", weight: 0.9 },
      { label: "Household income match", value: "Index 0.86", direction: "positive", weight: 0.78 },
      { label: "Parking availability", value: "Good", direction: "positive", weight: 0.7 },
      { label: "Direct competitor density", value: "7 within 600 m", direction: "negative", weight: 0.68 },
      { label: "Rent index", value: "EGP 410 / m2", direction: "negative", weight: 0.6 },
    ],
    narrative:
      "New Cairo presents a strong restaurant opportunity with high-income catchment fit and strong evening destination traffic. The upside is meaningful, but competition and rent pressure require careful positioning and lease diligence.",
    labelQuality: "high",
    dataSources: [
      "Aggregate mobile movement data (Q1-Q2 2026)",
      "Business registry - Cairo Governorate",
      "OSM point-of-interest layer (rev. May 2026)",
      "Rental index - New Cairo retail corridors Q1 2026",
      "Demographic model - CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 916,
  },

  "site-october-pharmacy-6p": {
    siteId: "site-october-pharmacy-6p",
    siteName: "Central Axis, 6th of October",
    siteType: "Pharmacy",
    coordinates: { lat: 29.9723, lng: 30.9361 },
    generatedAt: "2026-06-03T06:50:00Z",
    feasibilityScore: 69,
    scoreBand: "moderate",
    confidenceInterval: { low: 60, high: 77 },
    revenueRange: {
      low: 78000,
      high: 128000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 94000,
      benchmarkLabel: "October pharmacy median",
    },
    contributingFactors: [
      { label: "Residential catchment", value: "Dense", direction: "positive", weight: 0.86 },
      { label: "Healthcare adjacency", value: "2 clinics nearby", direction: "positive", weight: 0.74 },
      { label: "Direct competitor density", value: "4 within 500 m", direction: "negative", weight: 0.66 },
      { label: "Transit proximity", value: "Bus corridor: 220 m", direction: "positive", weight: 0.55 },
      { label: "Night activity", value: "Limited", direction: "negative", weight: 0.38 },
    ],
    narrative:
      "The October pharmacy candidate has a usable residential and healthcare catchment, but competition is visible and late-night demand is not proven. The site should advance only after operator-hours and service differentiation assumptions are confirmed.",
    labelQuality: "high",
    dataSources: [
      "Aggregate mobile movement data (Q1-Q2 2026)",
      "Business registry - Giza Governorate",
      "OSM point-of-interest layer (rev. May 2026)",
      "Demographic model - CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 704,
  },

  "site-sheikh-zayed-retail-8s": {
    siteId: "site-sheikh-zayed-retail-8s",
    siteName: "Beverly Hills Gate, Sheikh Zayed",
    siteType: "Premium Retail Store",
    coordinates: { lat: 30.0496, lng: 30.9769 },
    generatedAt: "2026-06-03T06:15:00Z",
    feasibilityScore: 63,
    scoreBand: "moderate",
    confidenceInterval: { low: 53, high: 72 },
    revenueRange: {
      low: 92000,
      high: 165000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 128000,
      benchmarkLabel: "Sheikh Zayed premium retail median",
    },
    contributingFactors: [
      { label: "Affluent catchment", value: "Index 0.9", direction: "positive", weight: 0.86 },
      { label: "Weekend destination traffic", value: "Moderate", direction: "positive", weight: 0.68 },
      { label: "Brand-fit corridor", value: "High", direction: "positive", weight: 0.64 },
      { label: "Footfall consistency", value: "Variable", direction: "negative", weight: 0.58 },
      { label: "Rent index", value: "EGP 390 / m2", direction: "negative", weight: 0.52 },
    ],
    narrative:
      "Sheikh Zayed is a credible premium retail candidate but not yet investment-ready. Customer fit is strong, while weekday traffic consistency and rent exposure remain the main constraints.",
    labelQuality: "medium",
    dataSources: [
      "Aggregate mobile movement data (Q1-Q2 2026)",
      "OSM point-of-interest layer (rev. May 2026)",
      "Rental index - West Cairo retail corridors Q1 2026",
      "Demographic model - CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 638,
  },

  // Moderate band
  "site-heliopolis-2b": {
    siteId: "site-heliopolis-2b",
    siteName: "8 Al-Ahrar St, Heliopolis",
    siteType: "Specialty Coffee & Café",
    coordinates: { lat: 30.0936, lng: 31.3219 },
    generatedAt: "2026-06-02T11:15:00Z",
    feasibilityScore: 54,
    scoreBand: "moderate",
    confidenceInterval: { low: 46, high: 62 },
    revenueRange: {
      low: 52000,
      high: 89000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 97500,
      benchmarkLabel: "Cairo café median",
    },
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
  },

  // Weak band
  "site-imbaba-3f": {
    siteId: "site-imbaba-3f",
    siteName: "Teraa al-Zumor, Imbaba",
    siteType: "Specialty Coffee & Café",
    coordinates: { lat: 30.0808, lng: 31.2139 },
    generatedAt: "2026-06-02T09:44:00Z",
    feasibilityScore: 31,
    scoreBand: "weak",
    confidenceInterval: { low: 24, high: 40 },
    revenueRange: {
      low: 18000,
      high: 41000,
      currency: "EGP",
      period: "monthly",
      categoryBenchmark: 97500,
      benchmarkLabel: "Cairo café median",
    },
    contributingFactors: [
      { label: "Pedestrian footfall", value: "3,200 / day", direction: "negative", weight: 0.90 },
      { label: "Demographic match", value: "Index 0.38", direction: "negative", weight: 0.85 },
      { label: "Direct competitor density", value: "1 within 400 m", direction: "positive", weight: 0.60 },
      { label: "Rent index", value: "EGP 180 / m²", direction: "positive", weight: 0.50 },
      { label: "Transit proximity", value: "Metro: 2.3 km", direction: "negative", weight: 0.44 },
    ],
    narrative:
      "This location scores weakly for a Specialty Coffee & Café. Footfall is low relative to the format's minimum viable threshold, and the demographic profile — predominantly lower-income households with low discretionary spending — does not match the format's typical customer base. The low rent softens the economic downside but does not offset the structural demand gap. Not recommended for this format at this location.",
    labelQuality: "medium",
    dataSources: [
      "Aggregate mobile movement data (Q1–Q2 2026)",
      "Demographic model – CAPMAS 2024 census interpolation",
    ],
    dataPointCount: 389,
  },
};

const EXTRA_SITES: Site[] = [
  {
    id: "site-dokki-8c",
    name: "Mossadak St, Dokki",
    coordinates: { lat: 30.0384, lng: 31.2095 },
    previewScore: 64,
    status: "pending",
  },
];

function confidenceLevel(report: FeasibilityReport): ConfidenceLevel {
  const intervalWidth = report.confidenceInterval.high - report.confidenceInterval.low;
  const sourceStrength = report.dataSources.length >= 4;

  if (intervalWidth <= 14 && sourceStrength && report.labelQuality === "high") {
    return "high";
  }

  if (intervalWidth <= 20 && report.labelQuality !== "low") {
    return "medium";
  }

  return "low";
}

const DASHBOARD_ASSESSMENTS: AssessmentPipelineItem[] = [
  {
    id: "asm-nasr-city-coffee-11a",
    site: SITES[1],
    businessType: RESULTS["site-nasr-city-coffee-11a"].siteType,
    status: "completed",
    updatedAt: "2026-06-03T09:20:00Z",
    summary:
      "Highest-confidence specialty coffee opportunity with strong activity density and premium customer fit.",
    nextStep: "Validate rent sensitivity and frontage conditions before lease negotiation.",
    report: RESULTS["site-nasr-city-coffee-11a"],
    confidenceLevel: confidenceLevel(RESULTS["site-nasr-city-coffee-11a"]),
  },
  {
    id: "asm-maadi-corniche-4a",
    site: SITES[0],
    businessType: RESULTS["site-maadi-corniche-4a"].siteType,
    status: "completed",
    updatedAt: "2026-06-03T08:40:00Z",
    summary:
      "Strong specialty coffee candidate with high pedestrian activity and limited direct competition.",
    nextStep: "Proceed to field validation before lease negotiation.",
    report: RESULTS["site-maadi-corniche-4a"],
    confidenceLevel: confidenceLevel(RESULTS["site-maadi-corniche-4a"]),
  },
  {
    id: "asm-new-cairo-restaurant-5e",
    site: SITES[2],
    businessType: RESULTS["site-new-cairo-restaurant-5e"].siteType,
    status: "completed",
    updatedAt: "2026-06-03T07:55:00Z",
    summary:
      "Strong restaurant opportunity with high-income demand and meaningful evening traffic.",
    nextStep: "Stress-test rent and differentiation assumptions before capital planning.",
    report: RESULTS["site-new-cairo-restaurant-5e"],
    confidenceLevel: confidenceLevel(RESULTS["site-new-cairo-restaurant-5e"]),
  },
  {
    id: "asm-october-pharmacy-6p",
    site: SITES[3],
    businessType: RESULTS["site-october-pharmacy-6p"].siteType,
    status: "completed",
    updatedAt: "2026-06-03T07:12:00Z",
    summary:
      "Moderate pharmacy candidate supported by residential density and nearby healthcare demand.",
    nextStep: "Validate competitor hours and operator service model before prioritizing.",
    report: RESULTS["site-october-pharmacy-6p"],
    confidenceLevel: confidenceLevel(RESULTS["site-october-pharmacy-6p"]),
  },
  {
    id: "asm-sheikh-zayed-retail-8s",
    site: SITES[4],
    businessType: RESULTS["site-sheikh-zayed-retail-8s"].siteType,
    status: "ready_for_analysis",
    updatedAt: "2026-06-02T18:45:00Z",
    summary:
      "Premium retail candidate with strong customer fit but variable weekday footfall.",
    nextStep: "Review weekday activity before treating the site as investment-ready.",
    report: RESULTS["site-sheikh-zayed-retail-8s"],
    confidenceLevel: confidenceLevel(RESULTS["site-sheikh-zayed-retail-8s"]),
  },
  {
    id: "asm-heliopolis-2b",
    site: SITES[5],
    businessType: RESULTS["site-heliopolis-2b"].siteType,
    status: "completed",
    updatedAt: "2026-06-02T16:25:00Z",
    summary:
      "Moderate opportunity; good demand fit is offset by competitor concentration.",
    nextStep: "Compare against Maadi and Dokki before prioritizing.",
    report: RESULTS["site-heliopolis-2b"],
    confidenceLevel: confidenceLevel(RESULTS["site-heliopolis-2b"]),
  },
  {
    id: "asm-dokki-8c",
    site: EXTRA_SITES[0],
    businessType: "Coworking",
    status: "draft",
    updatedAt: "2026-06-02T13:10:00Z",
    summary:
      "Central urban site with office catchment potential; assessment has not been parameterized.",
    nextStep: "Select target business model and budget assumptions.",
    confidenceLevel: "low",
  },
  {
    id: "asm-imbaba-3f",
    site: SITES[6],
    businessType: RESULTS["site-imbaba-3f"].siteType,
    status: "needs_review",
    updatedAt: "2026-06-02T10:05:00Z",
    summary:
      "Weak format fit driven by low footfall and poor demographic alignment.",
    nextStep: "Review risk assumptions and redirect analysis to alternative sites.",
    report: RESULTS["site-imbaba-3f"],
    confidenceLevel: confidenceLevel(RESULTS["site-imbaba-3f"]),
    attentionReason: "High-risk recommendation should be reviewed before sharing.",
  },
  {
    id: "asm-zamalek-9d",
    site: SITES[8],
    businessType: "Boutique Retail",
    status: "awaiting_inputs",
    updatedAt: "2026-06-01T18:45:00Z",
    summary:
      "Premium retail corridor with strong brand fit; source coverage is pending final refresh.",
    nextStep: "Complete business assumptions after coverage refresh completes.",
    confidenceLevel: "medium",
  },
  {
    id: "asm-fayoum-new-dist-7c",
    site: SITES[7],
    businessType: "Specialty Coffee & Cafe",
    status: "needs_review",
    updatedAt: "2026-06-01T11:20:00Z",
    summary:
      "Coverage gap: movement data remains below the minimum defensible scoring threshold.",
    nextStep: "Hold recommendation until coverage improves.",
    confidenceLevel: "low",
    attentionReason: "Insufficient data coverage.",
  },
];

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Returns executive dashboard intelligence for the operational home screen.
 * Real backend: GET /api/dashboard/executive
 */
export async function getExecutiveDashboard(): Promise<ExecutiveDashboardData> {
  await delay(600);

  const completed = DASHBOARD_ASSESSMENTS.filter(
    (assessment) => assessment.status === "completed"
  );
  const scoredReports = DASHBOARD_ASSESSMENTS.flatMap((assessment) =>
    assessment.report ? [assessment.report] : []
  );
  const averageScore = Math.round(
    scoredReports.reduce((sum, report) => sum + report.feasibilityScore, 0) /
      scoredReports.length
  );
  const highConfidence = DASHBOARD_ASSESSMENTS.filter(
    (assessment) => assessment.confidenceLevel === "high"
  ).length;
  const needsReview = DASHBOARD_ASSESSMENTS.filter(
    (assessment) => assessment.status === "needs_review"
  ).length;

  return {
    summary: {
      activeAssessments: DASHBOARD_ASSESSMENTS.length,
      completedAssessments: completed.length,
      highConfidenceOpportunities: highConfidence,
      needsReview,
      intelligenceSummary:
        "Nasr City is the highest-confidence opportunity in the current pipeline. New Cairo and Maadi are strong follow-up candidates, while Imbaba and Fayoum require review before recommendation sharing.",
    },
    opportunitySpotlight: DASHBOARD_ASSESSMENTS.filter(
      (assessment) =>
        assessment.status === "completed" ||
        assessment.status === "ready_for_analysis"
    )
      .sort((a, b) => (b.site.previewScore ?? 0) - (a.site.previewScore ?? 0))
      .slice(0, 5),
    pipeline: [
      {
        status: "draft",
        label: "Draft",
        count: DASHBOARD_ASSESSMENTS.filter((item) => item.status === "draft")
          .length,
        description: "Locations saved but not yet parameterized.",
      },
      {
        status: "awaiting_inputs",
        label: "Awaiting Inputs",
        count: DASHBOARD_ASSESSMENTS.filter(
          (item) => item.status === "awaiting_inputs"
        ).length,
        description: "Business model assumptions need completion.",
      },
      {
        status: "ready_for_analysis",
        label: "Ready for Analysis",
        count: DASHBOARD_ASSESSMENTS.filter(
          (item) => item.status === "ready_for_analysis"
        ).length,
        description: "Candidate sites can be scored next.",
      },
      {
        status: "completed",
        label: "Completed",
        count: completed.length,
        description: "Assessments with score, interval, and recommendation.",
      },
      {
        status: "needs_review",
        label: "Needs Review",
        count: needsReview,
        description: "Items requiring executive or data-quality attention.",
      },
    ],
    intelligenceOverview: [
      {
        label: "Average Feasibility",
        value: `${averageScore}`,
        detail: "Across completed scored assessments",
        trend: "up",
      },
      {
        label: "Average Confidence",
        value: "84%",
        detail: "Weighted by source coverage and interval width",
        trend: "flat",
      },
      {
        label: "Location Coverage",
        value: "92%",
        detail: "Greater Cairo demonstration grid",
        trend: "up",
      },
      {
        label: "Assessment Activity",
        value: "+8",
        detail: "New or updated assessments this week",
        trend: "up",
      },
      {
        label: "Geographic Spread",
        value: "8",
        detail: "Districts represented in active pipeline",
        trend: "flat",
      },
    ],
    recentAssessments: [...DASHBOARD_ASSESSMENTS].sort(
      (a, b) =>
        new Date(b.updatedAt).getTime() - new Date(a.updatedAt).getTime()
    ),
    recommendedActions: [
      {
        id: "action-validate-nasr-city",
        priority: "high",
        title: "Validate Nasr City lead opportunity",
        detail:
          "The strongest current specialty coffee candidate should move to frontage, rent, and peak-period validation.",
        href: "/workspace",
        actionLabel: "Open workspace",
      },
      {
        id: "action-compare-top",
        priority: "high",
        title: "Compare top-performing candidate locations",
        detail:
          "Nasr City, Maadi, and New Cairo should be reviewed as the primary investor demo set.",
        href: "/compare",
        actionLabel: "Compare sites",
      },
      {
        id: "action-review-imbaba",
        priority: "medium",
        title: "Review Imbaba high-risk assessment",
        detail:
          "Weak footfall and demographic fit should be reviewed before this recommendation is shared.",
        href: "/report",
        actionLabel: "Open report",
      },
      {
        id: "action-complete-inputs",
        priority: "medium",
        title: "Complete missing business inputs",
        detail:
          "Dokki and Zamalek need operating assumptions before feasibility can be trusted.",
        href: "/workspace",
        actionLabel: "Continue analysis",
      },
    ],
    platformStatus: {
      dataCoverage: 92,
      telemetryStatus: "simulated",
      sourceLayers: [
        "GIS layers",
        "Demographic model",
        "Commercial registry",
        "Movement aggregates",
        "Rental index",
        "EdgeAI telemetry seam",
      ],
      confidenceIndicators: [
        { label: "Backend seam", value: "Stable", status: "ready" },
        { label: "Telemetry intake", value: "Designed", status: "future" },
        { label: "Prediction contract", value: "Locked", status: "ready" },
        { label: "Label loop", value: "MVP path", status: "active" },
      ],
    },
  };
}

// ── TanStack Query keys ───────────────────────────────────────────────────────

export const queryKeys = {
  // NEW (asset-centric)
  assets: ["assets"] as const,
  asset: (assetId: string) => ["assets", assetId] as const,
  assessmentsForAsset: (assetId: string) =>
    ["assessments", assetId] as const,
  feasibilityForConcept: (assetId: string, concept: string) =>
    ["feasibility", assetId, concept] as const,
  valuation: (assetId: string) =>
    ["valuation", assetId] as const,
  temporal: (assetId: string) =>
    ["temporal", assetId] as const,
  // Persistence (saved assessments)
  savedAssessments: ["savedAssessments"] as const,
  savedAssessment: (id: string) => ["savedAssessments", id] as const,
  // Dashboard / comparison (purpose unchanged; shape may evolve)
  dashboard: ["dashboard", "executive"] as const,
  comparison: ["comparison", "executive"] as const,
};

/**
 * Returns the executive comparison brief for candidate locations.
 * Real backend: GET /api/comparison/executive
 */
export async function getComparativeAnalysis(): Promise<ComparativeIntelligenceData> {
  await delay(650);

  const candidateA = {
    site: SITES[1],
    report: RESULTS["site-nasr-city-coffee-11a"],
  };
  const candidateB = {
    site: SITES[5],
    report: RESULTS["site-heliopolis-2b"],
  };

  return {
    businessConcept: "Specialty Coffee & Cafe",
    generatedAt: "2026-06-03T09:10:00Z",
    candidates: [candidateA, candidateB],
    decision: {
      winnerSiteId: candidateA.site.id,
      headline: "Nasr City is the stronger location for this concept.",
      recommendation:
        "Prioritize Nasr City for field validation and lease diligence before committing capital to Heliopolis.",
      confidence: "high",
      why: [
        "Higher feasibility score with a tighter confidence interval.",
        "Better demographic fit for a specialty coffee format.",
        "Stronger all-day activity density with more manageable competition pressure.",
      ],
      tradeoffs: [
        "Nasr City carries slightly higher rent sensitivity on prime frontage.",
        "Heliopolis offers slightly lower rent pressure, but the demand and competition signals are weaker.",
      ],
      nextAction:
        "Open the Nasr City workspace, validate frontage quality and rent assumptions, then use the report for investment review.",
    },
    factors: [
      {
        id: "recommendation",
        label: "Recommendation",
        decisionQuestion: "Which site should advance first?",
        candidateA: {
          value: "Proceed to field validation",
          evidence:
            "Strong score band and defensible revenue upside support a near-term diligence decision.",
        },
        candidateB: {
          value: "Hold for differentiation plan",
          evidence:
            "Moderate score band requires a sharper concept or lower cost structure before commitment.",
        },
        advantage: "candidate_a",
        conclusion: "Nasr City is the clearer first-choice opportunity.",
      },
      {
        id: "score",
        label: "Feasibility Score",
        decisionQuestion: "Which location has the stronger modeled opportunity?",
        candidateA: {
          value: "82 / 100",
          evidence: "Strong band, confidence interval 76-88.",
        },
        candidateB: {
          value: "54 / 100",
          evidence: "Moderate band, confidence interval 46-62.",
        },
        advantage: "candidate_a",
        conclusion:
          "The 28-point gap is large enough to drive the executive recommendation.",
      },
      {
        id: "confidence",
        label: "Confidence",
        decisionQuestion: "Which recommendation is better supported?",
        candidateA: {
          value: "High",
          evidence: "1,024 supporting data points and five source layers.",
        },
        candidateB: {
          value: "Medium",
          evidence: "612 supporting data points and three source layers.",
        },
        advantage: "candidate_a",
        conclusion: "Nasr City has the stronger evidence base.",
      },
      {
        id: "revenue",
        label: "Revenue Range",
        decisionQuestion: "Where is the commercial upside stronger?",
        candidateA: {
          value: "EGP 96k-158k / month",
          evidence: "Upper range exceeds the Nasr City cafe median benchmark.",
        },
        candidateB: {
          value: "EGP 52k-89k / month",
          evidence: "Upper range remains below the Cairo cafe median benchmark.",
        },
        advantage: "candidate_a",
        conclusion: "Nasr City has materially stronger expected revenue capacity.",
      },
      {
        id: "competition",
        label: "Competition Context",
        decisionQuestion: "Where is the market less crowded?",
        candidateA: {
          value: "3 direct competitors within 400 m",
          evidence:
            "Competition is present but not saturated for a specialty format.",
        },
        candidateB: {
          value: "5 direct competitors within 400 m",
          evidence:
            "Comparable outlets dilute the available customer base and raise differentiation pressure.",
        },
        advantage: "candidate_a",
        conclusion:
          "Nasr City provides a clearer demand-to-competition balance.",
      },
      {
        id: "demographics",
        label: "Demographic Match",
        decisionQuestion: "Where does the local audience fit the concept?",
        candidateA: {
          value: "Index 0.88",
          evidence:
            "Higher discretionary-spend alignment for specialty coffee behavior.",
        },
        candidateB: {
          value: "Index 0.71",
          evidence:
            "Good but less concentrated match to premium cafe demand signals.",
        },
        advantage: "candidate_a",
        conclusion: "Nasr City aligns better with the target customer profile.",
      },
      {
        id: "accessibility",
        label: "Accessibility",
        decisionQuestion: "Which location is easier to capture repeatedly?",
        candidateA: {
          value: "High all-day exposure",
          evidence:
            "Abbas El Akkad activity is strong across daytime and evening windows.",
        },
        candidateB: {
          value: "Transit gap",
          evidence:
            "No metro within 1 km limits commuter and lunch-period capture.",
        },
        advantage: "candidate_a",
        conclusion:
          "Nasr City wins on activity exposure and repeat-capture potential.",
      },
      {
        id: "risk",
        label: "Risk Factors",
        decisionQuestion: "Which risk profile is more manageable?",
        candidateA: {
          value: "Rent sensitivity",
          evidence:
            "Prime frontage economics should be tested before lease commitment.",
        },
        candidateB: {
          value: "Competition concentration",
          evidence:
            "Differentiation risk is structural and harder to resolve after lease commitment.",
        },
        advantage: "candidate_a",
        conclusion:
          "Nasr City's main risk is commercial terms; Heliopolis carries a market-positioning risk.",
      },
    ],
    riskWatchlist: [
      "Validate Nasr City frontage quality, visibility, and rent sensitivity during peak windows.",
      "Confirm whether Heliopolis competitors overlap on price point, product mix, and opening hours.",
      "Do not treat either revenue range as a profit forecast; keep lease and staffing diligence separate.",
    ],
    trustBasis: [
      {
        label: "Evidence depth",
        value: "1,636",
        detail: "Combined supporting observations across both scored assessments.",
      },
      {
        label: "Source coverage",
        value: "5 layers",
        detail: "Movement, registry, POI, rental, and demographic intelligence.",
      },
      {
        label: "Decision confidence",
        value: "High",
        detail: "Winner is supported by score, interval, revenue, and market context.",
      },
      {
        label: "Telemetry seam",
        value: "Designed",
        detail: "Future EdgeAI streams can strengthen activity and access evidence.",
      },
    ],
  };
}

// ═════════════════════════════════════════════════════════════════════════════
// NEW API — Asset-centric, three-output model (SYSTEM_DESIGN.md §5.2)
// Session 13 data-layer migration
// ═════════════════════════════════════════════════════════════════════════════

// Asset fixtures removed: getAssets() now calls the real backend (GET /api/assets).
// Fixture data lives in backend/ingestion/seed_data.json and is loaded via the ingestion pipeline.

// ── Helper: build a source checklist from a set of present source keys ─────────

function buildChecklist(present: CoverageSourceKey[]): CoverageSourceStatus[] {
  const ALL_SOURCES: CoverageSourceKey[] = [
    "demographics",
    "gis",
    "traffic",
    "competition",
    "commercial_activity",
    "edgeai_telemetry",
  ];
  return ALL_SOURCES.map((source) => ({
    source,
    label: COVERAGE_SOURCE_LABELS[source],
    present: present.includes(source),
  }));
}


// getValuation is wired to the real backend (POST /api/valuation) — no fixtures.
// The valuation engine (backend/app/services/valuation.py) produces both modes:
// listing_evaluation (priced assets) and opportunity_discovery (null-price assets).

// ── TemporalOutput fixtures — coverage always present; activity conditional ────

const TEMPORAL_OUTPUTS: Record<string, TemporalOutput> = {
  // Maadi: EdgeAI sensor on-site → full temporal data
  "asset-maadi-corniche-4a": {
    coverageScore: 91,
    sourceChecklist: buildChecklist([
      "demographics", "gis", "traffic", "competition", "commercial_activity", "edgeai_telemetry",
    ]),
    activityProfile: [
      { hour: 8,  dayOfWeek: 1, relativeActivity: 0.31 },
      { hour: 9,  dayOfWeek: 1, relativeActivity: 0.52 },
      { hour: 11, dayOfWeek: 1, relativeActivity: 0.74 },
      { hour: 13, dayOfWeek: 1, relativeActivity: 0.88 },
      { hour: 15, dayOfWeek: 1, relativeActivity: 0.62 },
      { hour: 18, dayOfWeek: 1, relativeActivity: 0.91 },
      { hour: 20, dayOfWeek: 1, relativeActivity: 0.78 },
      { hour: 13, dayOfWeek: 5, relativeActivity: 0.95 },
      { hour: 18, dayOfWeek: 5, relativeActivity: 0.97 },
      { hour: 12, dayOfWeek: 6, relativeActivity: 0.93 },
    ],
    peakWindows: [
      { label: "Friday evening (18:00–21:00)", dayPattern: "Friday", peakHours: [18, 21], intensity: "high" },
      { label: "Weekday lunch (12:00–14:00)", dayPattern: "Monday–Thursday", peakHours: [12, 14], intensity: "moderate" },
      { label: "Weekday evening (18:00–20:00)", dayPattern: "Monday–Thursday", peakHours: [18, 20], intensity: "moderate" },
    ],
    forecastHorizon: "14 days",
    temporalConfidence: "high",
  },

  // Heliopolis: no sensor → coverage score + checklist only; activity null
  "asset-heliopolis-2b": {
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

  // Imbaba: limited sources; no sensor; sparse temporal
  "asset-imbaba-3f": {
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

  // Fayoum: minimal coverage; only demographics
  "asset-fayoum-new-dist-7c": {
    coverageScore: 28,
    sourceChecklist: buildChecklist([
      "demographics",
    ]),
    activityProfile: null,
    peakWindows: null,
    forecastHorizon: null,
    temporalConfidence: null,
  },

  // Zamalek: moderate; demographics + GIS only
  "asset-zamalek-9d": {
    coverageScore: 61,
    sourceChecklist: buildChecklist([
      "demographics", "gis", "competition",
    ]),
    activityProfile: null,
    peakWindows: null,
    forecastHorizon: null,
    temporalConfidence: null,
  },

  // Nasr City: sensor-equipped; full temporal
  "asset-nasr-city-coffee-11a": {
    coverageScore: 88,
    sourceChecklist: buildChecklist([
      "demographics", "gis", "traffic", "competition", "commercial_activity", "edgeai_telemetry",
    ]),
    activityProfile: [
      { hour: 9,  dayOfWeek: 1, relativeActivity: 0.55 },
      { hour: 11, dayOfWeek: 1, relativeActivity: 0.78 },
      { hour: 13, dayOfWeek: 1, relativeActivity: 0.92 },
      { hour: 16, dayOfWeek: 1, relativeActivity: 0.65 },
      { hour: 19, dayOfWeek: 1, relativeActivity: 0.88 },
      { hour: 13, dayOfWeek: 5, relativeActivity: 0.97 },
      { hour: 19, dayOfWeek: 5, relativeActivity: 0.99 },
    ],
    peakWindows: [
      { label: "Friday evening (19:00–22:00)", dayPattern: "Friday", peakHours: [19, 22], intensity: "high" },
      { label: "Weekday lunch (12:00–14:00)", dayPattern: "Monday–Thursday", peakHours: [12, 14], intensity: "high" },
    ],
    forecastHorizon: "14 days",
    temporalConfidence: "high",
  },
};

// ═════════════════════════════════════════════════════════════════════════════
// NEW PUBLIC API — asset-centric, three-output
// ═════════════════════════════════════════════════════════════════════════════

/**
 * Returns all candidate assets for map rendering and list browse.
 * Includes coverageScore, availabilityStatus, and commercial terms.
 * Backend: GET /api/assets  (FastAPI + PostgreSQL/PostGIS — Session 1 live)
 */
export async function getAssets(): Promise<Asset[]> {
  const res = await fetch(`${API_BASE}/api/assets`);
  if (!res.ok) {
    throw new Error(`GET /api/assets failed: ${res.status} ${res.statusText}`);
  }
  return res.json() as Promise<Asset[]>;
}

/**
 * Returns a single asset by ID.
 * Backend: GET /api/assets/{id}  (FastAPI + PostgreSQL/PostGIS — Session 1 live)
 * Throws on non-200 (including 404 — let TanStack Query handle it).
 */
export async function getAssetById(assetId: string): Promise<Asset> {
  const res = await fetch(`${API_BASE}/api/assets/${encodeURIComponent(assetId)}`);
  if (!res.ok) {
    throw new Error(`GET /api/assets/${assetId} failed: ${res.status} ${res.statusText}`);
  }
  return res.json() as Promise<Asset>;
}

/**
 * Returns all prior concept-assessments for an asset (one-to-many).
 * Enables a user to resume work and see all concepts already run against an asset.
 * In the mock, returns an empty array (no persisted assessments across sessions).
 * Real backend: GET /api/assets/:id/assessments
 */
export async function getAssessmentsForAsset(
  assetId: string
): Promise<ConceptAssessment[]> {
  await delay(300);
  // Mock: no server-side persistence yet; the store holds session assessments.
  // The real backend would return the user's prior assessments for this asset.
  void assetId;
  return [];
}

/**
 * Run a feasibility assessment for a specific asset + business concept.
 * Always returns a FeasibilityOutput (Face 1 of three). The concept is now
 * a first-class field in the signature, not buried in parameters.
 * Real backend: POST /api/assessments/feasibility
 */
export async function getFeasibilityForConcept(
  asset: Asset,
  concept: BusinessConcept,
  params: BusinessParameters
): Promise<FeasibilityOutput> {
  const res = await fetch(`${API_BASE}/api/assessments/feasibility`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ assetId: asset.id, concept, parameters: params }),
  });
  if (!res.ok) {
    throw new Error(
      `POST /api/assessments/feasibility failed: ${res.status} ${res.statusText}`
    );
  }
  return res.json() as Promise<FeasibilityOutput>;
}

/**
 * Return valuation intelligence for an asset (+ optional concept).
 *
 * The optional concept makes the feasibility signal the engine consumes
 * concept-specific (richest valuation); omitting it falls back to a generic
 * proxy at slightly reduced confidence (handled server-side).
 *
 * Returns null when valuation is unavailable for this request — an honest,
 * renderable state, not an error (the engine itself always returns a result for
 * a real asset, so null here means the asset was not found, the session is not
 * authenticated, or the service is unreachable). Returning null rather than
 * throwing keeps an always-present feasibility face from being blocked by a
 * valuation hiccup inside runConceptAssessment.
 *
 * Real backend: POST /api/valuation  (auth: session cookie via credentials:'include')
 */
export async function getValuation(
  assetId: string,
  concept?: BusinessConcept
): Promise<ValuationOutput | null> {
  try {
    const res = await fetch(`${API_BASE}/api/valuation`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify({ assetId, concept: concept ?? null }),
    });
    if (!res.ok) return null; // honest absence — valuation unavailable for this asset
    return (await res.json()) as ValuationOutput;
  } catch {
    return null; // network/transport failure → honest absence; never blocks feasibility
  }
}

/**
 * Return temporal / coverage intelligence for an asset.
 * coverageScore and sourceChecklist are ALWAYS present (we always know what
 * data sources we have). activityProfile and peakWindows are null when no
 * sensor coverage exists — this is an honest renderable state.
 * Real backend: GET /api/assets/:id/temporal
 */
export async function getTemporalIntelligence(
  assetId: string
): Promise<TemporalOutput> {
  await delay(600);
  return (
    TEMPORAL_OUTPUTS[assetId] ?? {
      coverageScore: 0,
      sourceChecklist: buildChecklist([]),
      activityProfile: null,
      peakWindows: null,
      forecastHorizon: null,
      temporalConfidence: null,
    }
  );
}

/**
 * Assemble all three intelligence faces into a ConceptAssessment.
 * This is a convenience function for the UI — fetches all three outputs in
 * parallel and returns the unified ConceptAssessment container.
 * In the real backend this would be a single orchestrated endpoint.
 * Real backend: POST /api/assessments { assetId, concept, parameters }
 */
export async function runConceptAssessment(
  asset: Asset,
  concept: BusinessConcept,
  params: BusinessParameters
): Promise<ConceptAssessment> {
  const [feasibility, valuation, temporal] = await Promise.all([
    getFeasibilityForConcept(asset, concept, params),
    getValuation(asset.id, concept),
    getTemporalIntelligence(asset.id),
  ]);

  return {
    assetId: asset.id,
    concept,
    parameters: params,
    generatedAt: new Date().toISOString(),
    feasibility,
    valuation,
    temporal,
  };
}

// ═════════════════════════════════════════════════════════════════════════════
// PERSISTENCE — save / list / get saved assessments
// Real backend: POST|GET /api/assessments  (Session I — live)
// Auth: cookie-based (credentials: 'include'); no Authorization header needed.
// ═════════════════════════════════════════════════════════════════════════════

/**
 * Save a completed feasibility assessment to the backend.
 * The backend stamps engine_version and records coverage_score from the asset.
 * Returns the StoredAssessmentResponse with the generated assessmentId.
 * Real backend: POST /api/assessments
 */
export async function saveAssessment(
  assetId: string,
  concept: BusinessConcept,
  feasibility: FeasibilityOutput
): Promise<SavedAssessmentRecord> {
  const res = await fetch(`${API_BASE}/api/assessments`, {
    method: "POST",
    credentials: "include",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ assetId, concept, feasibility }),
  });
  if (!res.ok) {
    throw new Error(`POST /api/assessments failed: ${res.status} ${res.statusText}`);
  }
  return res.json() as Promise<SavedAssessmentRecord>;
}

/**
 * List saved assessments, newest first. Optionally filtered by asset.
 * Results are scoped to the authenticated user on the backend.
 * Real backend: GET /api/assessments[?assetId=X]
 */
export async function listSavedAssessments(
  assetId?: string
): Promise<SavedAssessmentRecord[]> {
  const url = assetId
    ? `${API_BASE}/api/assessments?assetId=${encodeURIComponent(assetId)}`
    : `${API_BASE}/api/assessments`;
  const res = await fetch(url, { credentials: "include" });
  if (!res.ok) {
    throw new Error(`GET /api/assessments failed: ${res.status} ${res.statusText}`);
  }
  return res.json() as Promise<SavedAssessmentRecord[]>;
}

/**
 * Retrieve one saved assessment by ID.
 * Throws on 404 — let TanStack Query surface the error.
 * Real backend: GET /api/assessments/{id}
 */
export async function getSavedAssessment(
  assessmentId: string
): Promise<SavedAssessmentRecord> {
  const res = await fetch(
    `${API_BASE}/api/assessments/${encodeURIComponent(assessmentId)}`,
    { credentials: "include" }
  );
  if (!res.ok) {
    throw new Error(
      `GET /api/assessments/${assessmentId} failed: ${res.status} ${res.statusText}`
    );
  }
  return res.json() as Promise<SavedAssessmentRecord>;
}

// ═════════════════════════════════════════════════════════════════════════════
// AUTHENTICATION — register / login / logout / me
// Real backend: POST|GET /api/auth/*  (cookie-based — Auth Session 1+)
//
// Security: JWT lives exclusively in an httpOnly cookie set by the backend.
// No token ever touches localStorage, sessionStorage, or a JS variable.
// All auth fetch calls use credentials: 'include' so the browser sends the
// cookie automatically on every request to the same origin.
// ═════════════════════════════════════════════════════════════════════════════

// ── Auth response types ───────────────────────────────────────────────────────

export interface UserResponse {
  id: string;
  email: string;
  isActive: boolean;
  createdAt: string;
}

export interface LogoutResponse {
  message: string;
}

/**
 * Register a new user account.
 * Returns the created UserResponse (no password hash in the response).
 * Throws with the server's detail message on failure (409 if email taken).
 * Real backend: POST /api/auth/register
 */
export async function register(
  email: string,
  password: string
): Promise<UserResponse> {
  const res = await fetch(`${API_BASE}/api/auth/register`, {
    method: "POST",
    credentials: "include",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ email, password }),
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(
      (body as { detail?: string }).detail ??
        `POST /api/auth/register failed: ${res.status}`
    );
  }
  return res.json() as Promise<UserResponse>;
}

/**
 * Log in with email and password.
 * On success the backend sets an httpOnly cookie — no token in the response.
 * Returns the UserResponse so the auth context can update immediately.
 * Always returns 401 "Invalid credentials." on failure (no user enumeration).
 * Real backend: POST /api/auth/login
 */
export async function login(
  email: string,
  password: string
): Promise<UserResponse> {
  const res = await fetch(`${API_BASE}/api/auth/login`, {
    method: "POST",
    credentials: "include",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ email, password }),
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(
      (body as { detail?: string }).detail ??
        `POST /api/auth/login failed: ${res.status}`
    );
  }
  return res.json() as Promise<UserResponse>;
}

/**
 * Log out. Clears the httpOnly cookie server-side (Max-Age=0).
 * After this call the browser no longer sends the session cookie.
 * Real backend: POST /api/auth/logout
 */
export async function logout(): Promise<LogoutResponse> {
  const res = await fetch(`${API_BASE}/api/auth/logout`, {
    method: "POST",
    credentials: "include",
    headers: { "Content-Type": "application/json" },
  });
  if (!res.ok) {
    return { message: "Logged out. Server cookie-clear may have failed." };
  }
  return res.json() as Promise<LogoutResponse>;
}

/**
 * Validate the session cookie and return the current user.
 * Used by the AuthProvider on page load to restore a session.
 * Throws (401) if the cookie is absent, expired, or invalid — caller
 * should catch and treat as "not authenticated".
 * Real backend: GET /api/auth/me
 */
export async function getMe(): Promise<UserResponse> {
  const res = await fetch(`${API_BASE}/api/auth/me`, {
    credentials: "include",
  });
  if (!res.ok) {
    throw new Error(`GET /api/auth/me failed: ${res.status}`);
  }
  return res.json() as Promise<UserResponse>;
}
