import type { Site } from "./site";
import type { BusinessParameters } from "./parameters";
import type { FeasibilityReport } from "./report";

/**
 * The central object the entire product orbits:
 *   (location) + (niche + parameters) → (feasibility score)
 *
 * Built incrementally as the user advances through the flow.
 * All screens read from this one shape — never a private copy.
 */
export interface Assessment {
  selectedSite: Site | null;
  businessParameters: BusinessParameters | null;
  feasibilityResult: FeasibilityReport | null;
}

export type AssessmentTransition =
  | { type: "SELECT_SITE"; site: Site }
  | { type: "SET_PARAMETERS"; params: BusinessParameters }
  | { type: "SET_RESULT"; result: FeasibilityReport }
  | { type: "RESET" };

export interface ForecastData {
  siteId: string;
  period: "monthly";
  forecastRevenue: { low: number; high: number; currency: string };
  confidenceBand: number;
  generatedAt: string;
}

export interface ComparisonCandidate {
  site: Site;
  report: FeasibilityReport;
}

export type ComparisonAdvantage = "candidate_a" | "candidate_b" | "balanced";

export interface ComparisonFactorReading {
  value: string;
  evidence: string;
}

export interface ComparisonFactor {
  id: string;
  label: string;
  decisionQuestion: string;
  candidateA: ComparisonFactorReading;
  candidateB: ComparisonFactorReading;
  advantage: ComparisonAdvantage;
  conclusion: string;
}

export interface ComparativeDecision {
  winnerSiteId: string;
  headline: string;
  recommendation: string;
  confidence: ConfidenceLevel;
  why: string[];
  tradeoffs: string[];
  nextAction: string;
}

export interface ComparativeIntelligenceData {
  businessConcept: string;
  generatedAt: string;
  candidates: [ComparisonCandidate, ComparisonCandidate];
  decision: ComparativeDecision;
  factors: ComparisonFactor[];
  riskWatchlist: string[];
  trustBasis: Array<{ label: string; value: string; detail: string }>;
}

export type AssessmentStatus =
  | "draft"
  | "awaiting_inputs"
  | "ready_for_analysis"
  | "completed"
  | "needs_review";

export type ConfidenceLevel = "high" | "medium" | "low";

export interface AssessmentPipelineItem {
  id: string;
  site: Site;
  businessType: string;
  status: AssessmentStatus;
  updatedAt: string;
  summary: string;
  nextStep: string;
  report?: FeasibilityReport;
  confidenceLevel?: ConfidenceLevel;
  attentionReason?: string;
}

export interface PipelineStatusSummary {
  status: AssessmentStatus;
  label: string;
  count: number;
  description: string;
}

export interface DashboardMetric {
  label: string;
  value: string;
  detail: string;
  trend?: "up" | "down" | "flat";
}

export interface RecommendedDashboardAction {
  id: string;
  priority: "high" | "medium" | "low";
  title: string;
  detail: string;
  href: string;
  actionLabel: string;
}

export interface PlatformIntelligenceStatus {
  dataCoverage: number;
  telemetryStatus: "simulated" | "available" | "limited";
  sourceLayers: string[];
  confidenceIndicators: Array<{ label: string; value: string; status: string }>;
}

export interface ExecutiveDashboardData {
  summary: {
    activeAssessments: number;
    completedAssessments: number;
    highConfidenceOpportunities: number;
    needsReview: number;
    intelligenceSummary: string;
  };
  opportunitySpotlight: AssessmentPipelineItem[];
  pipeline: PipelineStatusSummary[];
  intelligenceOverview: DashboardMetric[];
  recentAssessments: AssessmentPipelineItem[];
  recommendedActions: RecommendedDashboardAction[];
  platformStatus: PlatformIntelligenceStatus;
}
