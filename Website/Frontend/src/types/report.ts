export type ScoreBand = "strong" | "moderate" | "weak";
export type FactorDirection = "positive" | "negative" | "neutral";

export interface ContributingFactor {
  label: string;
  value: string;
  direction: FactorDirection;
  weight: number; // 0–1, relative importance
}

export interface FeasibilityReport {
  siteId: string;
  siteName: string;
  siteType: string;
  coordinates: { lat: number; lng: number };
  generatedAt: string; // ISO timestamp

  feasibilityScore: number; // 0–100
  scoreBand: ScoreBand;
  confidenceInterval: { low: number; high: number };

  revenueRange: {
    low: number;
    high: number;
    currency: string;
    period: "monthly";
    categoryBenchmark: number;
    benchmarkLabel: string;
  };

  contributingFactors: ContributingFactor[];
  narrative: string;

  labelQuality: "high" | "medium" | "low";
  dataSources: string[];
  dataPointCount: number;
  coverageNote?: string;
}

export interface InsufficientDataReport {
  siteId: string;
  siteName: string;
  siteType: string;
  coordinates: { lat: number; lng: number };
  dataPointCount: number;
  minimumRequired: number;
  coverageNote: string;
}
