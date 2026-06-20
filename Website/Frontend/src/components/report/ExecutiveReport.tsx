"use client";

// ExecutiveReport — renders a single ConceptAssessment's three intelligence faces.
//
// Props changed in Session F:
//   before: report: FeasibilityReport (old single-output model)
//   after:  assessment: ConceptAssessment, asset: Asset (new three-output model)
//
// The three faces:
//   1. Feasibility — score + CI + success likelihood + contributing factors + narrative (dominant)
//   2. Valuation   — fair value + expected value, or honest "insufficient data" when null
//   3. Coverage    — coverage score + 6-source checklist + temporal note
//
// ValuationFace and CoverageFace are imported from @/components/shared/AssessmentFaces
// (reconciliation: same rendering context as workspace VerdictPanel; one component).

import { Button } from "@/components/primitives/Button";
import { ValuationFace, CoverageFace } from "@/components/shared/AssessmentFaces";
import {
  bandColors,
  bandLabel,
  cn,
  formatDate,
  formatNumber,
} from "@/lib/utils";
import type { Asset } from "@/types/asset";
import type {
  ConceptAssessment,
  FeasibilityOutput,
} from "@/types/assessment-v2";
import type { ContributingFactor } from "@/types/report";

// ── Concept label ─────────────────────────────────────────────────────────────

const CONCEPT_LABELS: Record<string, string> = {
  specialty_coffee: "Specialty Coffee",
  casual_dining: "Casual Dining",
  fast_casual_restaurant: "Fast Casual",
  pharmacy: "Pharmacy",
  premium_retail: "Premium Retail",
  fitness_studio: "Fitness Studio",
  beauty_wellness: "Beauty & Wellness",
  fast_food: "Quick Service",
  retail_fashion: "Fashion Retail",
  retail_grocery: "Grocery Retail",
  gym_fitness: "Gym & Fitness",
  coworking: "Coworking",
  other: "Other",
};

function conceptLabel(concept: string): string {
  return CONCEPT_LABELS[concept] ?? concept.replace(/_/g, " ");
}

// ── Scenario mode ─────────────────────────────────────────────────────────────

export type ScenarioMode = "strong" | "moderate" | "weak";

const scenarioLabels: Record<ScenarioMode, string> = {
  strong: "Strong",
  moderate: "Moderate",
  weak: "Weak",
};

export interface ExecutiveReportProps {
  assessment: ConceptAssessment;
  asset: Asset;
  mode: ScenarioMode;
  onModeChange: (mode: ScenarioMode) => void;
}

// ── Inline trust derivation ───────────────────────────────────────────────────
// Derived directly from FeasibilityOutput; does not depend on trust.ts.

function derivedConfidence(f: FeasibilityOutput): "High" | "Medium" | "Low" {
  const w = f.confidenceInterval.high - f.confidenceInterval.low;
  if (w < 15 && f.dataSources.length >= 4 && f.labelQuality === "high")
    return "High";
  if (w < 25 && f.labelQuality !== "low") return "Medium";
  return "Low";
}

function derivedRisk(f: FeasibilityOutput): string {
  if (f.scoreBand === "strong" && derivedConfidence(f) !== "Low") return "Manageable";
  if (f.scoreBand === "moderate") return "Elevated";
  return "High";
}

// ── Entry point ───────────────────────────────────────────────────────────────

export function ExecutiveReport({
  assessment,
  asset,
  mode,
  onModeChange,
}: ExecutiveReportProps) {
  const { feasibility } = assessment;
  const colors = bandColors(feasibility.scoreBand);
  const confidence = derivedConfidence(feasibility);
  const risk = derivedRisk(feasibility);

  const positiveDrivers = topFactors(feasibility.contributingFactors, "positive", 4);
  const riskFactors = topNegativeFactors(feasibility);
  const neutralFactors = topFactors(feasibility.contributingFactors, "neutral", 3);

  return (
    <main className="min-h-screen bg-page text-ink-body">
      <ReportTopBar
        mode={mode}
        onModeChange={onModeChange}
        siteName={asset.address}
        concept={conceptLabel(assessment.concept)}
      />

      <div className="mx-auto w-full max-w-7xl px-5 py-6 lg:px-8 lg:py-8">
        <ExecutiveSummary
          assessment={assessment}
          asset={asset}
          colors={colors}
          confidence={confidence}
          risk={risk}
        />

        <BusinessImpactSection
          feasibility={feasibility}
          confidence={confidence}
          risk={risk}
        />

        <div className="mt-6 grid gap-6 xl:grid-cols-[minmax(0,1.25fr)_380px]">
          {/* Main column — feasibility evidence */}
          <div className="space-y-6">
            <InsightSection
              emptyText="No dominant positive driver was isolated in this assessment."
              factors={positiveDrivers}
              label="Why this recommendation"
              tone="positive"
            />
            <InsightSection
              emptyText="No material risk driver was isolated in this assessment."
              factors={riskFactors}
              label="Decision constraints"
              tone="risk"
            />
            <LocationIntelligence feasibility={feasibility} />
          </div>

          {/* Sidebar — valuation, coverage, decision */}
          <div className="space-y-6">
            <DecisionSection
              feasibility={feasibility}
              confidence={confidence}
              risk={risk}
              neutralFactors={neutralFactors}
            />

            <ReportSection title="Valuation Intelligence">
              <ValuationFace assessment={assessment} />
            </ReportSection>

            <ReportSection title="Timing & Coverage">
              <CoverageFace temporal={assessment.temporal} />
            </ReportSection>

          </div>
        </div>

        {/* Methodology footnote */}
        <MethodologyFootnote feasibility={feasibility} />
      </div>
    </main>
  );
}

// ── Top bar ───────────────────────────────────────────────────────────────────

function ReportTopBar({
  mode,
  onModeChange,
  siteName,
  concept,
}: {
  mode: ScenarioMode;
  onModeChange: (mode: ScenarioMode) => void;
  siteName: string;
  concept: string;
}) {
  return (
    <header className="sticky top-0 z-20 border-b border-hairline bg-page/95 backdrop-blur">
      <div className="mx-auto flex h-14 max-w-7xl items-center justify-between gap-4 px-5 lg:px-8">
        <div className="min-w-0">
          <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint">
            Executive Memo · {concept}
          </p>
          <p className="truncate font-display text-sm text-ink-strong">{siteName}</p>
        </div>

        <div className="flex items-center gap-2">
          {/* Export stub — generation requires backend; not built yet */}
          <button
            type="button"
            disabled
            title="Export requires backend integration — available post-launch"
            className="hidden h-8 items-center gap-1.5 rounded-(--r-md) border border-hairline bg-transparent px-3 font-body text-xs text-ink-faint opacity-50 sm:inline-flex"
          >
            Export
          </button>

          <div className="hidden items-center gap-1 sm:flex">
            {(["strong", "moderate", "weak"] as ScenarioMode[]).map((item) => (
              <Button
                key={item}
                aria-pressed={mode === item}
                size="sm"
                type="button"
                variant={mode === item ? "primary" : "ghost"}
                onClick={() => onModeChange(item)}
              >
                {scenarioLabels[item]}
              </Button>
            ))}
          </div>
        </div>
      </div>
    </header>
  );
}

// ── Executive summary (hero) ───────────────────────────────────────────────────

function ExecutiveSummary({
  assessment,
  asset,
  colors,
  confidence,
  risk,
}: {
  assessment: ConceptAssessment;
  asset: Asset;
  colors: ReturnType<typeof bandColors>;
  confidence: string;
  risk: string;
}) {
  const f = assessment.feasibility;

  return (
    <section className="rounded-(--r-lg) border border-hairline bg-surface shadow-(--shadow-card)">
      <div className="grid gap-0 xl:grid-cols-[minmax(0,1fr)_380px]">
        {/* Left — recommendation + metrics */}
        <div className="border-b border-hairline p-6 lg:p-8 xl:border-b-0 xl:border-r">
          <div className="flex flex-wrap items-center gap-2">
            <StatusPill label="Completed assessment" />
            <StatusPill label={conceptLabel(assessment.concept)} />
            <StatusPill label={formatDate(f.generatedAt)} />
          </div>

          <div className="mt-7 max-w-3xl">
            <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
              Recommendation
            </p>
            <h1 className="mt-3 font-display text-4xl text-ink-strong lg:text-5xl">
              {recommendationLabel(f.scoreBand)}
            </h1>
            <p className="mt-4 max-w-2xl font-body text-base leading-7 text-ink-body">
              {executiveSummaryCopy(f.scoreBand)}
            </p>
          </div>

          {/* Four terse metrics */}
          <div className="mt-8 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
            <SummaryMetric
              label="Recommendation Score"
              value={`${f.feasibilityScore}`}
              sub={bandLabel(f.scoreBand)}
            />
            <SummaryMetric
              label="Confidence"
              value={`${f.confidenceInterval.low}–${f.confidenceInterval.high}`}
              sub="95% CI"
            />
            <SummaryMetric
              label="Success Likelihood"
              value={`${Math.round(f.successLikelihood * 100)}%`}
              sub="Business fit signal"
            />
            <SummaryMetric
              label="Evidence Strength"
              value={confidence}
              sub={`${f.dataSources.length} source layers`}
            />
          </div>

          <div className="mt-4 flex flex-wrap gap-2">
            <span className="rounded-(--r-sm) border border-hairline bg-sunken px-2.5 py-1 font-body text-xs text-ink-muted">
              {formatNumber(f.dataPointCount)} evidence records
            </span>
            <span className="rounded-(--r-sm) border border-hairline bg-sunken px-2.5 py-1 font-body text-xs text-ink-muted">
              Risk: {risk}
            </span>
          </div>
        </div>

        {/* Right — decision readiness + asset metadata */}
        <div className={cn("p-6 lg:p-8", colors.bg)}>
          <div className={cn("rounded-(--r-lg) border bg-page/35 p-5", colors.border)}>
            <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
              Next Action
            </p>
            <p className={cn("mt-4 font-display text-3xl", colors.text)}>
              {decisionReadiness(f.scoreBand)}
            </p>
            <p className="mt-3 font-body text-sm leading-6 text-ink-body">
              {decisionReadinessCopy(f.scoreBand)}
            </p>
          </div>

          <dl className="mt-5 divide-y divide-hairline rounded-(--r-lg) border border-hairline bg-sunken">
            <SummaryRow label="Location" value={asset.address} />
            <SummaryRow label="District" value={asset.area} />
            <SummaryRow label="Concept" value={conceptLabel(assessment.concept)} />
            <SummaryRow
              label="Coordinates"
              value={`${asset.coordinates.lat.toFixed(4)}, ${asset.coordinates.lng.toFixed(4)}`}
              mono
            />
            <SummaryRow
              label="Assessment ID"
              value={assessmentId(assessment)}
              mono
            />
          </dl>
        </div>
      </div>
    </section>
  );
}

// ── Location intelligence ─────────────────────────────────────────────────────

function BusinessImpactSection({
  feasibility,
  confidence,
  risk,
}: {
  feasibility: FeasibilityOutput;
  confidence: string;
  risk: string;
}) {
  return (
    <section className="mt-6 rounded-(--r-lg) border border-hairline bg-surface p-5 shadow-(--shadow-card)">
      <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
        Business Impact
      </p>
      <div className="mt-4 grid gap-4 md:grid-cols-[minmax(0,1fr)_280px]">
        <div>
          <h2 className="font-display text-2xl text-ink-strong">
            {expectedUpside(feasibility.scoreBand)} upside with {risk.toLowerCase()} risk
          </h2>
          <p className="mt-3 font-body text-sm leading-6 text-ink-body">
            {recommendedActionCopy(feasibility.scoreBand)}
          </p>
        </div>
        <div className="grid gap-3">
          <DecisionAttribute label="Confidence" value={confidence} />
          <DecisionAttribute
            label="Decision Readiness"
            value={decisionReadiness(feasibility.scoreBand)}
          />
        </div>
      </div>
    </section>
  );
}

function LocationIntelligence({ feasibility }: { feasibility: FeasibilityOutput }) {
  const metrics = locationMetrics(feasibility);

  return (
    <ReportSection title="Feasibility Evidence">
      <div className="grid gap-3 md:grid-cols-2 xl:grid-cols-3">
        {metrics.map((metric) => (
          <div
            key={metric.label}
            className="rounded-(--r-md) border border-hairline bg-sunken p-4"
          >
            <p className="font-body text-xs uppercase tracking-widest text-ink-faint">
              {metric.label}
            </p>
            <p className="mt-2 font-mono text-xl text-ink-strong">{metric.value}</p>
            <p className="mt-2 font-body text-sm leading-5 text-ink-muted">
              {metric.interpretation}
            </p>
          </div>
        ))}
      </div>
    </ReportSection>
  );
}

// ── Insight section ───────────────────────────────────────────────────────────

function InsightSection({
  label,
  factors,
  tone,
  emptyText,
}: {
  label: string;
  factors: ContributingFactor[];
  tone: "positive" | "risk";
  emptyText: string;
}) {
  return (
    <ReportSection title={label}>
      {factors.length === 0 ? (
        <p className="rounded-(--r-md) border border-hairline bg-sunken p-4 font-body text-sm text-ink-muted">
          {emptyText}
        </p>
      ) : (
        <div className="space-y-3">
          {factors.map((factor) => (
            <InsightCard key={factor.label} factor={factor} tone={tone} />
          ))}
        </div>
      )}
    </ReportSection>
  );
}

function InsightCard({
  factor,
  tone,
}: {
  factor: ContributingFactor;
  tone: "positive" | "risk";
}) {
  return (
    <article className="rounded-(--r-md) border border-hairline bg-sunken p-4">
      <div className="flex items-start justify-between gap-4">
        <div>
          <p className="font-body text-base font-medium text-ink-strong">
            {factor.label}
          </p>
          <p className="mt-2 font-body text-sm leading-6 text-ink-muted">
            {insightCopy(factor, tone)}
          </p>
        </div>
        <div className="shrink-0 text-right">
          <p
            className={cn(
              "font-mono text-xl",
              tone === "positive" ? "text-verdict-strong" : "text-verdict-warn"
            )}
          >
            {Math.round(factor.weight * 100)}%
          </p>
          <p className="font-body text-xs text-ink-faint">weight</p>
        </div>
      </div>
      <p className="mt-3 font-mono text-sm text-ink-body">{factor.value}</p>
    </article>
  );
}

// ── Decision section (sidebar) ─────────────────────────────────────────────────

function DecisionSection({
  feasibility,
  confidence,
  risk,
  neutralFactors,
}: {
  feasibility: FeasibilityOutput;
  confidence: string;
  risk: string;
  neutralFactors: ContributingFactor[];
}) {
  return (
    <ReportSection title="Recommended Action">
      <div className="rounded-(--r-lg) border border-hairline bg-sunken p-4">
        <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
          Recommended Action
        </p>
        <p className="mt-3 font-display text-2xl text-ink-strong">
          {recommendedAction(feasibility.scoreBand)}
        </p>
        <p className="mt-3 font-body text-sm leading-6 text-ink-body">
          {recommendedActionCopy(feasibility.scoreBand)}
        </p>
      </div>

      <div className="mt-4 grid gap-3">
        <DecisionAttribute label="Expected Upside" value={expectedUpside(feasibility.scoreBand)} />
        <DecisionAttribute label="Confidence" value={confidence} />
        <DecisionAttribute label="Risk Level" value={risk} />
        <DecisionAttribute label="Decision Readiness" value={decisionReadiness(feasibility.scoreBand)} />
      </div>

      {neutralFactors.length > 0 && (
        <div className="mt-4 rounded-(--r-md) border border-hairline bg-surface p-4">
          <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
            Conditions to Validate
          </p>
          <ul className="mt-3 space-y-2">
            {neutralFactors.map((factor) => (
              <li key={factor.label} className="font-body text-sm text-ink-body">
                {factor.label}:{" "}
                <span className="font-mono text-ink-muted">{factor.value}</span>
              </li>
            ))}
          </ul>
        </div>
      )}
    </ReportSection>
  );
}

// ── Methodology footnote ──────────────────────────────────────────────────────

function MethodologyFootnote({ feasibility }: { feasibility: FeasibilityOutput }) {
  return (
    <div className="mt-8 rounded-(--r-md) border border-hairline bg-surface p-5">
      <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
        Methodology
      </p>
      <p className="mt-3 font-body text-sm leading-6 text-ink-body">
        The score is generated by the platform intelligence layer from structured features and
        business assumptions. Confidence reflects interval width, data coverage, and source
        quality. The reporting layer explains the model output; it does not create the score.
        Never present a score as a profit forecast — always pair with its confidence interval and
        the contributing evidence.
      </p>
      <div className="mt-3 flex flex-wrap gap-x-6 gap-y-1">
        <p className="font-mono text-xs text-ink-faint">
          {formatNumber(feasibility.dataPointCount)} evidence records
        </p>
        <p className="font-mono text-xs text-ink-faint">
          Label quality: {feasibility.labelQuality}
        </p>
        <p className="font-mono text-xs text-ink-faint">
          Generated {formatDate(feasibility.generatedAt)}
        </p>
      </div>
      <ul className="mt-3 space-y-1">
        {feasibility.dataSources.map((source) => (
          <li
            key={source}
            className="flex items-start gap-2 font-body text-xs text-ink-faint"
          >
            <span
              className="mt-1.5 h-1 w-1 shrink-0 rounded-full bg-ink-faint/50"
              aria-hidden="true"
            />
            {source}
          </li>
        ))}
      </ul>
    </div>
  );
}

// ── Shared section wrapper ────────────────────────────────────────────────────

function ReportSection({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <section className="rounded-(--r-lg) border border-hairline bg-surface shadow-(--shadow-card)">
      <div className="border-b border-hairline bg-sunken px-5 py-3">
        <h2 className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
          {title}
        </h2>
      </div>
      <div className="p-5">{children}</div>
    </section>
  );
}

// ── Small primitives ──────────────────────────────────────────────────────────

function SummaryMetric({
  label,
  value,
  sub,
}: {
  label: string;
  value: string;
  sub: string;
}) {
  return (
    <div className="rounded-(--r-md) border border-hairline bg-sunken p-4">
      <p className="font-body text-xs uppercase tracking-widest text-ink-faint">{label}</p>
      <p className="mt-2 font-mono text-2xl font-semibold leading-none text-ink-strong">
        {value}
      </p>
      <p className="mt-2 font-body text-xs text-ink-muted">{sub}</p>
    </div>
  );
}

function SummaryRow({
  label,
  value,
  mono = false,
}: {
  label: string;
  value: string;
  mono?: boolean;
}) {
  return (
    <div className="grid grid-cols-[112px_1fr] gap-3 px-4 py-3">
      <dt className="font-body text-xs uppercase tracking-widest text-ink-faint">{label}</dt>
      <dd
        className={cn("min-w-0 truncate text-sm text-ink-body", mono ? "font-mono" : "font-body")}
        title={value}
      >
        {value}
      </dd>
    </div>
  );
}

function StatusPill({ label }: { label: string }) {
  return (
    <span className="rounded-(--r-sm) border border-hairline bg-sunken px-2.5 py-1 font-body text-xs text-ink-muted">
      {label}
    </span>
  );
}

function DecisionAttribute({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between gap-3 rounded-(--r-md) border border-hairline bg-sunken px-4 py-3">
      <span className="font-body text-xs uppercase tracking-widest text-ink-faint">
        {label}
      </span>
      <span className="font-mono text-sm text-ink-strong">{value}</span>
    </div>
  );
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function topFactors(
  factors: ContributingFactor[],
  direction: ContributingFactor["direction"],
  count: number
): ContributingFactor[] {
  return factors
    .filter((f) => f.direction === direction)
    .sort((a, b) => b.weight - a.weight)
    .slice(0, count);
}

function topNegativeFactors(f: FeasibilityOutput): ContributingFactor[] {
  const negative = topFactors(f.contributingFactors, "negative", 4);
  if (negative.length > 0) return negative;
  return [...f.contributingFactors]
    .sort((a, b) => a.weight - b.weight)
    .slice(0, 2)
    .map((factor) => ({ ...factor, direction: "negative" as const }));
}

function locationMetrics(f: FeasibilityOutput) {
  const byLabel = (fragment: string) =>
    f.contributingFactors.find((c) =>
      c.label.toLowerCase().includes(fragment)
    );

  const footfall = byLabel("footfall");
  const competition = byLabel("competitor");
  const demographic = byLabel("demographic");
  const rent = byLabel("rent");
  const transit = byLabel("transit");
  const parking = byLabel("parking");

  return [
    {
      label: "Footfall Indicators",
      value: footfall?.value ?? "Limited signal",
      interpretation: metricCopy(footfall, "Local movement intensity"),
    },
    {
      label: "Competition Density",
      value: competition?.value ?? "Unknown",
      interpretation: metricCopy(competition, "Nearby commercial pressure"),
    },
    {
      label: "Demographic Relevance",
      value: demographic?.value ?? "Unknown",
      interpretation: metricCopy(demographic, "Catchment fit"),
    },
    {
      label: "Rent Pressure",
      value: rent?.value ?? "Unknown",
      interpretation: metricCopy(rent, "Cost constraint"),
    },
    {
      label: "Accessibility",
      value: transit?.value ?? parking?.value ?? "Mixed",
      interpretation: metricCopy(transit ?? parking, "Access and convenience"),
    },
    {
      label: "Composite Signal",
      value: `${f.feasibilityScore}/100`,
      interpretation: "Aggregate signal across available local evidence.",
    },
  ];
}

function metricCopy(factor: ContributingFactor | undefined, fallback: string) {
  if (!factor) return `${fallback}; evidence layer not isolated in this report.`;
  if (factor.direction === "positive") return `${fallback}; favorable signal.`;
  if (factor.direction === "negative") return `${fallback}; constraint to validate.`;
  return `${fallback}; neutral but decision-relevant.`;
}

function assessmentId(assessment: ConceptAssessment) {
  return `ASM-${assessment.assetId.replace(/^asset-/, "").toUpperCase()}-${assessment.concept.substring(0, 3).toUpperCase()}`;
}

function recommendationLabel(band: string) {
  if (band === "strong") return "Strong Candidate";
  if (band === "moderate") return "Moderate Potential";
  return "High Risk";
}

function executiveSummaryCopy(band: string) {
  if (band === "strong") {
    return "The evidence supports advancing this location to field validation. The opportunity is not risk-free, but the current location intelligence shows a favorable commercial fit.";
  }
  if (band === "moderate") {
    return "The opportunity is viable only under disciplined assumptions. The site should remain in consideration, but it needs comparison against stronger alternatives before commitment.";
  }
  return "The current intelligence does not support prioritizing this location for the selected business model. The risk profile is too high relative to expected upside.";
}

function decisionReadiness(band: string) {
  if (band === "strong") return "Ready for Validation";
  if (band === "moderate") return "Needs Comparison";
  return "Not Ready";
}

function decisionReadinessCopy(band: string) {
  if (band === "strong") {
    return "Proceed only after on-site validation confirms the highest-weight drivers and visible constraints.";
  }
  if (band === "moderate") {
    return "Use this site as a benchmark candidate, then compare it against two or more alternatives.";
  }
  return "Do not advance this site without a material change in assumptions or evidence coverage.";
}

function recommendedAction(band: string) {
  if (band === "strong") return "Proceed to field validation.";
  if (band === "moderate") return "Run comparative analysis.";
  return "Evaluate alternative locations.";
}

function recommendedActionCopy(band: string) {
  if (band === "strong") {
    return "Validate peak-hour foot traffic, lease assumptions, and the immediate competitive set before moving to negotiation.";
  }
  if (band === "moderate") {
    return "Do not decide from this site alone. Compare against stronger nearby candidates and stress-test rent and differentiation assumptions.";
  }
  return "Redirect assessment effort to locations with stronger local demand, better data coverage, or a better fit for the selected format.";
}

function expectedUpside(band: string) {
  if (band === "strong") return "Material";
  if (band === "moderate") return "Conditional";
  return "Limited";
}

function insightCopy(factor: ContributingFactor, tone: "positive" | "risk") {
  if (tone === "positive") {
    return `${factor.label} is one of the strongest favorable signals in the current assessment and materially supports the recommendation.`;
  }
  return `${factor.label} is a decision constraint. Validate this item before treating the score as investment-ready.`;
}
