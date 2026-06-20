"use client";

import { useState, type FormEvent } from "react";
import { cn, bandColors, bandLabel, formatNumber, formatCurrency } from "@/lib/utils";
import { ValuationFace, CoverageFace } from "@/components/shared/AssessmentFaces";
import { Button } from "@/components/primitives/Button";
import type { Asset } from "@/types/asset";
import type {
  BusinessConcept,
  ConceptAssessment,
  ConceptAssessmentRecord,
  TemporalOutput,
} from "@/types/assessment-v2";
import type { BusinessDomain, BusinessParameters } from "@/types/parameters";

// ── Concept metadata ──────────────────────────────────────────────────────────

const CONCEPT_LABELS: Record<BusinessDomain, string> = {
  specialty_coffee: "Specialty Coffee",
  casual_dining:    "Casual Dining",
  fast_food:        "Quick Service",
  retail_fashion:   "Fashion Retail",
  retail_grocery:   "Grocery Retail",
  pharmacy:         "Pharmacy",
  gym_fitness:      "Gym & Fitness",
  coworking:        "Coworking",
  other:            "Other",
};

const DEFAULT_PARAMS: Omit<BusinessParameters, "domain"> = {
  operatingHours: { open: "09:00", close: "22:00", daysPerWeek: 6 },
  staffCount: 5,
  budget: 500000,
  currency: "EGP",
};

function evidenceStrength(score: number): "High" | "Medium" | "Low" {
  if (score >= 70) return "High";
  if (score >= 45) return "Medium";
  return "Low";
}

function assetRecommendation(score: number, hasTerms: boolean): string {
  if (score >= 70) return "Assess this asset next";
  if (score >= 45) return hasTerms ? "Assess, then compare" : "Assess with caution";
  return "Use as a backup option";
}

function assetReason(score: number, hasTerms: boolean): string {
  if (score >= 70 && hasTerms) {
    return "The evidence base is strong and commercial terms are visible enough to support a serious first assessment.";
  }
  if (score >= 70) {
    return "The evidence base is strong, but missing listed terms should be validated before negotiation.";
  }
  if (score >= 45) {
    return "There is enough evidence to learn from this location, but the result should be checked against alternatives.";
  }
  return "The platform has limited evidence here, so any result will carry wider uncertainty.";
}

function resultRecommendation(band: string): string {
  if (band === "strong") return "Advance to field validation";
  if (band === "moderate") return "Compare before committing";
  return "Prioritize another location";
}

function resultImpact(band: string): string {
  if (band === "strong") return "Material opportunity";
  if (band === "moderate") return "Conditional upside";
  return "Limited upside";
}

// ── Panel props ───────────────────────────────────────────────────────────────

export interface WorkspaceSidePanelProps {
  selectedAsset: Asset | null;
  currentAssessments: ConceptAssessmentRecord;
  activeConceptId: BusinessConcept | null;
  activeAssessment: ConceptAssessment | null;
  /** Which concept is currently being run (available while mutation is pending). */
  analyzingConcept: BusinessConcept | null;
  isAnalyzing: boolean;
  analysisError: string | null;
  /**
   * Temporal intelligence for the selected asset — fetched on asset selection,
   * before any concept is chosen. Provides the source checklist and coverage
   * score for the asset-level intelligence view. null while loading.
   */
  assetTemporal: TemporalOutput | null;
  /** Per-concept saved state: concept → persistenceId. undefined = unsaved. */
  savedConceptIds: Partial<Record<BusinessConcept, string>>;
  /** Called when the user clicks "Save to Portfolio" for the active concept. */
  onSaveAssessment: (concept: BusinessConcept) => void;
  /** True while the save mutation is in flight. */
  isSaving: boolean;
  /** Non-null when the last save attempt failed. */
  saveError: string | null;
  onSubmitConcept: (concept: BusinessConcept, params: BusinessParameters) => void;
  onSwitchConcept: (concept: BusinessConcept) => void;
  onReset: () => void;
}

// ── Concept history list ──────────────────────────────────────────────────────
// Vertical list of concept-assessment records.
// Each row is a data record — score, label, CI range, band.
// Clicking a row is a pure pointer change: no refetch, no data loss.
// Designed to feel like a Bloomberg-style data panel, not browser tabs.

function ConceptHistoryList({
  assessments,
  activeConceptId,
  savedConceptIds,
  onSwitch,
  onAddNew,
}: {
  assessments: ConceptAssessmentRecord;
  activeConceptId: BusinessConcept | null;
  savedConceptIds: Partial<Record<BusinessConcept, string>>;
  onSwitch: (concept: BusinessConcept) => void;
  onAddNew: () => void;
}) {
  const entries = Object.entries(assessments) as [BusinessConcept, ConceptAssessment][];

  return (
    <div className="shrink-0 border-b border-hairline">
      {/* Section header */}
      <div className="flex items-center justify-between px-5 pt-3.5 pb-2">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint">
          Assessed concepts
        </p>
        <button
          type="button"
          onClick={onAddNew}
          className="flex items-center gap-1 rounded-sm px-1 py-0.5 font-body text-[10px] text-accent-muted transition-colors hover:text-accent focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent"
        >
          <svg
            width="8" height="8" viewBox="0 0 8 8"
            fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"
            aria-hidden="true"
          >
            <line x1="4" y1="0.5" x2="4" y2="7.5" />
            <line x1="0.5" y1="4" x2="7.5" y2="4" />
          </svg>
          Run another
        </button>
      </div>

      {/* Concept rows — scrollable if list grows */}
      <div className="max-h-52 overflow-y-auto">
        {entries.map(([concept, assessment]) => {
          const isActive = concept === activeConceptId;
          const colors = bandColors(assessment.feasibility.scoreBand);
          const band = assessment.feasibility.scoreBand;
          const ci = assessment.feasibility.confidenceInterval;

          return (
            <button
              key={concept}
              type="button"
              onClick={() => onSwitch(concept)}
              aria-pressed={isActive}
              className={cn(
                "relative w-full text-left transition-colors",
                "border-t border-hairline first:border-t-0",
                "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-inset focus-visible:ring-accent",
                isActive ? "bg-surface" : "hover:bg-raised/40"
              )}
            >
              {/* Left-edge accent — band color, active row only */}
              {isActive && (
                <span
                  className={cn(
                    "absolute left-0 top-0 bottom-0 w-0.75 rounded-r-xs",
                    band === "strong"   ? "bg-verdict-strong" :
                    band === "moderate" ? "bg-verdict-warn"   :
                                          "bg-verdict-weak"
                  )}
                  aria-hidden="true"
                />
              )}

              <div className="flex items-center gap-3 pl-5 pr-4 py-3">
                {/* Score — left-anchored, tabular mono */}
                <div className="w-9 shrink-0 text-right">
                  <span className={cn(
                    "font-mono font-semibold leading-none tabular-nums",
                    isActive
                      ? cn("text-xl", colors.text)
                      : "text-sm text-ink-faint"
                  )}>
                    {assessment.feasibility.feasibilityScore}
                  </span>
                </div>

                {/* Concept label + CI range */}
                <div className="flex-1 min-w-0">
                  <p className={cn(
                    "font-body text-sm leading-none truncate",
                    isActive ? "font-medium text-ink-strong" : "font-normal text-ink-muted"
                  )}>
                    {CONCEPT_LABELS[concept] ?? concept}
                  </p>
                  <p className="mt-1 font-mono text-[10px] leading-none text-ink-faint">
                    CI&nbsp;{ci.low}–{ci.high}
                    <span className="mx-1.5 opacity-40">·</span>
                    <span className={isActive ? colors.text : ""}>
                      {band === "strong" ? "Strong" : band === "moderate" ? "Moderate" : "Weak"}
                    </span>
                  </p>
                </div>

                {/* Saved indicator — bookmark icon when saved, absent when unsaved */}
                {savedConceptIds[concept] && (
                  <svg
                    width="9" height="11" viewBox="0 0 9 11"
                    fill="currentColor"
                    className="shrink-0 text-accent-muted/70"
                    aria-label="Saved to portfolio"
                  >
                    <path d="M1 1h7a.5.5 0 01.5.5v9l-4-2-4 2V1.5A.5.5 0 011 1z" />
                  </svg>
                )}

                {/* Active indicator — chevron */}
                {isActive && (
                  <svg
                    width="10" height="10" viewBox="0 0 10 10"
                    fill="none" stroke="currentColor" strokeWidth="1.5"
                    strokeLinecap="round" strokeLinejoin="round"
                    className="shrink-0 text-ink-faint" aria-hidden="true"
                  >
                    <polyline points="3 1.5 7 5 3 8.5" />
                  </svg>
                )}
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
}

// ValuationFace and CoverageFace are imported from @/components/shared/AssessmentFaces.
// Extracted in Session F (reconciliation). The wrapper divs and section headers
// are kept here at the call site in VerdictPanel, since they are panel-specific
// (px-5 padding, border-b dividers).


// ── Asset intelligence panel ──────────────────────────────────────────────────
// The deliberate pause between asset selection and concept choice.
// Shows what the platform knows about this location BEFORE the user commits
// to a concept. This is the trust instrument: coverage is the primary subject.
//
// Listed commercial terms come from the Asset object (available immediately).
// Coverage score and source checklist come from TemporalOutput (async fetch).
// While TemporalOutput loads, coverage score from asset.coverageScore is shown
// and the source checklist skeleton is displayed.

function AssetIntelligencePanel({
  asset,
  temporal,
  onProceed,
}: {
  asset: Asset;
  temporal: TemporalOutput | null;
  onProceed: () => void;
}) {
  const coverageScore = temporal?.coverageScore ?? asset.coverageScore;
  const scoreTone =
    coverageScore >= 70 ? "text-status-high" :
    coverageScore >= 45 ? "text-status-med"  :
                           "text-status-low";
  const presentCount = temporal?.sourceChecklist.filter((s) => s.present).length;
  const hasTerms = !!(asset.monthlyRent || asset.salePrice);
  const strength = evidenceStrength(coverageScore);

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 overflow-y-auto">
        <div className="px-5 pt-5 pb-5 border-b border-hairline">
          <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
            Recommended next step
          </p>
          <div className="rounded-(--r-md) border border-accent/35 bg-accent-soft px-4 py-4">
            <p className="font-display text-xl leading-tight text-ink-strong">
              {assetRecommendation(coverageScore, hasTerms)}
            </p>
            <div className="mt-3 grid grid-cols-2 gap-3">
              <div>
                <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                  Confidence
                </p>
                <p className={cn("mt-1 font-mono text-sm font-semibold", scoreTone)}>
                  {strength}
                </p>
              </div>
              <div>
                <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                  Business impact
                </p>
                <p className="mt-1 font-body text-sm font-medium text-ink-body">
                  {hasTerms ? "Price can be tested" : "Terms need validation"}
                </p>
              </div>
            </div>
            <p className="mt-3 font-body text-xs leading-5 text-ink-muted">
              {assetReason(coverageScore, hasTerms)}
            </p>
          </div>
        </div>

        {/* Listed commercial terms */}
        <div className="px-5 pt-4 pb-5 border-b border-hairline">
          <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
            Business impact
          </p>

          {!hasTerms ? (
            <p className="font-body text-xs text-ink-faint">
              Commercial terms not listed for this asset.
            </p>
          ) : (
            <div className="rounded-(--r-md) border border-hairline bg-sunken px-4 py-3 space-y-2.5">
              {asset.monthlyRent && (
                <div className="flex items-center justify-between gap-4">
                  <span className="font-body text-xs text-ink-muted">Monthly rent</span>
                  <span className="font-mono text-sm font-semibold text-ink-strong tabular-nums">
                    {formatCurrency(asset.monthlyRent, asset.currency)}
                    <span className="ml-0.5 font-normal text-ink-faint text-xs">/mo</span>
                  </span>
                </div>
              )}
              {asset.salePrice && (
                <div className="flex items-center justify-between gap-4">
                  <span className="font-body text-xs text-ink-muted">Sale price</span>
                  <span className="font-mono text-sm font-semibold text-ink-strong tabular-nums">
                    {formatCurrency(asset.salePrice, asset.currency)}
                  </span>
                </div>
              )}
              {asset.size && (
                <div className="flex items-center justify-between gap-4">
                  <span className="font-body text-xs text-ink-muted">Floor area</span>
                  <span className="font-mono text-sm text-ink-body tabular-nums">
                    {asset.size}&nbsp;m²
                  </span>
                </div>
              )}
            </div>
          )}
        </div>

        {/* Data coverage — the trust instrument */}
        <div className="px-5 pt-4 pb-6">
          <div className="flex items-baseline justify-between mb-2">
            <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint">
              Supporting evidence
            </p>
            <div className="flex items-baseline gap-1">
              <span className={cn("font-mono text-base font-semibold tabular-nums leading-none", scoreTone)}>
                {coverageScore}
              </span>
              <span className="font-body text-[10px] text-ink-faint">/100</span>
              {presentCount !== undefined && (
                <>
                  <span className="mx-1 font-body text-[10px] text-ink-faint/30">·</span>
                  <span className="font-mono text-[10px] text-ink-faint">
                    {presentCount}/6 sources
                  </span>
                </>
              )}
            </div>
          </div>

          {/* Coverage summary — honest per band */}
          <p className="mb-4 font-body text-xs text-ink-muted leading-5">
            {coverageScore >= 70
              ? "Strong data foundation — multiple sources active. Assessments will be well-supported with relatively tight confidence intervals."
              : coverageScore >= 45
              ? "Partial coverage — assessment is possible, but confidence intervals will be moderate. Review which sources are active below."
              : "Limited coverage — only a few data sources are active at this location. Treat assessment results as indicative; confidence intervals will be wide."}
          </p>

          {/* Source checklist or loading skeleton */}
          {temporal ? (
            <div className="space-y-2">
              {temporal.sourceChecklist.map((source) => (
                <div key={source.source} className="flex items-center gap-2.5">
                  <span
                    className={cn(
                      "w-1.5 h-1.5 rounded-full shrink-0",
                      source.present ? "bg-status-high" : "bg-ink-faint/25"
                    )}
                    aria-hidden="true"
                  />
                  <span className={cn(
                    "font-body text-xs flex-1",
                    source.present ? "text-ink-muted" : "text-ink-faint"
                  )}>
                    {source.label}
                  </span>
                  {source.source === "edgeai_telemetry" && source.present && (
                    <span className="font-body text-[9px] uppercase tracking-wider text-accent-muted shrink-0">
                      active
                    </span>
                  )}
                  {source.source === "edgeai_telemetry" && !source.present && (
                    <span className="font-body text-[9px] uppercase tracking-wider text-ink-faint/50 shrink-0">
                      not yet
                    </span>
                  )}
                </div>
              ))}
            </div>
          ) : (
            // Skeleton while source data loads
            <div className="space-y-2" aria-label="Loading source data">
              {[78, 65, 72, 58, 68, 55].map((w, i) => (
                <div
                  key={i}
                  className="h-4 rounded-(--r-sm) animate-pulse bg-raised"
                  style={{ width: `${w}%` }}
                />
              ))}
            </div>
          )}

          {/* Temporal availability note */}
          <p className="mt-4 font-body text-[11px] text-ink-faint leading-relaxed">
            {temporal
              ? temporal.temporalConfidence
                ? `Sensor activity data is available — ${temporal.forecastHorizon ?? "short-term"} forecast horizon supported.`
                : "Sensor activity data not yet available at this location. Temporal intelligence expands as EdgeAI sensor coverage grows."
              : "Checking sensor coverage status…"}
          </p>
        </div>

      </div>

      {/* CTA — sticky at bottom; the single action from this view */}
      <div className="shrink-0 px-5 py-4 border-t border-hairline bg-surface">
        <Button
          type="button"
          variant="primary"
          size="lg"
          className="w-full"
          onClick={onProceed}
        >
          Choose a business concept
        </Button>
        <p className="mt-2.5 font-body text-xs text-ink-faint text-center">
          Select a concept and submit parameters to run the assessment.
        </p>
      </div>
    </div>
  );
}

// ── Verdict panel ─────────────────────────────────────────────────────────────
// Primary result view for the active concept-assessment.
// Feasibility score dominates. Valuation and coverage are supporting intelligence.

function VerdictPanel({
  assessment,
  isSaved,
  persistenceId,
  onSave,
  isSaving,
  saveError,
}: {
  assessment: ConceptAssessment;
  isSaved: boolean;
  persistenceId: string | undefined;
  onSave: () => void;
  isSaving: boolean;
  saveError: string | null;
}) {
  const { feasibility } = assessment;
  const colors = bandColors(feasibility.scoreBand);

  return (
    <div className="flex flex-col flex-1 overflow-hidden">
    <div className="flex-1 overflow-y-auto">
      <div className="px-5 pt-5 pb-5 border-b border-hairline">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Recommended next step
        </p>
        <div className={cn("rounded-(--r-md) border px-4 py-4", colors.bg, colors.border)}>
          <p className="font-display text-xl leading-tight text-ink-strong">
            {resultRecommendation(feasibility.scoreBand)}
          </p>
          <div className="mt-3 grid grid-cols-2 gap-3">
            <div>
              <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                Confidence
              </p>
              <p className="mt-1 font-mono text-sm font-semibold text-ink-body">
                CI {feasibility.confidenceInterval.low}-{feasibility.confidenceInterval.high}
              </p>
            </div>
            <div>
              <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                Business impact
              </p>
              <p className="mt-1 font-body text-sm font-medium text-ink-body">
                {resultImpact(feasibility.scoreBand)}
              </p>
            </div>
          </div>
          <p className="mt-3 font-body text-xs leading-5 text-ink-muted">
            {feasibility.narrative}
          </p>
        </div>
      </div>

      {/* Score is evidence, not the headline. */}
      <div className="px-5 pt-5 pb-5 border-b border-hairline">
        <div className="flex items-start gap-4">
          <div className="shrink-0">
            <div className="flex items-baseline gap-1">
              <span
                className={cn(
                  "font-mono font-semibold leading-none tracking-tight select-none tabular-nums",
                  "text-[4.5rem]",
                  colors.text
                )}
                aria-label={`Feasibility score: ${feasibility.feasibilityScore} out of 100`}
              >
                {feasibility.feasibilityScore}
              </span>
              <span className="font-mono text-lg text-ink-faint leading-none pb-1.5">
                /100
              </span>
            </div>
          </div>

          {/* Band badge + secondary metrics */}
          <div className="flex flex-col gap-2 pt-2 min-w-0">
            <div
              className={cn(
                "inline-flex items-center gap-1.5 self-start",
                "px-2.5 py-1 rounded-(--r-sm) border",
                "font-body text-xs font-medium uppercase tracking-wider",
                colors.bg, colors.border, colors.text
              )}
              role="status"
            >
              <span
                className={cn(
                  "w-1.5 h-1.5 rounded-full shrink-0",
                  feasibility.scoreBand === "strong"   ? "bg-verdict-strong" :
                  feasibility.scoreBand === "moderate" ? "bg-verdict-warn"   :
                                                          "bg-verdict-weak"
                )}
                aria-hidden="true"
              />
              {bandLabel(feasibility.scoreBand)}
            </div>

            <p className="font-mono text-xs text-ink-muted">
              CI{" "}
              <span className="text-ink-body font-medium tabular-nums">
                {feasibility.confidenceInterval.low}
              </span>
              {" – "}
              <span className="text-ink-body font-medium tabular-nums">
                {feasibility.confidenceInterval.high}
              </span>
              <span className="text-ink-faint ml-1">(95%)</span>
            </p>

            <p className="font-mono text-xs text-ink-muted">
              Success{" "}
              <span className="text-ink-body font-medium tabular-nums">
                {Math.round(feasibility.successLikelihood * 100)}%
              </span>
              <span className="text-ink-faint ml-1">likelihood</span>
            </p>

            <p className="font-body text-xs text-ink-faint">
              {formatNumber(feasibility.dataPointCount)} local data points
            </p>
          </div>
        </div>
      </div>

      {/* Contributing factors */}
      <div className="px-5 pt-5 pb-4 border-b border-hairline">
        <h3 className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Contributing factors
        </h3>
        <div className="space-y-2">
          {feasibility.contributingFactors.map((factor) => (
            <div key={factor.label} className="flex items-start justify-between gap-3">
              <div className="flex items-start gap-2 min-w-0">
                <span
                  className={cn(
                    "mt-0.5 w-1.5 h-1.5 rounded-full shrink-0",
                    factor.direction === "positive" ? "bg-verdict-strong" :
                    factor.direction === "negative" ? "bg-verdict-weak"   :
                                                       "bg-ink-faint"
                  )}
                  aria-hidden="true"
                />
                <span className="font-body text-sm text-ink-body truncate">
                  {factor.label}
                </span>
              </div>
              <span className="font-mono text-xs text-ink-muted shrink-0 tabular-nums">
                {factor.value}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Assessment narrative */}
      <div className="px-5 pt-5 pb-4 border-b border-hairline">
        <h3 className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Assessment narrative
        </h3>
        <p className="font-body text-sm text-ink-body leading-6">
          {feasibility.narrative}
        </p>
      </div>

      {/* Valuation intelligence — honest empty when null */}
      <div className="px-5 pt-4 pb-5 border-b border-hairline">
        <h3 className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Valuation intelligence
        </h3>
        <ValuationFace assessment={assessment} />
      </div>

      {/* Coverage intelligence — always shows source checklist */}
      <div className="px-5 pt-4 pb-5">
        <h3 className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Coverage intelligence
        </h3>
        <CoverageFace temporal={assessment.temporal} />
      </div>

      {/* Intelligence sources — footnote tier */}
      <div className="px-5 pt-4 pb-5 border-t border-hairline">
        <h3 className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2.5">
          Intelligence sources
        </h3>
        <ul className="space-y-1">
          {feasibility.dataSources.map((source) => (
            <li key={source} className="flex items-start gap-2 font-body text-xs text-ink-faint">
              <span
                className="mt-1.5 w-1 h-1 rounded-full bg-ink-faint/50 shrink-0"
                aria-hidden="true"
              />
              {source}
            </li>
          ))}
        </ul>
        <p className="mt-3 font-body text-xs text-ink-faint">
          Label quality:{" "}
          <span className="font-mono text-ink-muted">{feasibility.labelQuality}</span>
          {" · "}
          Generated{" "}
          <span className="font-mono text-ink-muted">
            {new Date(feasibility.generatedAt).toLocaleDateString("en-GB", {
              day: "numeric",
              month: "short",
              year: "numeric",
            })}
          </span>
        </p>
      </div>

      <div className="h-8" />
    </div>

    {/* Sticky save footer — per-concept grain */}
    <div className="shrink-0 border-t border-hairline px-5 py-4 bg-surface">
      {isSaved ? (
        <div className="flex items-center gap-2.5">
          {/* Filled bookmark */}
          <svg
            width="10" height="12" viewBox="0 0 10 12"
            fill="currentColor" className="shrink-0 text-accent-muted"
            aria-hidden="true"
          >
            <path d="M1 1h8a.5.5 0 01.5.5v10l-4.5-2.25L1 11.5V1.5A.5.5 0 011 1z" />
          </svg>
          <span className="font-body text-xs text-ink-muted">
            Saved assessment
          </span>
          {persistenceId && (
            <span className="ml-auto font-mono text-[10px] text-ink-faint/60 truncate max-w-24">
              {persistenceId.slice(0, 8)}…
            </span>
          )}
        </div>
      ) : (
        <>
          <Button
            type="button"
            variant="primary"
            size="md"
            className="w-full"
            onClick={onSave}
            disabled={isSaving}
          >
            {isSaving ? (
              <span className="flex items-center justify-center gap-2">
                <span
                  className="w-3.5 h-3.5 rounded-full border-2 border-transparent border-t-current animate-spin"
                  aria-hidden="true"
                />
                Saving…
              </span>
            ) : (
              "Save assessment"
            )}
          </Button>
          {saveError && (
            <p className="mt-2 font-body text-xs text-verdict-weak text-center">
              {saveError}
            </p>
          )}
          {!saveError && (
            <p className="mt-2 font-body text-xs text-ink-faint text-center">
              Saves the recommendation, confidence, factors, and narrative.
            </p>
          )}
        </>
      )}
    </div>
    </div>
  );
}

// ── Concept form ──────────────────────────────────────────────────────────────
// One guided step: choose concept first; operating parameters reveal below.

function ConceptForm({
  onSubmit,
  isSubmitting,
  errorMessage,
  existingConcepts,
}: {
  onSubmit: (concept: BusinessConcept, params: BusinessParameters) => void;
  isSubmitting: boolean;
  errorMessage: string | null;
  existingConcepts: BusinessConcept[];
}) {
  const [concept, setConcept] = useState<BusinessDomain | null>(null);
  const [hours, setHours]     = useState(DEFAULT_PARAMS.operatingHours);
  const [staffCount, setStaffCount] = useState(DEFAULT_PARAMS.staffCount);
  const [budget, setBudget]   = useState(DEFAULT_PARAMS.budget);

  function handleSubmit(e: FormEvent<HTMLFormElement>) {
    e.preventDefault();
    if (!concept) return;
    onSubmit(concept, {
      domain: concept,
      operatingHours: hours,
      staffCount,
      budget,
      currency: "EGP",
    });
  }

  const inputBase =
    "h-9 w-full rounded-(--r-md) border border-hairline bg-sunken px-3 font-mono text-sm text-ink-strong outline-none transition-colors focus:border-accent placeholder:text-ink-faint";

  return (
    <form onSubmit={handleSubmit} className="flex flex-col gap-0 h-full">
      <div className="flex-1 overflow-y-auto px-5 py-5 space-y-6">

        {/* Step 1: Business concept */}
        <section>
          <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint mb-3">
            1 — Business concept
          </p>
          <div className="grid grid-cols-1 gap-1">
            {(Object.entries(CONCEPT_LABELS) as [BusinessDomain, string][]).map(([value, label]) => {
              const alreadyRun = existingConcepts.includes(value);
              const isSelected = concept === value;
              return (
                <button
                  key={value}
                  type="button"
                  onClick={() => setConcept(value)}
                  className={cn(
                    "flex items-center justify-between w-full text-left px-3.5 py-2.5",
                    "rounded-(--r-md) border font-body text-sm transition-colors",
                    "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent",
                    isSelected
                      ? "bg-accent-soft border-accent text-ink-strong"
                      : alreadyRun
                      ? "bg-transparent border-hairline text-ink-faint opacity-40 cursor-not-allowed"
                      : "bg-transparent border-hairline text-ink-muted hover:border-rule hover:bg-raised hover:text-ink-body"
                  )}
                  disabled={alreadyRun}
                  aria-pressed={isSelected}
                  aria-disabled={alreadyRun}
                >
                  {label}
                  {alreadyRun && (
                    <span className="font-mono text-[10px] text-ink-faint ml-2 shrink-0">
                      assessed
                    </span>
                  )}
                </button>
              );
            })}
          </div>
        </section>

        {/* Steps 2–3: revealed after concept is chosen */}
        {concept && (
          <>
            {/* Operating hours */}
            <section>
              <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint mb-3">
                2 — Operating hours
              </p>
              <div className="space-y-3">
                <div className="grid grid-cols-2 gap-3">
                  <label className="block">
                    <span className="block font-body text-xs text-ink-muted mb-1.5">Opens</span>
                    <input
                      type="time"
                      value={hours.open}
                      onChange={(e) => setHours({ ...hours, open: e.target.value })}
                      className={inputBase}
                      required
                    />
                  </label>
                  <label className="block">
                    <span className="block font-body text-xs text-ink-muted mb-1.5">Closes</span>
                    <input
                      type="time"
                      value={hours.close}
                      onChange={(e) => setHours({ ...hours, close: e.target.value })}
                      className={inputBase}
                      required
                    />
                  </label>
                </div>
                <label className="block">
                  <span className="block font-body text-xs text-ink-muted mb-1.5">
                    Trading days/week —{" "}
                    <span className="font-mono text-ink-strong font-medium">
                      {hours.daysPerWeek}
                    </span>
                  </span>
                  <input
                    type="range"
                    min={1} max={7} step={1}
                    value={hours.daysPerWeek}
                    onChange={(e) => setHours({ ...hours, daysPerWeek: Number(e.target.value) })}
                    className="w-full accent-accent"
                  />
                  <div className="flex justify-between font-mono text-xs text-ink-faint mt-1 select-none">
                    <span>1</span><span>7</span>
                  </div>
                </label>
              </div>
            </section>

            {/* Staffing & budget */}
            <section>
              <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint mb-3">
                3 — Staffing &amp; capital
              </p>
              <div className="space-y-4">
                <label className="block">
                  <span className="block font-body text-xs text-ink-muted mb-1.5">
                    Staff count —{" "}
                    <span className="font-mono text-ink-strong font-medium">{staffCount}</span>
                  </span>
                  <input
                    type="range"
                    min={1} max={30} step={1}
                    value={staffCount}
                    onChange={(e) => setStaffCount(Number(e.target.value))}
                    className="w-full accent-accent"
                  />
                  <div className="flex justify-between font-mono text-xs text-ink-faint mt-1 select-none">
                    <span>1</span><span>30</span>
                  </div>
                </label>
                <label className="block">
                  <span className="block font-body text-xs text-ink-muted mb-1.5">
                    Setup capital (EGP)
                  </span>
                  <div className="relative">
                    <span className="absolute left-3 top-1/2 -translate-y-1/2 font-mono text-xs text-ink-faint pointer-events-none">
                      EGP
                    </span>
                    <input
                      type="number"
                      min={50000}
                      step={10000}
                      value={budget}
                      onChange={(e) => setBudget(Number(e.target.value))}
                      className={cn(inputBase, "pl-11")}
                      required
                    />
                  </div>
                </label>
              </div>
            </section>

            {errorMessage && (
              <p className="rounded-(--r-md) border border-verdict-weak/40 bg-verdict-weak-wash px-3 py-2.5 font-body text-sm text-ink-body">
                {errorMessage}
              </p>
            )}
          </>
        )}
      </div>

      {/* Sticky submit */}
      {concept && (
        <div className="px-5 py-4 border-t border-hairline bg-surface">
          <Button
            type="submit"
            variant="primary"
            size="lg"
            className="w-full"
            disabled={isSubmitting}
          >
            {isSubmitting ? "Analysing…" : "Get recommendation"}
          </Button>
          <p className="mt-2.5 font-body text-xs text-ink-faint text-center">
            Returns a recommendation, confidence interval, and supporting evidence.
          </p>
        </div>
      )}
    </form>
  );
}

// ── Asset header ──────────────────────────────────────────────────────────────
// Sticky at the top of the panel whenever an asset is selected.

function AssetHeader({
  asset,
  assessmentCount,
  onReset,
}: {
  asset: Asset;
  assessmentCount: number;
  onReset: () => void;
}) {
  return (
    <div className="shrink-0 border-b border-hairline bg-surface px-5 py-4">
      <div className="flex items-start justify-between gap-3">
        <div className="min-w-0">
          <p className="font-body text-[10px] text-ink-faint uppercase tracking-widest mb-1">
            {asset.area} · {asset.propertyType.replace(/_/g, " ")}
          </p>
          <h2
            className="font-display text-base font-semibold text-ink-strong leading-snug truncate"
            title={asset.address}
          >
            {asset.address}
          </h2>
          <div className="mt-1.5 flex items-center gap-3 flex-wrap">
            <span className="font-body text-xs text-ink-muted">
              Coverage{" "}
              <span className="font-mono text-ink-body tabular-nums">{asset.coverageScore}</span>
              <span className="text-ink-faint">/100</span>
            </span>
            {asset.monthlyRent && (
              <span className="font-body text-xs text-ink-muted">
                Rent{" "}
                <span className="font-mono text-ink-body tabular-nums">
                  {formatCurrency(asset.monthlyRent, asset.currency)}
                </span>
                <span className="text-ink-faint">/mo</span>
              </span>
            )}
            {assessmentCount > 0 && (
              <span className="font-body text-xs text-ink-faint">
                {assessmentCount}&nbsp;concept{assessmentCount !== 1 ? "s" : ""} run
              </span>
            )}
          </div>
        </div>

        <button
          type="button"
          onClick={onReset}
          aria-label="Clear workbench and deselect asset"
          className="shrink-0 w-7 h-7 flex items-center justify-center rounded-(--r-sm) text-ink-faint hover:text-ink-muted hover:bg-raised transition-colors"
        >
          <svg
            width="12" height="12" viewBox="0 0 12 12"
            fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"
            aria-hidden="true"
          >
            <line x1="1" y1="1" x2="11" y2="11" />
            <line x1="11" y1="1" x2="1" y2="11" />
          </svg>
        </button>
      </div>
    </div>
  );
}

// ── Idle panel ────────────────────────────────────────────────────────────────
// Shown when no asset is selected. Reads as a waiting intelligence workbench,
// not a marketing page.

function IdlePanel() {
  return (
    <div className="flex h-full flex-col px-8 py-10">
      {/* Workbench prompt — centred */}
      <div className="flex-1 flex flex-col justify-center text-center">
        {/* Icon: analytical crosshair in a grid container */}
        <div className="mx-auto mb-6 relative w-12 h-12 rounded-(--r-md) bg-sunken border border-hairline overflow-hidden flex items-center justify-center">
          <div
            className="absolute inset-0 bg-[linear-gradient(rgba(34,211,238,0.06)_1px,transparent_1px),linear-gradient(90deg,rgba(34,211,238,0.06)_1px,transparent_1px)] bg-size-[8px_8px]"
            aria-hidden="true"
          />
          <svg
            width="22" height="22" viewBox="0 0 22 22"
            fill="none" stroke="#60A5FA" strokeWidth="1.25" strokeLinecap="round"
            aria-hidden="true"
          >
            <circle cx="11" cy="11" r="6.5" />
            <circle cx="11" cy="11" r="2" />
            <line x1="11" y1="2"    x2="11" y2="4.5"  />
            <line x1="11" y1="17.5" x2="11" y2="20"   />
            <line x1="2"  y1="11"   x2="4.5"  y2="11" />
            <line x1="17.5" y1="11" x2="20"   y2="11" />
          </svg>
        </div>

        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
          Intelligence workbench
        </p>
        <h2 className="font-display text-xl font-semibold text-ink-strong mb-3 leading-tight">
          Select an asset
        </h2>
        <p className="font-body text-sm text-ink-muted leading-relaxed max-w-xs mx-auto">
          Click a marker on the map or search above to load the asset profile,
          then choose a business concept and run an assessment.
        </p>
      </div>

      {/* Coverage legend — anchored to bottom */}
      <div className="mt-auto pt-6">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
          Map coverage legend
        </p>
        <div className="space-y-2">
          {[
            { color: "#22C55E", label: "High coverage",  sub: "≥ 70" },
            { color: "#06B6D4", label: "Moderate",       sub: "45–69" },
            { color: "#F97316", label: "Lower",           sub: "< 45" },
            { color: "#64748B", label: "Not available",   sub: "" },
          ].map(({ color, label, sub }) => (
            <div key={label} className="flex items-center gap-3">
              <div
                className="w-2.5 h-2.5 rounded-full shrink-0"
                style={{ background: color }}
                aria-hidden="true"
              />
              <span className="font-body text-xs text-ink-muted">{label}</span>
              {sub && (
                <span className="font-mono text-[10px] text-ink-faint ml-auto">{sub}</span>
              )}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

// ── Analyzing panel ───────────────────────────────────────────────────────────
// Shown while a concept assessment mutation is in flight.
// Displays the actual concept being analyzed (not a generic placeholder).

function AnalyzingPanel({
  assetName,
  conceptLabel,
}: {
  assetName: string;
  conceptLabel: string;
}) {
  return (
    <div className="flex h-full flex-col items-center justify-center px-8 text-center">
      <div className="relative w-14 h-14 mb-6">
        <div className="absolute inset-0 rounded-full border-2 border-accent/20" />
        <div className="absolute inset-0 rounded-full border-2 border-transparent border-t-accent animate-spin" />
      </div>
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
        Analysing
      </p>
      <h2
        className="font-display text-lg font-semibold text-ink-strong mb-1 max-w-full truncate px-2"
        title={conceptLabel}
      >
        {conceptLabel}
      </h2>
      <p
        className="font-body text-sm text-ink-muted truncate max-w-full px-2"
        title={assetName}
      >
        at {assetName}
      </p>
      <p className="mt-4 font-body text-xs text-ink-faint max-w-50 leading-5">
        Running feasibility model — footfall, demographics, competition, rent pressure.
      </p>
    </div>
  );
}

// ── Main side panel ───────────────────────────────────────────────────────────

export function WorkspaceSidePanel({
  selectedAsset,
  currentAssessments,
  activeConceptId,
  activeAssessment,
  analyzingConcept,
  isAnalyzing,
  analysisError,
  assetTemporal,
  savedConceptIds,
  onSaveAssessment,
  isSaving,
  saveError,
  onSubmitConcept,
  onSwitchConcept,
  onReset,
}: WorkspaceSidePanelProps) {
  const [showForm, setShowForm] = useState(false);

  const assessmentEntries = Object.entries(currentAssessments) as [BusinessConcept, ConceptAssessment][];
  const hasAssessments    = assessmentEntries.length > 0;
  const existingConcepts  = assessmentEntries.map(([c]) => c);

  const handleSubmitConcept = (concept: BusinessConcept, params: BusinessParameters) => {
    setShowForm(false);
    onSubmitConcept(concept, params);
  };

  const renderPanelBody = () => {
    if (!selectedAsset) return <IdlePanel />;

    if (isAnalyzing) {
      return (
        <AnalyzingPanel
          assetName={selectedAsset.address}
          conceptLabel={
            analyzingConcept
              ? (CONCEPT_LABELS[analyzingConcept] ?? analyzingConcept)
              : "Feasibility model"
          }
        />
      );
    }

    if (hasAssessments && !showForm) {
      const activePersistenceId = activeAssessment
        ? savedConceptIds[activeAssessment.concept]
        : undefined;

      return (
        <>
          <ConceptHistoryList
            assessments={currentAssessments}
            activeConceptId={activeConceptId}
            savedConceptIds={savedConceptIds}
            onSwitch={(concept) => {
              onSwitchConcept(concept);
              setShowForm(false);
            }}
            onAddNew={() => setShowForm(true)}
          />

          {activeAssessment ? (
            <VerdictPanel
              assessment={activeAssessment}
              isSaved={!!activePersistenceId}
              persistenceId={activePersistenceId}
              onSave={() => onSaveAssessment(activeAssessment.concept)}
              isSaving={isSaving}
              saveError={saveError}
            />
          ) : (
            <div className="flex flex-1 items-center justify-center p-8 text-center">
              <p className="font-body text-sm text-ink-muted">
                Select a concept above to view its assessment.
              </p>
            </div>
          )}
        </>
      );
    }

    // Asset selected, no assessments yet — show the intelligence brief before
    // the user commits to a concept. This is the deliberate pre-concept step.
    if (!hasAssessments && !showForm) {
      return (
        <AssetIntelligencePanel
          asset={selectedAsset}
          temporal={assetTemporal}
          onProceed={() => setShowForm(true)}
        />
      );
    }

    return (
      <ConceptForm
        onSubmit={handleSubmitConcept}
        isSubmitting={isAnalyzing}
        errorMessage={analysisError}
        existingConcepts={existingConcepts}
      />
    );
  };

  return (
    <aside
      className={cn(
        "flex h-full min-h-0 flex-col overflow-hidden bg-surface",
        "border-b border-hairline lg:border-b-0 lg:border-l"
      )}
      aria-label="Assessment panel"
    >
      {selectedAsset && (
        <AssetHeader
          asset={selectedAsset}
          assessmentCount={assessmentEntries.length}
          onReset={onReset}
        />
      )}

      <div className={cn(
        "flex flex-col min-h-0",
        selectedAsset ? "flex-1 overflow-hidden" : "h-full"
      )}>
        {renderPanelBody()}
      </div>
    </aside>
  );
}
