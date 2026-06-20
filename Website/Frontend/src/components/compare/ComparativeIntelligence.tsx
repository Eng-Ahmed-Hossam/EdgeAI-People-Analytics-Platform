"use client";

import { useCallback, useState } from "react";
import Link from "next/link";
import { useQuery } from "@tanstack/react-query";
import { getAssets, runConceptAssessment, queryKeys } from "@/lib/api";
import {
  bandColors,
  bandLabel,
  cn,
  formatCurrency,
} from "@/lib/utils";
import type { Asset } from "@/types/asset";
import type {
  BusinessConcept,
  ConceptAssessment,
} from "@/types/assessment-v2";
import type { BusinessDomain } from "@/types/parameters";

// ── Constants ─────────────────────────────────────────────────────────────────

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

const MAX_ASSETS = 4;
const MIN_ASSETS = 2;

const DEFAULT_PARAMS = {
  operatingHours: { open: "09:00", close: "22:00", daysPerWeek: 6 },
  staffCount: 5,
  budget: 500_000,
  currency: "EGP",
};

// ── Helpers ───────────────────────────────────────────────────────────────────

function shortName(asset: Asset): string {
  return asset.area;
}

function coverageTone(score: number): string {
  return score >= 70 ? "text-status-high" :
         score >= 45 ? "text-status-med"  :
                       "text-status-low";
}

function confidenceLabel(assessment: ConceptAssessment): "High" | "Medium" | "Low" {
  const width =
    assessment.feasibility.confidenceInterval.high -
    assessment.feasibility.confidenceInterval.low;
  if (width <= 14 && assessment.temporal.coverageScore >= 70) return "High";
  if (width <= 26 && assessment.temporal.coverageScore >= 45) return "Medium";
  return "Low";
}

function impactLabel(assessment: ConceptAssessment): string {
  if (assessment.feasibility.scoreBand === "strong") return "Best current opportunity";
  if (assessment.feasibility.scoreBand === "moderate") return "Conditional opportunity";
  return "High-risk option";
}

function availabilityLabel(asset: Asset): string {
  if (asset.availabilityStatus === "available") return "Available";
  if (asset.availabilityStatus === "reserved")  return "Reserved";
  if (asset.availabilityStatus === "occupied")  return "Occupied";
  return "Sold";
}

// ── Derived comparison data ───────────────────────────────────────────────────

interface RankedResult {
  asset: Asset;
  assessment: ConceptAssessment;
  rank: number;
}

function rankResults(
  results: Record<string, ConceptAssessment>,
  assets: Asset[]
): RankedResult[] {
  return assets
    .filter((a) => !!results[a.id])
    .map((a) => ({ asset: a, assessment: results[a.id]! }))
    .sort(
      (a, b) =>
        b.assessment.feasibility.feasibilityScore -
        a.assessment.feasibility.feasibilityScore
    )
    .map((item, i) => ({ ...item, rank: i + 1 }));
}

function winnerStrengths(assessment: ConceptAssessment): Array<{ label: string; value: string }> {
  return assessment.feasibility.contributingFactors
    .filter((f) => f.direction === "positive")
    .sort((a, b) => b.weight - a.weight)
    .slice(0, 3)
    .map((f) => ({ label: f.label, value: f.value }));
}

function winnerWeaknesses(assessment: ConceptAssessment): Array<{ label: string; value: string }> {
  return assessment.feasibility.contributingFactors
    .filter((f) => f.direction === "negative")
    .sort((a, b) => b.weight - a.weight)
    .slice(0, 2)
    .map((f) => ({ label: f.label, value: f.value }));
}

function coverageWarning(ranked: RankedResult[]): string | null {
  if (ranked.length < 2) return null;
  const winner = ranked[0];
  const winnerCoverage = winner.assessment.temporal.coverageScore;
  const maxOtherCoverage = Math.max(
    ...ranked.slice(1).map((r) => r.assessment.temporal.coverageScore)
  );
  if (maxOtherCoverage - winnerCoverage >= 20) {
    return `${shortName(winner.asset)} has lower data coverage (${winnerCoverage}/100) than some alternatives (up to ${maxOtherCoverage}/100). The recommendation is based on the available evidence; confidence intervals for ${shortName(winner.asset)} are wider.`;
  }
  return null;
}

function runnerUpAdvantages(
  winner: RankedResult,
  runnerUp: RankedResult
): Array<{ label: string; detail: string }> {
  const advantages: Array<{ label: string; detail: string }> = [];
  for (const factor of runnerUp.assessment.feasibility.contributingFactors) {
    if (factor.direction !== "positive") continue;
    const winnerFactor = winner.assessment.feasibility.contributingFactors.find(
      (f) => f.label === factor.label
    );
    if (winnerFactor && winnerFactor.direction === "negative") {
      advantages.push({
        label: factor.label,
        detail: `${shortName(runnerUp.asset)}: ${factor.value} vs. ${shortName(winner.asset)}: ${winnerFactor.value}`,
      });
      if (advantages.length >= 2) break;
    }
  }
  return advantages;
}

// ── Phase: Setup ──────────────────────────────────────────────────────────────

function ConceptPicker({
  selected,
  onSelect,
}: {
  selected: BusinessConcept | null;
  onSelect: (concept: BusinessConcept) => void;
}) {
  return (
    <div>
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
        Business concept
      </p>
      <div className="grid grid-cols-2 gap-1.5 sm:grid-cols-3">
        {(Object.entries(CONCEPT_LABELS) as [BusinessDomain, string][]).map(
          ([value, label]) => (
            <button
              key={value}
              type="button"
              onClick={() => onSelect(value)}
              aria-pressed={selected === value}
              className={cn(
                "px-3.5 py-2.5 rounded-(--r-md) border font-body text-sm text-left transition-colors",
                "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent",
                selected === value
                  ? "bg-accent-soft border-accent text-ink-strong"
                  : "border-hairline text-ink-muted hover:border-rule hover:bg-raised hover:text-ink-body"
              )}
            >
              {label}
            </button>
          )
        )}
      </div>
    </div>
  );
}

function AssetPicker({
  assets,
  selected,
  onToggle,
  isLoading,
  isError,
}: {
  assets: Asset[];
  selected: Set<string>;
  onToggle: (id: string) => void;
  isLoading: boolean;
  isError: boolean;
}) {
  const atMax = selected.size >= MAX_ASSETS;

  return (
    <div>
      <div className="flex items-baseline justify-between mb-3">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint">
          Locations to compare
        </p>
        <p className="font-mono text-[10px] text-ink-faint">
          {selected.size}/{MAX_ASSETS} selected
        </p>
      </div>

      {isError && (
        <p className="rounded-(--r-md) border border-verdict-weak/40 bg-verdict-weak-wash px-3 py-2.5 font-body text-sm text-ink-body">
          Locations could not be loaded. Check the API seam.
        </p>
      )}

      {isLoading && !isError && (
        <div className="space-y-1.5">
          {[0, 1, 2, 3].map((i) => (
            <div key={i} className="h-14 animate-pulse rounded-(--r-md) bg-raised" />
          ))}
        </div>
      )}

      {!isLoading && !isError && (
        <div className="space-y-1.5">
          {assets.map((asset) => {
            const isSelected = selected.has(asset.id);
            const isDisabled = !isSelected && atMax;
            const coverageScore = asset.coverageScore;

            return (
              <button
                key={asset.id}
                type="button"
                onClick={() => !isDisabled && onToggle(asset.id)}
                aria-pressed={isSelected}
                disabled={isDisabled}
                className={cn(
                  "w-full rounded-(--r-md) border px-4 py-3 text-left transition-colors",
                  "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent",
                  "disabled:opacity-40 disabled:cursor-not-allowed",
                  isSelected
                    ? "border-accent bg-accent-soft"
                    : "border-hairline hover:border-rule hover:bg-raised/60"
                )}
              >
                <div className="flex items-center justify-between gap-3">
                  <div className="min-w-0">
                    <p className="truncate font-body text-sm font-medium text-ink-strong">
                      {asset.address}
                    </p>
                    <p className="mt-0.5 font-body text-xs text-ink-faint">
                      {asset.area} · {availabilityLabel(asset)}
                      {asset.monthlyRent && (
                        <span className="ml-2 font-mono">
                          {formatCurrency(asset.monthlyRent, asset.currency)}/mo
                        </span>
                      )}
                    </p>
                  </div>
                  <div className="shrink-0 text-right">
                    <span className={cn("font-mono text-sm font-semibold tabular-nums", coverageTone(coverageScore))}>
                      {coverageScore}
                    </span>
                    <p className="font-body text-[10px] text-ink-faint mt-0.5">cov.</p>
                  </div>
                </div>
              </button>
            );
          })}
        </div>
      )}
    </div>
  );
}

function SetupView({
  assets,
  assetsLoading,
  assetsError,
  concept,
  setConcept,
  selectedIds,
  toggleId,
  onRun,
}: {
  assets: Asset[];
  assetsLoading: boolean;
  assetsError: boolean;
  concept: BusinessConcept | null;
  setConcept: (c: BusinessConcept) => void;
  selectedIds: Set<string>;
  toggleId: (id: string) => void;
  onRun: () => void;
}) {
  const canRun = !!concept && selectedIds.size >= MIN_ASSETS;

  return (
    <div className="mx-auto max-w-2xl px-5 py-10 lg:px-8">
      <div className="mb-8">
        <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
          Decision briefing
        </p>
        <h1 className="mt-2 font-display text-3xl text-ink-strong">
          Choose the strongest location
        </h1>
        <p className="mt-3 font-body text-sm text-ink-muted leading-6 max-w-lg">
          Choose a business concept and select {MIN_ASSETS}–{MAX_ASSETS} locations. The comparison
          recommends the strongest option, explains confidence, and shows the tradeoffs.
        </p>
      </div>

      <div className="space-y-8">
        <ConceptPicker selected={concept} onSelect={setConcept} />

        <AssetPicker
          assets={assets}
          selected={selectedIds}
          onToggle={toggleId}
          isLoading={assetsLoading}
          isError={assetsError}
        />

        <div>
          <button
            type="button"
            onClick={onRun}
            disabled={!canRun}
            className={cn(
              "w-full rounded-(--r-md) border h-11 px-5 font-body text-base font-medium transition-colors",
              "focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-accent",
              "disabled:opacity-40 disabled:cursor-not-allowed",
              canRun
                ? "bg-accent border-transparent text-ink-strong hover:bg-accent-hover cursor-pointer"
                : "bg-accent border-transparent text-ink-strong"
            )}
          >
            {canRun
              ? `Compare ${selectedIds.size} location${selectedIds.size !== 1 ? "s" : ""} — ${CONCEPT_LABELS[concept!]}`
              : `Select concept and ${MIN_ASSETS}–${MAX_ASSETS} locations`}
          </button>
          {canRun && (
            <p className="mt-2 font-body text-xs text-ink-faint text-center">
              Uses standard operating parameters and ranks every location against the same concept.
            </p>
          )}
        </div>
      </div>
    </div>
  );
}

// ── Phase: Running ────────────────────────────────────────────────────────────

function RunningView({
  assets,
  completedIds,
}: {
  assets: Asset[];
  completedIds: Set<string>;
}) {
  return (
    <div className="mx-auto max-w-md px-5 py-16 text-center">
      <div className="relative w-14 h-14 mx-auto mb-6">
        <div className="absolute inset-0 rounded-full border-2 border-accent/20" />
        <div className="absolute inset-0 rounded-full border-2 border-transparent border-t-accent animate-spin" />
      </div>
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
        Running comparison
      </p>
      <p className="font-display text-xl text-ink-strong mb-6">
        {completedIds.size} of {assets.length} scored
      </p>

      <div className="space-y-2 text-left">
        {assets.map((asset) => {
          const done = completedIds.has(asset.id);
          return (
            <div
              key={asset.id}
              className={cn(
                "flex items-center gap-3 rounded-(--r-md) border px-3 py-2.5 transition-colors",
                done ? "border-verdict-good/40 bg-verdict-strong-wash" : "border-hairline"
              )}
            >
              <span
                className={cn(
                  "w-1.5 h-1.5 rounded-full shrink-0",
                  done ? "bg-verdict-strong" : "bg-ink-faint/30 animate-pulse"
                )}
                aria-hidden="true"
              />
              <span className={cn("font-body text-sm", done ? "text-ink-body" : "text-ink-muted")}>
                {asset.address}
              </span>
              {done && (
                <span className="ml-auto font-mono text-[10px] text-verdict-strong shrink-0">
                  done
                </span>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}

// ── Phase: Results — winner hero ──────────────────────────────────────────────

function WinnerHero({
  ranked,
  concept,
}: {
  ranked: RankedResult[];
  concept: BusinessConcept;
}) {
  const winner = ranked[0];
  const runnerUp = ranked[1] ?? null;
  const { feasibility, temporal, valuation } = winner.assessment;
  const colors = bandColors(feasibility.scoreBand);
  const scoreDelta = runnerUp
    ? feasibility.feasibilityScore - runnerUp.assessment.feasibility.feasibilityScore
    : null;
  const strengths = winnerStrengths(winner.assessment);
  const warning = coverageWarning(ranked);
  const confidence = confidenceLabel(winner.assessment);

  return (
    <section className="border-b border-hairline pb-7 mb-7">
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-4">
        Recommendation — {CONCEPT_LABELS[concept]}
      </p>

      <div className="mb-6 rounded-(--r-lg) border border-accent/35 bg-accent-soft px-5 py-5">
        <div className="flex flex-col gap-4 lg:flex-row lg:items-end lg:justify-between">
          <div className="min-w-0">
            <h1
              className="font-display text-3xl font-semibold text-ink-strong leading-tight"
              title={winner.asset.address}
            >
              Recommended: {winner.asset.address}
            </h1>
            <p className="mt-2 font-body text-sm leading-6 text-ink-muted">
              {impactLabel(winner.assessment)} for {CONCEPT_LABELS[concept]}.
              {scoreDelta !== null && scoreDelta > 0
                ? ` Leads the next option by ${scoreDelta} points.`
                : " Best option among the selected candidates."}
            </p>
          </div>
          <div className="grid grid-cols-2 gap-3 sm:min-w-72">
            <div>
              <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                Confidence
              </p>
              <p className="mt-1 font-mono text-sm font-semibold text-ink-strong">
                {confidence}
              </p>
            </div>
            <div>
              <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
        Next action
              </p>
              <p className="mt-1 font-body text-sm font-medium text-ink-body">
                Validate winner
              </p>
            </div>
          </div>
        </div>
      </div>

      <div className="grid gap-6 lg:grid-cols-[minmax(0,1fr)_340px]">
        {/* Left: winner identity + score */}
        <div>
          <h2
            className="font-display text-3xl font-semibold text-ink-strong leading-tight mb-4"
            title={winner.asset.address}
          >
            Confidence and evidence
          </h2>

          <div className="flex items-end gap-4 mb-4">
            <div className="flex items-baseline gap-1">
              <span
                className={cn(
                  "font-mono font-semibold leading-none tracking-tight tabular-nums select-none",
                  "text-[4.5rem]",
                  colors.text
                )}
                aria-label={`Feasibility score: ${feasibility.feasibilityScore} out of 100`}
              >
                {feasibility.feasibilityScore}
              </span>
              <span className="font-mono text-lg text-ink-faint pb-1.5">/100</span>
            </div>

            <div className="flex flex-col gap-1.5 pb-1">
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

              {scoreDelta !== null && scoreDelta > 0 && (
                <p className="font-mono text-xs text-ink-muted">
                  +<span className="text-ink-body tabular-nums">{scoreDelta}</span>{" "}
                  <span className="text-ink-faint">
                    pts vs. {shortName(runnerUp!.asset)}
                  </span>
                </p>
              )}
            </div>
          </div>

          {/* Key strengths */}
          {strengths.length > 0 && (
            <div className="mt-2">
              <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
                Key advantages
              </p>
              <ul className="space-y-1.5">
                {strengths.map((s) => (
                  <li key={s.label} className="flex items-start gap-2">
                    <span className="mt-1.5 w-1.5 h-1.5 rounded-full bg-verdict-strong shrink-0" aria-hidden="true" />
                    <span className="font-body text-sm text-ink-body">
                      {s.label}
                      <span className="ml-2 font-mono text-xs text-ink-muted tabular-nums">{s.value}</span>
                    </span>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>

        {/* Right: supporting data */}
        <div className="rounded-(--r-lg) border border-hairline bg-surface px-5 py-5 space-y-4">
          <div>
            <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2.5">
              Listed terms
            </p>
            <div className="space-y-2">
              {winner.asset.monthlyRent && (
                <div className="flex justify-between items-center gap-4">
                  <span className="font-body text-xs text-ink-muted">Monthly rent</span>
                  <span className="font-mono text-sm font-medium text-ink-strong tabular-nums">
                    {formatCurrency(winner.asset.monthlyRent, winner.asset.currency)}/mo
                  </span>
                </div>
              )}
              {winner.asset.salePrice && (
                <div className="flex justify-between items-center gap-4">
                  <span className="font-body text-xs text-ink-muted">Sale price</span>
                  <span className="font-mono text-sm font-medium text-ink-strong tabular-nums">
                    {formatCurrency(winner.asset.salePrice, winner.asset.currency)}
                  </span>
                </div>
              )}
              {winner.asset.size && (
                <div className="flex justify-between items-center gap-4">
                  <span className="font-body text-xs text-ink-muted">Floor area</span>
                  <span className="font-mono text-sm text-ink-body tabular-nums">{winner.asset.size} m²</span>
                </div>
              )}
              {valuation && (
                <div className="flex justify-between items-center gap-4">
                  <span className="font-body text-xs text-ink-muted">Market position</span>
                  <span className={cn(
                    "inline-flex items-center px-2 py-0.5 rounded-(--r-sm) border",
                    "font-body text-[10px] font-medium uppercase tracking-wide",
                    valuation.priceDelta === "below_market"
                      ? "bg-verdict-strong-wash border-verdict-good/60 text-verdict-strong"
                      : valuation.priceDelta === "above_market"
                      ? "bg-verdict-weak-wash border-verdict-weak/40 text-verdict-weak"
                      : "border-hairline text-ink-muted"
                  )}>
                    {valuation.priceDelta === "below_market" ? "Below market" :
                     valuation.priceDelta === "above_market" ? "Above market" : "At market"}
                  </span>
                </div>
              )}
            </div>
          </div>

          <div className="border-t border-hairline pt-4">
            <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2.5">
              Data coverage
            </p>
            <div className="flex items-center gap-2">
              <span className={cn("font-mono text-lg font-semibold tabular-nums", coverageTone(temporal.coverageScore))}>
                {temporal.coverageScore}
              </span>
              <span className="font-body text-xs text-ink-faint">/100</span>
              <span className="mx-1 text-ink-faint/30">·</span>
              <span className="font-mono text-xs text-ink-faint">
                {temporal.sourceChecklist.filter((s) => s.present).length}/6 sources
              </span>
            </div>
            {warning && (
              <p className="mt-2 font-body text-[11px] text-verdict-warn leading-5">
                {warning}
              </p>
            )}
          </div>
        </div>
      </div>
      <div className="mt-5 flex flex-wrap gap-2">
        <Link
          href="/report"
          className="inline-flex h-9 items-center rounded-(--r-md) border border-accent bg-accent px-4 font-body text-sm font-semibold text-ink-strong hover:bg-accent-hover"
        >
          Generate decision memo
        </Link>
        <button
          type="button"
          disabled
          title="Field workflow requires future operating contracts."
          className="inline-flex h-9 items-center rounded-(--r-md) border border-hairline px-4 font-body text-sm text-ink-faint opacity-60"
        >
          Schedule field validation
        </button>
      </div>
    </section>
  );
}

// ── Tradeoffs ─────────────────────────────────────────────────────────────────

function TradeoffsSection({ ranked }: { ranked: RankedResult[] }) {
  const winner = ranked[0];
  const runnerUp = ranked[1] ?? null;

  const weaknesses = winnerWeaknesses(winner.assessment);
  const runnerUpAdvs = runnerUp ? runnerUpAdvantages(winner, runnerUp) : [];

  if (weaknesses.length === 0 && runnerUpAdvs.length === 0) return null;

  return (
    <section className="mb-7">
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
        Tradeoffs
      </p>
      <div className="grid gap-4 md:grid-cols-2">
        {weaknesses.length > 0 && (
          <div className="rounded-(--r-md) border border-hairline bg-surface px-5 py-4">
            <p className="font-body text-xs font-medium text-ink-muted mb-3">
              Where {shortName(winner.asset)} falls short
            </p>
            <ul className="space-y-2">
              {weaknesses.map((w) => (
                <li key={w.label} className="flex items-start gap-2">
                  <span className="mt-1.5 w-1.5 h-1.5 rounded-full bg-verdict-weak shrink-0" aria-hidden="true" />
                  <span className="font-body text-sm text-ink-body">
                    {w.label}
                    <span className="ml-2 font-mono text-xs text-ink-muted tabular-nums">{w.value}</span>
                  </span>
                </li>
              ))}
            </ul>
          </div>
        )}

        {runnerUpAdvs.length > 0 && runnerUp && (
          <div className="rounded-(--r-md) border border-hairline bg-surface px-5 py-4">
            <p className="font-body text-xs font-medium text-ink-muted mb-3">
              Where {shortName(runnerUp.asset)} scores higher
            </p>
            <ul className="space-y-2">
              {runnerUpAdvs.map((a) => (
                <li key={a.label} className="flex items-start gap-2">
                  <span className="mt-1.5 w-1.5 h-1.5 rounded-full bg-status-med shrink-0" aria-hidden="true" />
                  <span className="font-body text-sm text-ink-body">
                    {a.label}
                    <span className="ml-2 font-mono text-xs text-ink-muted">{a.detail}</span>
                  </span>
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </section>
  );
}

// ── Candidate table ───────────────────────────────────────────────────────────
// Calm side-by-side data table — all candidates, tabular numbers aligned.
// Winner gets a left-edge accent. Low-coverage assets are visually distinguished.

function CandidateTable({ ranked }: { ranked: RankedResult[] }) {
  return (
    <section className="mb-7">
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-3">
        All candidates
      </p>

      <div className="rounded-(--r-lg) border border-hairline bg-surface overflow-hidden">
        {/* Header */}
        <div className="grid grid-cols-[minmax(0,1fr)_56px_88px_80px_72px_96px] gap-0 border-b border-hairline bg-sunken px-4 py-2">
          {["Location", "Score", "Interval", "Band", "Cov.", "Rent"].map((h) => (
            <p key={h} className="font-body text-[10px] font-medium uppercase tracking-wider text-ink-faint">
              {h}
            </p>
          ))}
        </div>

        {/* Rows */}
        {ranked.map((item) => {
          const { feasibility, temporal, valuation } = item.assessment;
          const colors = bandColors(feasibility.scoreBand);
          const isWinner = item.rank === 1;

          return (
            <div
              key={item.asset.id}
              className={cn(
                "relative grid grid-cols-[minmax(0,1fr)_56px_88px_80px_72px_96px]",
                "gap-0 border-b border-hairline last:border-b-0 px-4 py-3.5",
                isWinner ? "bg-surface" : "hover:bg-raised/30 transition-colors"
              )}
            >
              {/* Left-edge winner accent */}
              {isWinner && (
                <span
                  className="absolute left-0 top-0 bottom-0 w-0.75 rounded-r-xs bg-verdict-strong"
                  aria-hidden="true"
                />
              )}

              {/* Location */}
              <div className="min-w-0 pr-3">
                <p className={cn("font-body text-sm truncate", isWinner ? "font-medium text-ink-strong" : "text-ink-body")}>
                  {item.asset.address}
                </p>
                <p className="font-body text-xs text-ink-faint mt-0.5">{item.asset.area}</p>
              </div>

              {/* Score */}
              <p className={cn("font-mono text-sm font-semibold tabular-nums self-center", colors.text)}>
                {feasibility.feasibilityScore}
              </p>

              {/* CI */}
              <p className="font-mono text-xs text-ink-muted self-center tabular-nums">
                {feasibility.confidenceInterval.low}–{feasibility.confidenceInterval.high}
              </p>

              {/* Band */}
              <div className="self-center">
                <span className={cn(
                  "inline-flex items-center gap-1 px-1.5 py-0.5 rounded-(--r-sm) border",
                  "font-body text-[10px] font-medium",
                  colors.bg, colors.border, colors.text
                )}>
                  {feasibility.scoreBand === "strong" ? "Strong" :
                   feasibility.scoreBand === "moderate" ? "Mod." : "Weak"}
                </span>
              </div>

              {/* Coverage */}
              <p className={cn("font-mono text-sm font-semibold tabular-nums self-center", coverageTone(temporal.coverageScore))}>
                {temporal.coverageScore}
              </p>

              {/* Rent / market position */}
              <div className="self-center">
                {item.asset.monthlyRent ? (
                  <p className="font-mono text-xs text-ink-body tabular-nums">
                    {formatCurrency(item.asset.monthlyRent, item.asset.currency)}/mo
                  </p>
                ) : (
                  <p className="font-mono text-xs text-ink-faint">—</p>
                )}
                {valuation && (
                  <p className={cn(
                    "font-body text-[10px] mt-0.5",
                    valuation.priceDelta === "below_market" ? "text-verdict-strong" :
                    valuation.priceDelta === "above_market" ? "text-verdict-weak"   : "text-ink-faint"
                  )}>
                    {valuation.priceDelta === "below_market" ? "Below mkt" :
                     valuation.priceDelta === "above_market" ? "Above mkt" : "At market"}
                  </p>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
}

// ── Detailed evidence ─────────────────────────────────────────────────────────
// Per-asset breakdown — contributing factors, valuation summary, coverage.
// Deferred to the bottom: the recommendation is the answer; this is the evidence.

function AssetEvidenceCard({ item }: { item: RankedResult }) {
  const { feasibility, valuation, temporal } = item.assessment;
  const colors = bandColors(feasibility.scoreBand);
  const isWinner = item.rank === 1;
  const presentSources = temporal.sourceChecklist.filter((s) => s.present).length;

  return (
    <div className={cn(
      "rounded-(--r-lg) border bg-surface",
      isWinner ? "border-verdict-good/40" : "border-hairline"
    )}>
      {/* Card header */}
      <div className="flex items-start justify-between gap-3 border-b border-hairline px-5 py-4">
        <div className="min-w-0">
          {isWinner && (
            <p className="font-body text-[9px] font-medium uppercase tracking-widest text-verdict-strong mb-1">
              Recommended
            </p>
          )}
          <h3 className="font-display text-base font-semibold text-ink-strong truncate" title={item.asset.address}>
            {item.asset.address}
          </h3>
          <p className="mt-0.5 font-body text-xs text-ink-faint">{item.asset.area} · {availabilityLabel(item.asset)}</p>
        </div>
        <div className="shrink-0 text-right">
          <span className={cn("font-mono text-2xl font-semibold tabular-nums leading-none", colors.text)}>
            {feasibility.feasibilityScore}
          </span>
          <p className="font-mono text-xs text-ink-faint mt-0.5">
            CI {feasibility.confidenceInterval.low}–{feasibility.confidenceInterval.high}
          </p>
        </div>
      </div>

      {/* Contributing factors */}
      <div className="px-5 py-4 border-b border-hairline">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2.5">
          Contributing factors
        </p>
        <div className="space-y-2">
          {feasibility.contributingFactors.slice(0, 5).map((factor) => (
            <div key={factor.label} className="flex items-center justify-between gap-3">
              <div className="flex items-center gap-2 min-w-0">
                <span
                  className={cn(
                    "w-1.5 h-1.5 rounded-full shrink-0",
                    factor.direction === "positive" ? "bg-verdict-strong" :
                    factor.direction === "negative" ? "bg-verdict-weak"   : "bg-ink-faint"
                  )}
                  aria-hidden="true"
                />
                <span className="font-body text-xs text-ink-body truncate">{factor.label}</span>
              </div>
              <span className="font-mono text-xs text-ink-muted shrink-0 tabular-nums">{factor.value}</span>
            </div>
          ))}
        </div>
      </div>

      {/* Valuation */}
      <div className="px-5 py-4 border-b border-hairline">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2.5">
          Valuation intelligence
        </p>
        {valuation ? (
          <div className="space-y-2">
            <div className="flex justify-between items-center gap-4">
              <span className="font-body text-xs text-ink-muted">Fair value</span>
              <span className="font-mono text-sm font-medium text-ink-strong tabular-nums">
                {formatCurrency(valuation.fairPriceEstimate, valuation.currency)}/mo
              </span>
            </div>
            <div className="flex justify-between items-center gap-4">
              <span className="font-body text-xs text-ink-muted">Confidence</span>
              <span className={cn(
                "font-body text-xs capitalize",
                valuation.confidence === "high"   ? "text-verdict-strong" :
                valuation.confidence === "medium" ? "text-verdict-warn"   : "text-ink-muted"
              )}>
                {valuation.confidence}
              </span>
            </div>
          </div>
        ) : (
          <p className="font-body text-xs text-ink-faint leading-5">
            Insufficient market transaction data. Valuation intelligence becomes available as comparable activity accumulates.
          </p>
        )}
      </div>

      {/* Coverage */}
      <div className="px-5 py-4">
        <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
          Coverage
        </p>
        <div className="flex items-center gap-2">
          <span className={cn("font-mono text-sm font-semibold tabular-nums", coverageTone(temporal.coverageScore))}>
            {temporal.coverageScore}
          </span>
          <span className="font-body text-xs text-ink-faint">/100</span>
          <span className="mx-1 text-ink-faint/30">·</span>
          <span className="font-mono text-xs text-ink-faint">{presentSources}/6 sources</span>
          {temporal.temporalConfidence && (
            <>
              <span className="mx-1 text-ink-faint/30">·</span>
              <span className="font-body text-xs text-accent-muted">sensor data available</span>
            </>
          )}
        </div>
        {!temporal.temporalConfidence && (
          <p className="mt-1.5 font-body text-[11px] text-ink-faint leading-5">
            No sensor activity data at this location yet.
          </p>
        )}
      </div>
    </div>
  );
}

function DetailedEvidence({ ranked }: { ranked: RankedResult[] }) {
  return (
    <section>
      <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-4">
        Detailed evidence
      </p>
      <div className="grid gap-5 md:grid-cols-2">
        {ranked.map((item) => (
          <AssetEvidenceCard key={item.asset.id} item={item} />
        ))}
      </div>

      {/* Methodology note */}
      <div className="mt-6 rounded-(--r-md) border border-hairline bg-sunken px-5 py-4">
        <p className="font-body text-xs text-ink-faint leading-5">
          All scores use standard operating parameters (9 AM–10 PM, 6 days/week, 5 staff, EGP 500K capital).
          Scores reflect feasibility for the selected concept at each location.
          Revenue ranges and exact profit figures are not shown — the score and confidence interval are the product outputs.
          Data sources and label quality are indicated per location in the evidence cards.
        </p>
      </div>
    </section>
  );
}

// ── Phase: Results ─────────────────────────────────────────────────────────────

function ResultsView({
  ranked,
  concept,
  onReset,
}: {
  ranked: RankedResult[];
  concept: BusinessConcept;
  onReset: () => void;
}) {
  return (
    <div className="mx-auto max-w-300 px-5 py-8 lg:px-8">
      {/* Page header */}
      <div className="flex items-start justify-between gap-4 mb-8">
        <div>
          <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
          Decision briefing
          </p>
          <p className="mt-1 font-body text-sm text-ink-muted">
            {CONCEPT_LABELS[concept]} · {ranked.length} locations compared
          </p>
        </div>
        <button
          type="button"
          onClick={onReset}
          className="shrink-0 rounded-(--r-md) border border-hairline px-3.5 py-2 font-body text-sm text-ink-muted hover:text-ink-body hover:bg-raised transition-colors"
        >
          New comparison
        </button>
      </div>

      <WinnerHero ranked={ranked} concept={concept} />
      <TradeoffsSection ranked={ranked} />
      <CandidateTable ranked={ranked} />
      <DetailedEvidence ranked={ranked} />
    </div>
  );
}

// ── Main component ────────────────────────────────────────────────────────────

type Phase = "setup" | "running" | "results";

export function ComparativeIntelligence() {
  const [phase, setPhase] = useState<Phase>("setup");
  const [concept, setConcept] = useState<BusinessConcept | null>(null);
  const [selectedIds, setSelectedIds] = useState<Set<string>>(new Set());
  const [completedIds, setCompletedIds] = useState<Set<string>>(new Set());
  const [ranked, setRanked] = useState<RankedResult[]>([]);

  const {
    data: assets = [],
    isLoading: assetsLoading,
    isError: assetsError,
  } = useQuery({
    queryKey: queryKeys.assets,
    queryFn: getAssets,
    staleTime: 5 * 60 * 1000,
  });

  const toggleId = useCallback((id: string) => {
    setSelectedIds((prev) => {
      const next = new Set(prev);
      if (next.has(id)) {
        next.delete(id);
      } else if (next.size < MAX_ASSETS) {
        next.add(id);
      }
      return next;
    });
  }, []);

  const handleRun = useCallback(async () => {
    if (!concept || selectedIds.size < MIN_ASSETS) return;
    const selectedAssets = assets.filter((a) => selectedIds.has(a.id));
    setPhase("running");
    setCompletedIds(new Set());

    const accumulated: Record<string, ConceptAssessment> = {};

    await Promise.allSettled(
      selectedAssets.map(async (asset) => {
        try {
          const result = await runConceptAssessment(asset, concept, {
            domain: concept,
            ...DEFAULT_PARAMS,
          });
          accumulated[asset.id] = result;
          setCompletedIds((prev) => new Set([...prev, asset.id]));
        } catch {
          // Asset failed to score — excluded from comparison
        }
      })
    );

    const finalRanked = rankResults(accumulated, selectedAssets);
    setRanked(finalRanked);
    setPhase("results");
  }, [assets, concept, selectedIds]);

  const handleReset = useCallback(() => {
    setPhase("setup");
    setConcept(null);
    setSelectedIds(new Set());
    setCompletedIds(new Set());
    setRanked([]);
  }, []);

  const runningAssets = assets.filter((a) => selectedIds.has(a.id));

  return (
    <main className="min-h-full bg-page text-ink-body">
      {phase === "setup" && (
        <SetupView
          assets={assets}
          assetsLoading={assetsLoading}
          assetsError={assetsError}
          concept={concept}
          setConcept={setConcept}
          selectedIds={selectedIds}
          toggleId={toggleId}
          onRun={handleRun}
        />
      )}

      {phase === "running" && (
        <RunningView assets={runningAssets} completedIds={completedIds} />
      )}

      {phase === "results" && ranked.length > 0 && concept && (
        <ResultsView ranked={ranked} concept={concept} onReset={handleReset} />
      )}
    </main>
  );
}
