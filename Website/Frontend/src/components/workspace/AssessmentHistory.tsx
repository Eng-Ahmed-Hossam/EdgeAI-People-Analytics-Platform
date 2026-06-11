"use client";

import { useState, useMemo } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  listSavedAssessments,
  getAssets,
  getValuation,
  getTemporalIntelligence,
  queryKeys,
} from "@/lib/api";
import { cn, bandColors, bandLabel } from "@/lib/utils";
import type { SavedAssessmentRecord } from "@/types/saved-assessment";
import type { Asset } from "@/types/asset";
import type { BusinessConcept, ConceptAssessment } from "@/types/assessment-v2";
import type { BusinessDomain } from "@/types/parameters";

// ── Concept labels (shared with WorkspaceSidePanel) ───────────────────────────

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

function formatDate(iso: string): string {
  return new Date(iso).toLocaleDateString("en-GB", {
    day: "numeric",
    month: "short",
    year: "numeric",
  });
}

// ── Single saved-assessment row ───────────────────────────────────────────────

function SavedAssessmentRow({
  record,
  asset,
  isReopening,
  onReopen,
}: {
  record: SavedAssessmentRecord;
  asset: Asset | undefined;
  isReopening: boolean;
  onReopen: (record: SavedAssessmentRecord) => void;
}) {
  const { feasibility } = record;
  const colors = bandColors(feasibility.scoreBand);
  const ci = feasibility.confidenceInterval;

  return (
    <div className={cn(
      "border border-hairline rounded-(--r-md) bg-surface transition-colors",
      "hover:border-border-strong"
    )}>
      <div className="px-4 pt-3.5 pb-3 flex items-start gap-4">
        {/* Score — dominant, tabular mono */}
        <div className="shrink-0 w-12 text-right mt-0.5">
          <span className={cn(
            "font-mono text-2xl font-semibold leading-none tabular-nums",
            colors.text
          )}>
            {feasibility.feasibilityScore}
          </span>
          <p className="font-mono text-[10px] text-ink-faint mt-0.5">/ 100</p>
        </div>

        {/* Middle — concept, asset, meta */}
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2 mb-0.5">
            <span className={cn(
              "inline-flex items-center gap-1 px-1.5 py-0.5 rounded-(--r-sm) border",
              "font-body text-[10px] uppercase tracking-wider font-medium",
              colors.bg, colors.border, colors.text
            )}>
              {bandLabel(feasibility.scoreBand)}
            </span>
            <span className="font-body text-sm font-medium text-ink-strong truncate">
              {CONCEPT_LABELS[record.concept] ?? record.concept}
            </span>
          </div>

          {asset ? (
            <p className="font-body text-xs text-ink-muted truncate">
              {asset.address}
              <span className="text-ink-faint"> · {asset.area}</span>
            </p>
          ) : (
            <p className="font-body text-xs text-ink-faint font-mono truncate">
              {record.assetId}
            </p>
          )}

          <div className="mt-1.5 flex items-center gap-3 flex-wrap">
            <span className="font-mono text-[10px] text-ink-faint">
              CI{" "}
              <span className="text-ink-muted">{ci.low}–{ci.high}</span>
            </span>
            <span className="font-mono text-[10px] text-ink-faint">
              {record.engineVersion}
            </span>
            <span className="font-mono text-[10px] text-ink-faint">
              {formatDate(record.createdAt)}
            </span>
          </div>
        </div>

        {/* Reopen CTA */}
        <div className="shrink-0 self-center">
          <button
            type="button"
            onClick={() => onReopen(record)}
            disabled={isReopening}
            className={cn(
              "px-3 py-1.5 rounded-(--r-sm) border border-hairline",
              "font-body text-xs font-medium text-ink-body",
              "transition-colors hover:border-accent hover:text-accent",
              "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent",
              "disabled:opacity-50 disabled:cursor-not-allowed"
            )}
          >
            {isReopening ? (
              <span className="flex items-center gap-1.5">
                <span className="w-3 h-3 rounded-full border border-transparent border-t-accent animate-spin" aria-hidden="true" />
                Loading…
              </span>
            ) : (
              "Reopen"
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

// ── Assessment History panel ──────────────────────────────────────────────────

export interface AssessmentHistoryProps {
  onClose: () => void;
  onReopen: (asset: Asset, assessment: ConceptAssessment, persistenceId: string) => void;
}

export function AssessmentHistory({ onClose, onReopen }: AssessmentHistoryProps) {
  const [reopeningId, setReopeningId] = useState<string | null>(null);
  const [reopenError, setReopenError] = useState<string | null>(null);

  const {
    data: savedRecords = [],
    isLoading,
    isError,
  } = useQuery({
    queryKey: queryKeys.savedAssessments,
    queryFn: () => listSavedAssessments(),
    staleTime: 30 * 1000,
  });

  const { data: assets = [] } = useQuery({
    queryKey: queryKeys.assets,
    queryFn: getAssets,
    staleTime: 5 * 60 * 1000,
  });

  const assetsById = useMemo(() => {
    const map: Record<string, Asset> = {};
    assets.forEach((a) => { map[a.id] = a; });
    return map;
  }, [assets]);

  async function handleReopen(record: SavedAssessmentRecord) {
    setReopeningId(record.assessmentId);
    setReopenError(null);
    try {
      const asset = assetsById[record.assetId];
      if (!asset) {
        throw new Error(`Asset not found in list: ${record.assetId}. The asset may have been removed.`);
      }

      // Re-fetch valuation and temporal for the asset.
      // These are live state; only feasibility is restored from the stored record.
      const [valuation, temporal] = await Promise.all([
        getValuation(record.assetId),
        getTemporalIntelligence(record.assetId),
      ]);

      const conceptAssessment: ConceptAssessment = {
        assetId: record.assetId,
        concept: record.concept as BusinessConcept,
        parameters: {
          domain: record.concept as BusinessDomain,
          operatingHours: { open: "09:00", close: "22:00", daysPerWeek: 6 },
          staffCount: 5,
          budget: 500000,
          currency: "EGP",
        },
        generatedAt: record.createdAt,
        feasibility: record.feasibility, // from stored record — the round-trip guarantee
        valuation,
        temporal,
      };

      onReopen(asset, conceptAssessment, record.assessmentId);
    } catch (err) {
      setReopenError(
        err instanceof Error
          ? err.message
          : "Reopen failed. Please try again."
      );
    } finally {
      setReopeningId(null);
    }
  }

  return (
    <aside
      className={cn(
        // Mobile
        "absolute inset-x-3 bottom-3 z-20 max-h-[74vh] overflow-hidden",
        "rounded-(--r-lg) border border-hairline bg-surface shadow-(--shadow-pop)",
        // Desktop
        "lg:static lg:inset-auto lg:flex lg:h-full lg:max-h-none",
        "lg:w-105 lg:shrink-0 lg:flex-col",
        "lg:rounded-none lg:border-y-0 lg:border-r-0 lg:border-l lg:shadow-none"
      )}
      aria-label="Saved portfolio"
    >
      {/* Header */}
      <div className="shrink-0 border-b border-hairline px-5 py-4 flex items-center justify-between gap-3">
        <div>
          <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-0.5">
            Portfolio
          </p>
          <h2 className="font-display text-base font-semibold text-ink-strong leading-tight">
            Saved Assessments
          </h2>
        </div>

        <button
          type="button"
          onClick={onClose}
          aria-label="Close portfolio and return to workspace"
          className="shrink-0 w-7 h-7 flex items-center justify-center rounded-(--r-sm) text-ink-faint hover:text-ink-muted hover:bg-raised transition-colors"
        >
          {/* Back arrow */}
          <svg
            width="12" height="12" viewBox="0 0 12 12"
            fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"
            aria-hidden="true"
          >
            <polyline points="7.5 1.5 3 6 7.5 10.5" />
          </svg>
        </button>
      </div>

      {/* Body — scrollable list */}
      <div className="flex-1 overflow-y-auto">
        {/* Loading */}
        {isLoading && (
          <div className="px-5 pt-5 space-y-3">
            {[0, 1, 2].map((i) => (
              <div key={i} className="h-20 animate-pulse rounded-(--r-md) bg-raised" />
            ))}
          </div>
        )}

        {/* Error */}
        {isError && !isLoading && (
          <div className="px-5 pt-5">
            <div className="rounded-(--r-md) border border-verdict-weak/40 bg-verdict-weak-wash px-4 py-3">
              <p className="font-body text-sm text-ink-body">
                Could not load saved assessments. Check that the backend is running.
              </p>
            </div>
          </div>
        )}

        {/* Empty */}
        {!isLoading && !isError && savedRecords.length === 0 && (
          <div className="flex flex-col items-center justify-center h-full px-8 py-12 text-center">
            <div className="w-10 h-10 rounded-(--r-md) bg-sunken border border-hairline flex items-center justify-center mb-4">
              <svg
                width="18" height="18" viewBox="0 0 18 18"
                fill="none" stroke="#64748B" strokeWidth="1.25" strokeLinecap="round"
                aria-hidden="true"
              >
                <path d="M4 3h10a1 1 0 011 1v11.5l-5-2.5L5 15.5V4a1 1 0 011-1z" />
              </svg>
            </div>
            <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint mb-2">
              No saved assessments
            </p>
            <p className="font-body text-sm text-ink-muted leading-relaxed">
              Run an assessment, review the result, then save it to build your portfolio.
            </p>
            <button
              type="button"
              onClick={onClose}
              className="mt-5 font-body text-xs text-accent-muted hover:text-accent transition-colors"
            >
              ← Back to workspace
            </button>
          </div>
        )}

        {/* Reopen error */}
        {reopenError && (
          <div className="px-5 pt-4">
            <div className="rounded-(--r-md) border border-verdict-weak/40 bg-verdict-weak-wash px-4 py-3">
              <p className="font-body text-xs text-ink-body">{reopenError}</p>
            </div>
          </div>
        )}

        {/* Records list */}
        {!isLoading && !isError && savedRecords.length > 0 && (
          <div className="px-4 py-4 space-y-2.5">
            <p className="font-body text-[10px] font-medium uppercase tracking-widest text-ink-faint px-1 mb-3">
              {savedRecords.length}&nbsp;saved · newest first
            </p>

            {savedRecords.map((record) => (
              <SavedAssessmentRow
                key={record.assessmentId}
                record={record}
                asset={assetsById[record.assetId]}
                isReopening={reopeningId === record.assessmentId}
                onReopen={handleReopen}
              />
            ))}
          </div>
        )}
      </div>
    </aside>
  );
}
