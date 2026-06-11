"use client";

import { useCallback, useMemo, useState } from "react";
import dynamic from "next/dynamic";
import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";

// maplibre-gl must not run during SSR — dynamic import with ssr:false is the
// belt-and-suspenders companion to serverExternalPackages in next.config.ts.
const DecisionMap = dynamic(
  () => import("@/components/workspace/DecisionMap").then((m) => ({ default: m.DecisionMap })),
  { ssr: false, loading: () => <div className="h-full w-full bg-sunken" /> }
);
import { WorkspaceSidePanel } from "@/components/workspace/WorkspaceSidePanel";
import { AssessmentHistory } from "@/components/workspace/AssessmentHistory";
import {
  getAssets,
  getTemporalIntelligence,
  runConceptAssessment,
  saveAssessment,
  listSavedAssessments,
  queryKeys,
} from "@/lib/api";
import {
  useAssessmentStore,
  selectCurrentAssetAssessments,
  selectActiveConceptAssessment,
} from "@/store/assessmentStore";
import type { Asset } from "@/types/asset";
import type { BusinessConcept, ConceptAssessment } from "@/types/assessment-v2";
import type { BusinessParameters } from "@/types/parameters";
import { cn } from "@/lib/utils";

// ── Helpers ───────────────────────────────────────────────────────────────────

function coverageTone(asset: Asset): string {
  if (asset.availabilityStatus !== "available") return "text-ink-faint";
  if (asset.coverageScore >= 70) return "text-status-high";
  if (asset.coverageScore >= 45) return "text-status-med";
  return "text-status-low";
}

function availabilityLabel(asset: Asset): string {
  if (asset.availabilityStatus === "available") return "Available";
  if (asset.availabilityStatus === "reserved") return "Reserved";
  if (asset.availabilityStatus === "occupied") return "Occupied";
  return "Sold";
}

// ── Asset search surface ───────────────────────────────────────────────────────
// Floating panel over the top-left of the map. Allows keyword search
// and shows filtered asset list for selection.

function AssetSearchSurface({
  assets,
  selectedAsset,
  onSelectAsset,
  isLoading,
  isError,
  assessedAssetIds,
  savedCount,
  onShowHistory,
}: {
  assets: Asset[];
  selectedAsset: Asset | null;
  onSelectAsset: (asset: Asset) => void;
  isLoading: boolean;
  isError: boolean;
  assessedAssetIds: Set<string>;
  savedCount: number;
  onShowHistory: () => void;
}) {
  const [query, setQuery] = useState("");

  const filtered = useMemo(() => {
    const q = query.trim().toLowerCase();
    if (!q) return assets;
    return assets.filter(
      (a) =>
        a.address.toLowerCase().includes(q) ||
        a.area.toLowerCase().includes(q)
    );
  }, [assets, query]);

  const visible = filtered.slice(0, 6);

  return (
    <div className="absolute left-3 right-3 top-3 z-10 max-w-130 rounded-(--r-lg) border border-hairline bg-surface/95 shadow-(--shadow-pop) backdrop-blur-sm">
      {/* Search header */}
      <div className="border-b border-hairline px-4 pt-4 pb-3">
        <div className="flex items-center justify-between gap-3 mb-3">
          <div>
            <p className="font-body text-xs font-medium uppercase tracking-widest text-ink-faint">
              Site Selection Workbench
            </p>
            <h1 className="mt-0.5 font-display text-base text-ink-strong leading-tight">
              EdgeAI Location Intelligence
            </h1>
          </div>

          {/* Portfolio toggle — always accessible */}
          <button
            type="button"
            onClick={onShowHistory}
            className={cn(
              "shrink-0 flex items-center gap-1.5 px-2.5 py-1.5 rounded-(--r-sm)",
              "border border-hairline bg-sunken",
              "font-body text-xs text-ink-muted",
              "transition-colors hover:border-accent hover:text-accent",
              "focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-accent"
            )}
            title="View saved portfolio"
          >
            {/* Bookmark icon */}
            <svg
              width="9" height="11" viewBox="0 0 9 11"
              fill={savedCount > 0 ? "currentColor" : "none"}
              stroke="currentColor"
              strokeWidth="1.25"
              className="shrink-0"
              aria-hidden="true"
            >
              <path d="M1 1h7a.5.5 0 01.5.5v9l-4-2-4 2V1.5A.5.5 0 011 1z" />
            </svg>
            Portfolio
            {savedCount > 0 && (
              <span className="font-mono text-[10px] text-accent-muted">
                {savedCount}
              </span>
            )}
          </button>
        </div>

        <label>
          <span className="sr-only">Search assets by address or district</span>
          <input
            className="h-9 w-full rounded-(--r-md) border border-hairline bg-sunken px-3 font-body text-sm text-ink-strong outline-none transition-colors placeholder:text-ink-faint focus:border-accent"
            placeholder="Search address, district, or corridor…"
            type="search"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
          />
        </label>
      </div>

      {/* Asset list */}
      <div className="max-h-72 overflow-y-auto p-2">
        {isError && (
          <p className="rounded-(--r-md) border border-verdict-weak/40 bg-verdict-weak-wash px-3 py-2.5 font-body text-sm text-ink-body">
            Assets could not be loaded. Check the API seam.
          </p>
        )}

        {isLoading && !isError && (
          <div className="space-y-1.5">
            {[0, 1, 2].map((i) => (
              <div key={i} className="h-14 animate-pulse rounded-(--r-md) bg-sunken" />
            ))}
          </div>
        )}

        {!isLoading && !isError && visible.length === 0 && (
          <div className="rounded-(--r-md) border border-hairline bg-sunken px-3 py-4">
            <p className="font-body text-sm text-ink-strong">No assets matched this search.</p>
            <p className="mt-0.5 font-body text-xs text-ink-muted">
              Try another district, street, or corridor.
            </p>
          </div>
        )}

        {!isLoading && !isError && visible.length > 0 && (
          <div className="space-y-1">
            {visible.map((asset) => {
              const isSelected = selectedAsset?.id === asset.id;
              const isAssessed = assessedAssetIds.has(asset.id);
              return (
                <button
                  key={asset.id}
                  type="button"
                  onClick={() => onSelectAsset(asset)}
                  className={cn(
                    "w-full rounded-(--r-md) border px-3 py-2.5 text-left transition-colors",
                    isSelected
                      ? "border-accent bg-accent-soft"
                      : "border-transparent bg-transparent hover:border-hairline hover:bg-sunken"
                  )}
                >
                  <div className="flex items-start justify-between gap-3">
                    <div className="min-w-0">
                      <p className="truncate font-body text-sm font-medium text-ink-strong">
                        {asset.address}
                      </p>
                      <p className="mt-0.5 font-body text-xs text-ink-faint">
                        {asset.area} · {availabilityLabel(asset)}
                        {isAssessed && (
                          <span className="ml-2 text-accent-muted">assessed</span>
                        )}
                      </p>
                    </div>
                    <div className="shrink-0 text-right">
                      <span className={cn("font-mono text-sm font-semibold", coverageTone(asset))}>
                        {asset.coverageScore}
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
    </div>
  );
}

// ── Map legend ────────────────────────────────────────────────────────────────

function MapLegend() {
  return (
    <div className="absolute bottom-4 left-4 z-10 hidden rounded-(--r-md) border border-hairline bg-surface/95 px-3 py-2 shadow-(--shadow-card) backdrop-blur-sm md:block">
      <div className="flex items-center gap-4">
        {[
          { color: "bg-status-high", label: "High coverage" },
          { color: "bg-status-med",  label: "Moderate" },
          { color: "bg-status-low",  label: "Lower" },
          { color: "bg-ink-faint",   label: "Unavailable" },
        ].map(({ color, label }) => (
          <div key={label} className="flex items-center gap-1.5">
            <span className={cn("h-2.5 w-2.5 rounded-full", color)} />
            <span className="font-body text-xs text-ink-muted">{label}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

// ── Selected asset badge ───────────────────────────────────────────────────────

function SelectedAssetBadge({ asset }: { asset: Asset | null }) {
  if (!asset) return null;
  return (
    <div className="absolute bottom-16 left-3 z-10 hidden max-w-95 rounded-(--r-md) border border-hairline bg-surface/95 px-4 py-3 shadow-(--shadow-card) backdrop-blur-sm md:block">
      <p className="font-body text-[0.6rem] uppercase tracking-widest text-ink-faint">
        Evaluating
      </p>
      <div className="mt-0.5 flex items-center justify-between gap-3">
        <p className="truncate font-body text-sm font-medium text-ink-strong">
          {asset.address}
        </p>
        <span className={cn("font-mono text-sm font-semibold shrink-0", coverageTone(asset))}>
          {asset.coverageScore}
          <span className="text-ink-faint text-xs font-normal ml-0.5">cov</span>
        </span>
      </div>
    </div>
  );
}

// ── Main workbench orchestrator ───────────────────────────────────────────────

export function WorkspaceWorkbench() {
  // ── New store slices (no deprecated fields accessed here) ─────────────────
  const selectedAsset        = useAssessmentStore((s) => s.selectedAsset);
  const activeConceptId      = useAssessmentStore((s) => s.activeConceptId);
  const allConceptAssessments = useAssessmentStore((s) => s.allConceptAssessments);
  const savedAssessmentIds   = useAssessmentStore((s) => s.savedAssessmentIds);
  const selectAsset          = useAssessmentStore((s) => s.selectAsset);
  const addConceptAssessment  = useAssessmentStore((s) => s.addConceptAssessment);
  const setActiveConcept     = useAssessmentStore((s) => s.setActiveConcept);
  const markAssessmentSaved  = useAssessmentStore((s) => s.markAssessmentSaved);
  const reset                = useAssessmentStore((s) => s.reset);

  const currentAssessments = useAssessmentStore(selectCurrentAssetAssessments);
  const activeAssessment   = useAssessmentStore(selectActiveConceptAssessment);

  // Per-concept saved state for the currently selected asset
  const savedConceptIds = useMemo(
    () => (selectedAsset ? (savedAssessmentIds[selectedAsset.id] ?? {}) : {}),
    [savedAssessmentIds, selectedAsset]
  );

  // ── History panel state ────────────────────────────────────────────────────
  const [showHistory, setShowHistory] = useState(false);
  // Counter used as part of the panel key so it remounts after reopen,
  // ensuring showForm resets to false even when the same asset is re-selected.
  const [panelResetKey, setPanelResetKey] = useState(0);

  // ── Query client (for cache invalidation after save) ──────────────────────
  const queryClient = useQueryClient();

  // ── Asset list (map markers + search surface) ─────────────────────────────
  const {
    data: assets = [],
    isLoading: assetsLoading,
    isError: assetsError,
  } = useQuery({
    queryKey: queryKeys.assets,
    queryFn: getAssets,
    staleTime: 5 * 60 * 1000,
  });

  // ── Asset temporal intelligence (pre-concept coverage brief) ───────────────
  const { data: assetTemporal = null } = useQuery({
    queryKey: queryKeys.temporal(selectedAsset?.id ?? ""),
    queryFn: () => getTemporalIntelligence(selectedAsset!.id),
    enabled: !!selectedAsset,
    staleTime: 10 * 60 * 1000,
  });

  // ── Saved assessments list (shared cache with AssessmentHistory) ───────────
  // Fetched in the background so the Portfolio count is always current without
  // the user having to open the panel first.
  const { data: savedAssessmentsList = [] } = useQuery({
    queryKey: queryKeys.savedAssessments,
    queryFn: () => listSavedAssessments(),
    staleTime: 30 * 1000,
  });

  const savedCount = savedAssessmentsList.length;

  // Build the set of assessed asset IDs for map + search surface styling
  const assessedAssetIds = useMemo(
    () => new Set(Object.keys(allConceptAssessments)),
    [allConceptAssessments]
  );

  // Build the record of assetId → highest feasibility score for marker labels
  const assessedScores = useMemo(() => {
    const out: Record<string, number> = {};
    for (const [assetId, byConceptRecord] of Object.entries(allConceptAssessments)) {
      const scores = Object.values(byConceptRecord).map(
        (a) => a.feasibility.feasibilityScore
      );
      if (scores.length > 0) {
        out[assetId] = Math.max(...scores);
      }
    }
    return out;
  }, [allConceptAssessments]);

  // ── Assessment mutation ────────────────────────────────────────────────────
  const assessmentMutation = useMutation({
    mutationFn: ({
      asset,
      concept,
      params,
    }: {
      asset: Asset;
      concept: BusinessConcept;
      params: BusinessParameters;
    }) => runConceptAssessment(asset, concept, params),

    onSuccess: (result) => {
      const currentAsset = useAssessmentStore.getState().selectedAsset;
      if (currentAsset?.id === result.assetId) {
        addConceptAssessment(result);
      }
    },
  });

  // ── Save mutation ──────────────────────────────────────────────────────────
  // Per-concept grain: saves only the active concept's feasibility output.
  // On success, marks that concept as saved in the store and invalidates the
  // savedAssessments cache so the Portfolio count and history list refresh.
  const saveMutation = useMutation({
    mutationFn: ({
      assetId,
      concept,
      assessment,
    }: {
      assetId: string;
      concept: BusinessConcept;
      assessment: ConceptAssessment;
    }) => saveAssessment(assetId, concept, assessment.feasibility),

    onSuccess: (saved) => {
      markAssessmentSaved(saved.assetId, saved.concept, saved.assessmentId);
      // Refresh the portfolio count and history list
      void queryClient.invalidateQueries({ queryKey: queryKeys.savedAssessments });
    },
  });

  // ── Stable callbacks ─────────────────────────────────────────────────────

  const handleSelectAsset = useCallback(
    (asset: Asset) => {
      if (asset.id !== selectedAsset?.id) {
        assessmentMutation.reset();
        saveMutation.reset();
      }
      selectAsset(asset);
    },
    [assessmentMutation, saveMutation, selectAsset, selectedAsset]
  );

  const handleSubmitConcept = useCallback(
    (concept: BusinessConcept, params: BusinessParameters) => {
      if (!selectedAsset) return;
      assessmentMutation.mutate({ asset: selectedAsset, concept, params });
    },
    [assessmentMutation, selectedAsset]
  );

  const handleSwitchConcept = useCallback(
    (concept: BusinessConcept) => {
      setActiveConcept(concept);
    },
    [setActiveConcept]
  );

  const handleSaveAssessment = useCallback(
    (concept: BusinessConcept) => {
      if (!selectedAsset) return;
      const assessment = allConceptAssessments[selectedAsset.id]?.[concept];
      if (!assessment) return;
      saveMutation.mutate({ assetId: selectedAsset.id, concept, assessment });
    },
    [selectedAsset, allConceptAssessments, saveMutation]
  );

  const handleReset = useCallback(() => {
    assessmentMutation.reset();
    saveMutation.reset();
    reset();
  }, [assessmentMutation, saveMutation, reset]);

  // ── Reopen a saved assessment ─────────────────────────────────────────────
  // Called by AssessmentHistory when user clicks "Reopen".
  // Loads the asset + reconstructed ConceptAssessment into the workspace store,
  // marks it as saved (so VerdictPanel shows the saved state immediately),
  // increments panelResetKey to force WorkspaceSidePanel to remount
  // (clearing any in-progress form state), then closes the history panel.
  const handleReopen = useCallback(
    (asset: Asset, assessment: ConceptAssessment, persistenceId: string) => {
      assessmentMutation.reset();
      saveMutation.reset();
      selectAsset(asset);
      addConceptAssessment(assessment);
      markAssessmentSaved(asset.id, assessment.concept as BusinessConcept, persistenceId);
      setPanelResetKey((k) => k + 1);
      setShowHistory(false);
    },
    [assessmentMutation, saveMutation, selectAsset, addConceptAssessment, markAssessmentSaved]
  );

  return (
    <div className="relative flex h-full min-h-180 overflow-hidden bg-page text-ink-body lg:min-h-0">
      {/* Map area — fills remaining width */}
      <section className="relative min-w-0 flex-1 basis-[74%]">
        <DecisionMap
          assets={assets}
          selectedAsset={selectedAsset}
          onSelectAsset={handleSelectAsset}
          assessedScores={assessedScores}
        />

        <AssetSearchSurface
          assets={assets}
          selectedAsset={selectedAsset}
          onSelectAsset={handleSelectAsset}
          isLoading={assetsLoading}
          isError={assetsError}
          assessedAssetIds={assessedAssetIds}
          savedCount={savedCount}
          onShowHistory={() => setShowHistory(true)}
        />

        <SelectedAssetBadge asset={selectedAsset} />
        <MapLegend />
      </section>

      {/* Right panel — toggles between workspace panel and portfolio history */}
      {showHistory ? (
        <AssessmentHistory
          onClose={() => setShowHistory(false)}
          onReopen={handleReopen}
        />
      ) : (
        <WorkspaceSidePanel
          key={`${selectedAsset?.id ?? "idle"}-${panelResetKey}`}
          selectedAsset={selectedAsset}
          currentAssessments={currentAssessments}
          activeConceptId={activeConceptId}
          activeAssessment={activeAssessment}
          analyzingConcept={assessmentMutation.variables?.concept ?? null}
          isAnalyzing={assessmentMutation.isPending}
          assetTemporal={assetTemporal}
          savedConceptIds={savedConceptIds}
          onSaveAssessment={handleSaveAssessment}
          isSaving={saveMutation.isPending}
          saveError={
            saveMutation.isError
              ? "Save failed. The backend may be unreachable — check the API and try again."
              : null
          }
          analysisError={
            assessmentMutation.isError
              ? "The assessment did not complete. The selected asset and prior concepts are preserved — retry the assessment."
              : null
          }
          onSubmitConcept={handleSubmitConcept}
          onSwitchConcept={handleSwitchConcept}
          onReset={handleReset}
        />
      )}
    </div>
  );
}
