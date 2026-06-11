// ─────────────────────────────────────────────────────────────────────────────
// Assessment Store — EdgeAI Location Intelligence (SYSTEM_DESIGN.md §5.1)
//
// This store is the single source of truth for assessment state. No screen may
// hold a private copy of these values.
//
//   selectedAsset, allConceptAssessments, activeConceptId
//   Transitions: selectAsset, addConceptAssessment, setActiveConcept
//
// The transitions implement the one-to-many contract:
//   - selectAsset  does NOT cascade-clear allConceptAssessments
//   - addConceptAssessment  adds/updates ONE concept's entry; never touches others
//   - setActiveConcept  is a pure pointer; no refetch, no clear, no computation
// ─────────────────────────────────────────────────────────────────────────────

import { create } from "zustand";
import { devtools } from "zustand/middleware";

// New types
import type { Asset } from "@/types/asset";
import type {
  BusinessConcept,
  ConceptAssessment,
  ConceptAssessmentRecord,
  AssetConceptAssessmentMap,
} from "@/types/assessment-v2";

// ── State interface ────────────────────────────────────────────────────────────

interface AssessmentStoreState {
  selectedAsset: Asset | null;

  // All concept-assessments in this session, keyed assetId → concept → result.
  // Persists across asset switches — switching to a different asset and back
  // preserves all previously accumulated assessments for each asset.
  allConceptAssessments: AssetConceptAssessmentMap;

  // Which concept's result is currently displayed for the selected asset.
  // Null means no concept has been run yet (or the user hasn't chosen one).
  activeConceptId: BusinessConcept | null;

  // ── Saved-state tracking (Session J — additive; no existing transitions changed) ──
  //
  // Tracks which (assetId, concept) pairs have been saved to the backend and
  // what their persistence IDs are. Keyed assetId → concept → persistenceId.
  // undefined for a concept means unsaved. This is in-session memory only —
  // after refresh the store resets, but history is re-fetched from the backend.
  savedAssessmentIds: Record<string, Partial<Record<BusinessConcept, string>>>;

  // ── Transitions ─────────────────────────────────────────────────────────

  /**
   * Set the selected asset. Does NOT clear allConceptAssessments — if this
   * asset was assessed earlier in the session, those assessments are preserved.
   * Resets activeConceptId to null (no concept is active for the new selection).
   */
  selectAsset: (asset: Asset) => void;

  /**
   * Add (or update) a completed concept-assessment in the collection for the
   * assessment's asset. Never touches other concepts' entries. Sets this
   * concept as the active one.
   *
   * Implements the one-to-many accumulation — calling this for coffee_shop,
   * then pharmacy, then restaurant leaves all three in the collection.
   */
  addConceptAssessment: (assessment: ConceptAssessment) => void;

  /**
   * Pure pointer change — sets which existing assessment is displayed.
   * Does NOT refetch, recompute, or clear anything. Switching concepts is
   * selection, not computation.
   */
  setActiveConcept: (concept: BusinessConcept) => void;

  /**
   * Mark a concept-assessment as saved. Called after a successful
   * POST /api/assessments. Records the persistence ID for that (assetId, concept).
   * Per-concept grain — marking coffee saved does not touch pharmacy or any other.
   */
  markAssessmentSaved: (
    assetId: string,
    concept: BusinessConcept,
    persistenceId: string
  ) => void;

  /** Clears all state. */
  reset: () => void;
}

// ── Initial state ─────────────────────────────────────────────────────────────

const initialState = {
  selectedAsset: null,
  allConceptAssessments: {} as AssetConceptAssessmentMap,
  activeConceptId: null,
  savedAssessmentIds: {} as Record<string, Partial<Record<BusinessConcept, string>>>,
} satisfies Pick<
  AssessmentStoreState,
  "selectedAsset" | "allConceptAssessments" | "activeConceptId" | "savedAssessmentIds"
>;

// ── Store ─────────────────────────────────────────────────────────────────────

export const useAssessmentStore = create<AssessmentStoreState>()(
  devtools(
    (set) => ({
      ...initialState,

      selectAsset: (asset) =>
        set(
          {
            selectedAsset: asset,
            activeConceptId: null,
            // allConceptAssessments is deliberately NOT touched here.
            // The asset's prior assessments from earlier in the session persist.
          },
          false,
          "selectAsset"
        ),

      addConceptAssessment: (assessment) =>
        set(
          (state) => {
            const { assetId, concept } = assessment;
            // Spread the existing per-asset record to preserve all other concepts
            const priorForAsset = state.allConceptAssessments[assetId] ?? {};
            return {
              allConceptAssessments: {
                ...state.allConceptAssessments,
                [assetId]: {
                  ...priorForAsset,
                  [concept]: assessment, // add/update this concept only
                },
              },
              activeConceptId: concept, // make this the active view
            };
          },
          false,
          "addConceptAssessment"
        ),

      setActiveConcept: (concept) =>
        // Pure pointer — no side effects
        set({ activeConceptId: concept }, false, "setActiveConcept"),

      markAssessmentSaved: (assetId, concept, persistenceId) =>
        set(
          (state) => {
            const prior = state.savedAssessmentIds[assetId] ?? {};
            return {
              savedAssessmentIds: {
                ...state.savedAssessmentIds,
                [assetId]: { ...prior, [concept]: persistenceId },
              },
            };
          },
          false,
          "markAssessmentSaved"
        ),

      reset: () => set(initialState, false, "reset"),
    }),
    { name: "EdgeAI / Assessment" }
  )
);

// ── Derived selectors ──────────────────────────────────────────────────────────

// Stable empty record returned when no assessments exist.
// MUST be a module-level constant — returning `{}` inline creates a new object
// reference on every call, causing Zustand's useSyncExternalStore to detect an
// infinite "snapshot changed" loop. Object.is({}, {}) === false.
const EMPTY_ASSESSMENT_RECORD: ConceptAssessmentRecord = {};

/**
 * All concept-assessments for the currently selected asset.
 * Returns a stable empty record when no asset is selected or no concepts have been run.
 */
export function selectCurrentAssetAssessments(
  state: AssessmentStoreState
): ConceptAssessmentRecord {
  if (!state.selectedAsset) return EMPTY_ASSESSMENT_RECORD;
  return state.allConceptAssessments[state.selectedAsset.id] ?? EMPTY_ASSESSMENT_RECORD;
}

/**
 * The currently active concept-assessment (the one activeConceptId points to).
 * Returns null when no asset is selected, no concept is active, or the active
 * concept hasn't been run yet.
 */
export function selectActiveConceptAssessment(
  state: AssessmentStoreState
): ConceptAssessment | null {
  if (!state.selectedAsset || !state.activeConceptId) return null;
  return (
    state.allConceptAssessments[state.selectedAsset.id]?.[state.activeConceptId] ??
    null
  );
}

/**
 * How many concept-assessments exist for the currently selected asset.
 */
export function selectConceptAssessmentCount(
  state: AssessmentStoreState
): number {
  if (!state.selectedAsset) return 0;
  const record = state.allConceptAssessments[state.selectedAsset.id];
  if (!record) return 0;
  return Object.keys(record).length;
}

