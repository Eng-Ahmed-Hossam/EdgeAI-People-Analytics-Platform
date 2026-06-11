// ─────────────────────────────────────────────────────────────────────────────
// SavedAssessmentRecord — backend DTO for a persisted ConceptAssessment.
//
// Mirrors backend StoredAssessmentResponse (schemas/assessment.py).
// Only the feasibility face is stored server-side; valuation and temporal are
// re-fetched live on reopen because they reflect the current state of the asset,
// not the moment of assessment.
//
// Round-trip contract: feasibility is reconstructed from feasibility_json on the
// backend, so every field (score, CI, factor order, neutral factors) is
// byte-for-byte identical to the original engine output.
// ─────────────────────────────────────────────────────────────────────────────

import type { BusinessConcept, FeasibilityOutput } from "./assessment-v2";

export interface SavedAssessmentRecord {
  assessmentId: string;
  assetId: string;
  ownerId: string | null;
  concept: BusinessConcept;
  engineVersion: string;
  coverageScore: number;
  createdAt: string; // ISO 8601 — when persisted to the backend
  feasibility: FeasibilityOutput;
}
