// ─────────────────────────────────────────────────────────────────────────────
// [SUPERSEDED] Site — the prior central object, replaced by Asset
//
// Asset (src/types/asset.ts) is now the central domain object per
// SYSTEM_DESIGN.md §2.1. Site may persist as an internal or future concept
// (e.g. a geographic point in scoring computations or sensor placement) but
// the product experience is asset-first. Users discover Assets, not Sites.
//
// These types are kept for UI compatibility during migration. Once the workspace
// and all screens are updated to use Asset, these can be removed.
// ─────────────────────────────────────────────────────────────────────────────

export type SiteStatus = "scored" | "pending" | "insufficient";

export interface SiteCoordinates {
  lat: number;
  lng: number;
}

export interface Site {
  id: string;
  name: string;
  coordinates: SiteCoordinates;
  previewScore?: number;
  status: SiteStatus;
}
