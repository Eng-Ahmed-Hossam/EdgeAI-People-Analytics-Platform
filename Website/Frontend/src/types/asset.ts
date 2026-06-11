// ─────────────────────────────────────────────────────────────────────────────
// Asset — the central domain object (SYSTEM_DESIGN.md §2.1)
//
// An Asset is a real commercial opportunity (rent or purchase) seeded by the
// platform. It is the stable spine onto which all intelligence attaches.
// Users discover Assets; they do not create or manage them (single-sided product).
//
// Supersedes: Site (src/types/site.ts) as the central object.
// Site may persist as an internal/future concept but the product is asset-first.
// ─────────────────────────────────────────────────────────────────────────────

export type PropertyType =
  | "retail_unit"
  | "ground_floor_commercial"
  | "office_unit"
  | "kiosk"
  | "warehouse"
  | "mixed_use"
  | "other";

export type AvailabilityStatus =
  | "available"
  | "reserved"
  | "occupied"
  | "sold";

// Near-term: seeded/public. Long-term: partners, providers, direct onboarding.
export type ListingSource =
  | "seeded"
  | "public_registry"
  | "partner"
  | "direct"
  | "other";

export interface AssetCoordinates {
  lat: number;
  lng: number;
}

export interface Asset {
  // ── Identity & location ─────────────────────────────────────────────────────
  id: string;
  coordinates: AssetCoordinates;
  address: string;
  area: string;          // district / neighbourhood label (e.g. "Maadi", "Heliopolis")
  size?: number;         // sq metres; optional

  // ── Commercial terms ────────────────────────────────────────────────────────
  // The listed price the valuation service reasons against.
  // Either or both may be present depending on the listing type.
  monthlyRent?: number;  // asking monthly rent in local currency
  salePrice?: number;    // asking purchase price
  currency: string;      // ISO 4217, e.g. "EGP"

  // ── Classification ─────────────────────────────────────────────────────────
  propertyType: PropertyType;
  availabilityStatus: AvailabilityStatus;

  // ── Provenance ─────────────────────────────────────────────────────────────
  listingSource: ListingSource;

  // ── Intelligence readiness ─────────────────────────────────────────────────
  // Surfaced on asset selection, before concept choice (SYSTEM_DESIGN.md §2.4).
  // A coverage score of 0 means the platform has almost no intelligence about
  // this asset; 100 means all six data sources are present.
  coverageScore: number; // 0–100
}
