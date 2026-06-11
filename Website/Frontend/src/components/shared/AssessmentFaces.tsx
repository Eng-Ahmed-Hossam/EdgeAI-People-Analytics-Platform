"use client";

// Shared assessment face components — used by the workspace active-concept view
// (WorkspaceSidePanel VerdictPanel) and the report.
//
// RECONCILIATION DECISION (Session F):
// ValuationFace and CoverageFace render the same post-assessment supporting
// intelligence in both the workspace and the report. One shared implementation
// eliminates divergent rendering of the same data.
//
// Kept distinct (NOT extracted here, per Session C decision):
// AssetIntelligencePanel's coverage block — coverage-as-primary-subject
// (pre-concept, loading skeleton, per-band explanatory text, EdgeAI
// "active / not yet" labels). That context differs; a few shared lines are
// acceptable repetition rather than a premature abstraction.
//
// Honest-absence states:
// ValuationFace renders the same "Insufficient market transaction data" copy
// wherever valuation is null — workspace and report show an identical state.

import { cn, formatCurrency } from "@/lib/utils";
import type {
  ConceptAssessment,
  TemporalOutput,
  ValuationOutput,
  ValuationVerdict,
} from "@/types/assessment-v2";

// ── Valuation face ────────────────────────────────────────────────────────────
// Renders the valuation intelligence for a ConceptAssessment.
// The valuation engine runs in two modes; this component branches on `mode`:
//   - "listing_evaluation"    → is the listed price fair? (range, gap, market position)
//   - "opportunity_discovery" → how strong is the commercial potential? (no price)
// assessment.valuation === null → honest sparse state.
// Callers add their own section wrapper and header.

// Verdict → human label + colour tone. Underpriced / high potential read positive
// (green); overpriced / weak read negative (red); the middle reads neutral.
const VERDICT_LABELS: Record<ValuationVerdict, string> = {
  underpriced: "Underpriced",
  fairly_priced: "Fairly priced",
  overpriced: "Overpriced",
  high_potential: "High potential",
  moderate_potential: "Moderate potential",
  low_potential: "Low potential",
  insufficient_data: "Insufficient data",
};

function verdictTone(verdict: ValuationVerdict): string {
  if (verdict === "underpriced" || verdict === "high_potential") {
    return "border-verdict-good/60 bg-verdict-strong-wash text-verdict-strong";
  }
  if (verdict === "overpriced" || verdict === "low_potential") {
    return "border-verdict-weak/40 bg-verdict-weak-wash text-verdict-weak";
  }
  if (verdict === "moderate_potential") {
    return "border-verdict-warn/40 bg-verdict-warn-wash text-verdict-warn";
  }
  return "border-hairline text-ink-muted";
}

export function ValuationFace({ assessment }: { assessment: ConceptAssessment }) {
  const { valuation } = assessment;

  if (valuation === null) {
    return (
      <p className="font-body text-xs text-ink-faint leading-5">
        Insufficient market transaction data to estimate fair value for this asset.
        Valuation intelligence becomes available as comparable activity accumulates.
      </p>
    );
  }

  return valuation.mode === "opportunity_discovery" ? (
    <OpportunityFace valuation={valuation} />
  ) : (
    <ListingFace valuation={valuation} />
  );
}

// Mode A — listing evaluation: price fairness against the fair-value range.
function ListingFace({ valuation }: { valuation: ValuationOutput }) {
  const { fairValueRange, currency } = valuation;
  return (
    <div className="space-y-2.5">
      <div className="flex items-center justify-between gap-4">
        <span className="font-body text-xs text-ink-muted">Verdict</span>
        <span
          className={cn(
            "inline-flex items-center rounded-(--r-sm) border px-2 py-0.5",
            "font-body text-[10px] font-medium uppercase tracking-wide",
            verdictTone(valuation.verdict)
          )}
        >
          {VERDICT_LABELS[valuation.verdict]}
        </span>
      </div>
      <ValRow
        label="Fair value range"
        value={`${formatCurrency(fairValueRange.low, currency)}–${formatCurrency(
          fairValueRange.high,
          currency
        )}/mo`}
      />
      <ValRow
        label="Expected value"
        value={`${formatCurrency(valuation.expectedValue, currency)}/mo`}
      />
      {valuation.valuationGapPct !== null && (
        <ValRow
          label="Listed vs. fair"
          value={`${valuation.valuationGapPct > 0 ? "+" : ""}${valuation.valuationGapPct.toFixed(
            1
          )}%`}
        />
      )}
      <Drivers drivers={valuation.drivers} />
      <p className="font-body text-[11px] text-ink-faint leading-relaxed">
        {valuation.valuationNarrative}
      </p>
    </div>
  );
}

// Mode B — opportunity discovery: commercial-potential assessment, no listed price.
function OpportunityFace({ valuation }: { valuation: ValuationOutput }) {
  const hasIndicative = valuation.fairValueRange.high > 0;
  return (
    <div className="space-y-2.5">
      <div className="flex items-center justify-between gap-4">
        <span className="font-body text-xs text-ink-muted">Commercial potential</span>
        <span
          className={cn(
            "inline-flex items-center rounded-(--r-sm) border px-2 py-0.5",
            "font-body text-[10px] font-medium uppercase tracking-wide",
            verdictTone(valuation.verdict)
          )}
        >
          {VERDICT_LABELS[valuation.verdict]}
        </span>
      </div>
      <p className="font-body text-[11px] text-ink-faint leading-relaxed">
        No listing price — this is a commercial-potential assessment, not a price
        evaluation.
      </p>
      {hasIndicative && (
        <ValRow
          label="Indicative rent"
          value={`${formatCurrency(
            valuation.fairValueRange.low,
            valuation.currency
          )}–${formatCurrency(valuation.fairValueRange.high, valuation.currency)}/mo`}
        />
      )}
      <Drivers drivers={valuation.drivers} />
      <p className="font-body text-[11px] text-ink-faint leading-relaxed">
        {valuation.valuationNarrative}
      </p>
    </div>
  );
}

// Top valuation drivers (max 3) — traceable reasons behind the verdict.
function Drivers({ drivers }: { drivers: ValuationOutput["drivers"] }) {
  if (drivers.length === 0) return null;
  return (
    <div className="space-y-1.5 pt-0.5">
      {drivers.slice(0, 3).map((d, i) => (
        <div key={`${d.label}-${i}`} className="flex items-start gap-2">
          <span
            className={cn(
              "mt-1.5 h-1.5 w-1.5 shrink-0 rounded-full",
              d.direction === "positive"
                ? "bg-verdict-strong"
                : d.direction === "negative"
                  ? "bg-verdict-weak"
                  : "bg-ink-faint/40"
            )}
            aria-hidden="true"
          />
          <span className="font-body text-[11px] text-ink-muted leading-snug">
            <span className="text-ink-body">{d.label}:</span> {d.detail}
          </span>
        </div>
      ))}
    </div>
  );
}

function ValRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between gap-4">
      <span className="font-body text-xs text-ink-muted">{label}</span>
      <span className="font-mono text-sm font-medium text-ink-strong tabular-nums">
        {value}
      </span>
    </div>
  );
}

// ── Coverage face ─────────────────────────────────────────────────────────────
// Renders coverage score + source checklist + temporal availability note.
// Coverage score and checklist are always present.
// Temporal note is conditional on temporalConfidence.
// Callers add their own section wrapper and header.

export function CoverageFace({ temporal }: { temporal: TemporalOutput }) {
  const presentCount = temporal.sourceChecklist.filter((s) => s.present).length;
  const totalCount = temporal.sourceChecklist.length;

  const scoreTone =
    temporal.coverageScore >= 70
      ? "text-status-high"
      : temporal.coverageScore >= 45
        ? "text-status-med"
        : "text-status-low";

  return (
    <div>
      {/* Coverage score + source count — always present */}
      <div className="mb-3 flex items-center gap-1.5">
        <span
          className={cn(
            "font-mono text-sm font-semibold tabular-nums leading-none",
            scoreTone
          )}
        >
          {temporal.coverageScore}
        </span>
        <span className="font-body text-[10px] text-ink-faint">/100</span>
        <span className="mx-1 text-xs text-ink-faint/30">·</span>
        <span className="font-mono text-[10px] text-ink-faint">
          {presentCount}/{totalCount} sources
        </span>
      </div>

      {/* Source checklist */}
      <div className="space-y-1.5">
        {temporal.sourceChecklist.map((source) => (
          <div key={source.source} className="flex items-center gap-2">
            <span
              className={cn(
                "h-1.5 w-1.5 shrink-0 rounded-full",
                source.present ? "bg-status-high" : "bg-ink-faint/25"
              )}
              aria-hidden="true"
            />
            <span
              className={cn(
                "font-body text-xs",
                source.present ? "text-ink-muted" : "text-ink-faint"
              )}
            >
              {source.label}
            </span>
          </div>
        ))}
      </div>

      {/* Temporal availability note */}
      <p className="mt-3 font-body text-[11px] text-ink-faint leading-relaxed">
        {temporal.temporalConfidence
          ? `Temporal activity data available — ${temporal.forecastHorizon ?? "short-term"} forecast horizon.`
          : "Temporal activity data not yet available. Activity profiles require EdgeAI sensor coverage at this asset."}
      </p>
    </div>
  );
}
