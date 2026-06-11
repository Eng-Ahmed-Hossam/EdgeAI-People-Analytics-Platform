"""
Valuation engine — Phase 1: transparent rule-based valuation.

This is the second intelligence service (production architecture §8.2: Valuation
Service — "fair price and expected value"; status GROWTH, data model FOUNDATION).
It is DOWNSTREAM of the feasibility engine: it reads the feasibility signal as a
first-class input, not merely the asset's raw characteristics. The architecture's
discipline is honoured exactly as the feasibility engine honours it.

═══════════════════════════════════════════════════════════════════════════════
DESIGN PRINCIPLES (do not drift)
═══════════════════════════════════════════════════════════════════════════════

1. TRANSPARENT RULE-BASED ONLY.
   No ML, no black box. Every verdict is a function of justifiable inputs with
   stated reasoning. The fair-value estimate is an explicit arithmetic of a
   district market benchmark, a property-type multiplier, and a feasibility
   premium. A developer or investor can trace any output to its inputs.

2. WEIGHTS / BENCHMARKS ARE REASONED PRIORS, NOT CALIBRATION COEFFICIENTS.
   The district rent benchmarks below are reasoned from observable asking-rent
   patterns in Greater Cairo (see the per-tier rationale on each constant). They
   are NOT fitted to transaction data — there is no transaction dataset yet.
   Phase 2 replaces these priors with comparables learned from real rent /
   transaction / commercial-activity data as it accumulates. Do NOT represent
   these numbers as empirically calibrated.

3. VALUATION CONFIDENCE IS STRUCTURALLY MORE CONSERVATIVE THAN FEASIBILITY.
   Valuation depends on location data + price data + asset characteristics —
   strictly more uncertainty layers than feasibility (which depends on location
   data + characteristics alone). This engine derives the feasibility confidence
   from coverage and then applies an ADDITIONAL multiplicative discount
   (VALUATION_CONFIDENCE_DISCOUNT). This is a structural property, computed every
   time — not an afterthought. valuation_confidence < feasibility_confidence for
   the same asset, always.

4. COVERAGE GATES CONFIDENCE — THEN AN EXTRA DISCOUNT IS APPLIED.
   Coverage gates valuation confidence exactly as it gates feasibility confidence
   (more coverage → higher confidence, narrower range), and THEN the additional
   valuation discount is layered on. A 91-coverage asset that yields ~82%
   feasibility confidence yields ~69% valuation confidence here (see
   VALUATION_CONFIDENCE_DISCOUNT rationale).

5. NEVER CLAIM ABSOLUTE MARKET VALUE.
   The engine estimates a reasonable RANGE derived from available evidence and
   priors, and says so in the narrative. It never emits a single "the market
   value is X" claim. Output always carries a range and a confidence band.

═══════════════════════════════════════════════════════════════════════════════
THE TWO OPERATING MODES (determined by whether the asset has a listed price)
═══════════════════════════════════════════════════════════════════════════════

Mode A — LISTING EVALUATION  (asset has monthly_rent or sale_price)
   Assesses whether the listed price is fair relative to:
     · the asset's area (rent normalised to EGP/m²/month),
     · property type and district benchmarks (reasoned priors below), and
     · CRITICALLY the feasibility signal for the asset's commercial potential.
   The feasibility signal is the differentiator: two assets at the SAME listed
   price but with different feasibility scores produce DIFFERENT verdicts — the
   high-potential location reads underpriced, the weak one overpriced. A great
   location at the going rate is a bargain; a poor location at the going rate is
   overpriced. That is the engine's core intelligence.
   Output: verdict (underpriced / fairly_priced / overpriced), fair-value range,
   valuation gap %, confidence, drivers, narrative.

Mode B — OPPORTUNITY DISCOVERY  (asset has null price; OSM / unlisted assets)
   Cannot assess price fairness (there is no price). Instead assesses commercial
   POTENTIAL quality from the feasibility signal and location characteristics, so
   the ~317 OSM assets remain useful rather than valueless.
   Output: verdict (high_potential / moderate_potential / low_potential /
   insufficient_data), confidence, drivers, narrative, and an INDICATIVE fair-rent
   range (what comparable space would rent for) clearly labelled as not a price
   evaluation. The `mode` field tells the frontend to render this differently.

═══════════════════════════════════════════════════════════════════════════════
THE FEASIBILITY SIGNAL AS INPUT — what is available and what is inferred
═══════════════════════════════════════════════════════════════════════════════

The feasibility engine produces: score (0–100), score band, success likelihood,
a coverage-gated confidence interval, and contributing factors. It does NOT
produce a revenue figure — that is the future Forecasting/Temporal service. This
engine therefore INFERS commercial value from (feasibility score + band) combined
with district and property characteristics; it never consumes a revenue number
that does not exist.

How the signal is obtained (three cases from the brief):
  · Concept supplied  → run the feasibility engine for that concept = the richest,
    concept-specific signal. Because the feasibility engine is deterministic,
    recomputing it here is identical to consuming a previously-persisted
    assessment for the same concept; reading persisted assessments is a Phase-2
    optimisation that would yield the same numbers.
  · Concept absent     → run a LIGHTWEIGHT PROXY using a representative concept,
    at reduced confidence (PROXY_CONFIDENCE_PENALTY) — flagged in the narrative as
    a generic estimate, not a concept-specific assessment.
  · Coverage < 30      → treated as the low-confidence floor: Mode A still returns
    a (very wide, low-confidence, indicative-only) range; Mode B caps the verdict
    at low_potential / insufficient_data. See COVERAGE_FLOOR.

Phase 1 vs Phase 2:
  Phase 1 (this file) = reasoned priors: district benchmarks, a feasibility
  premium curve, and a reasoned confidence discount. Phase 2 = market-data
  calibrated: real rent/transaction comparables replace the benchmarks, a learned
  price model replaces the premium curve, and the confidence discount shrinks as
  comparable sparsity (a major uncertainty layer today) is reduced by real data.
"""

from __future__ import annotations

from datetime import UTC, datetime

from app.services.feasibility import (
    _district_lookup,
    compute_feasibility,
)

VALUATION_ENGINE_VERSION = "v1.0-rule-based"

# A representative concept used when the caller supplies none. "other" is the
# feasibility engine's generic profile (conservative modifiers, neutral weights).
_PROXY_CONCEPT = "other"


# ═══════════════════════════════════════════════════════════════════════════════
# REASONED PRIOR 1 — District rent benchmarks (EGP per m² per month)
#
# These are the "going rate" for ordinary retail space in each district tier,
# location-quality-agnostic beyond the tier itself. The feasibility premium (prior
# 3) then adjusts for how desirable a SPECIFIC use is at the location. Separating
# "what space costs here" (this benchmark) from "how valuable this use is here"
# (the feasibility premium) is the core of the model.
#
# RATIONALE (reasoned from observable Greater Cairo asking-rent patterns, NOT
# fitted to transaction data — see seed_data.json implied rents in parentheses):
#   prime_commercial 380 — Maadi (~329), Heliopolis (~353), Nasr City (~444),
#                          Zamalek (~633, a premium outlier). 380 sits in the
#                          lower-middle of prime asking rents: a defensible centre
#                          that reads strong prime locations as underpriced.
#   active_urban     250 — Mohandeseen/Dokki/October corridor; clearly below prime,
#                          clearly above dense-residential.
#   dense_residential 180 — Imbaba (~200); high footfall but low commercial premium.
#   peripheral        95 — New Fayoum (~83); thin commercial demand, low rents.
#   limited_context  200 — unknown/Greater-Cairo fallback; mid-range, paired with a
#                          wider range and lower confidence (we don't know the area).
# The tier label comes from the SAME district table the feasibility engine uses
# (_district_lookup), so valuation and feasibility never disagree about the area.
# ═══════════════════════════════════════════════════════════════════════════════
_DISTRICT_RENT_BENCHMARK: dict[str, int] = {
    "prime_commercial":  380,
    "active_urban":      250,
    "dense_residential": 180,
    "peripheral":         95,
    "limited_context":   200,
}
_DEFAULT_RENT_BENCHMARK = 200

# Sale-price benchmark (EGP per m², purchase) — coarse prior, used only when an
# asset lists a sale price but NO monthly rent (rare; no seeded asset is sale-only).
# Reasoned as ~110× monthly rent benchmark (a ~0.9% monthly gross yield is typical
# for Egyptian commercial property). Marked coarse: the rent path is the primary,
# better-grounded path.
_DISTRICT_SALE_BENCHMARK: dict[str, int] = {
    tier: bench * 110 for tier, bench in _DISTRICT_RENT_BENCHMARK.items()
}
_DEFAULT_SALE_BENCHMARK = _DEFAULT_RENT_BENCHMARK * 110


# ═══════════════════════════════════════════════════════════════════════════════
# REASONED PRIOR 2 — Property-type rent multipliers (× the district benchmark)
#
# Different formats command different rent per m². RATIONALE:
#   kiosk 1.30                  — tiny footprint, very high rent/m² (prime frontage)
#   ground_floor_commercial 1.05 — premium street frontage vs a plain retail unit
#   retail_unit 1.00            — the reference format
#   mixed_use 0.90              — partly non-retail
#   office_unit 0.80            — office rents/m² sit below prime retail
#   warehouse 0.45              — large, cheap-per-m² space
#   other 0.90                  — conservative default
# ═══════════════════════════════════════════════════════════════════════════════
_PROPERTY_RENT_MULT: dict[str, float] = {
    "kiosk":                   1.30,
    "ground_floor_commercial": 1.05,
    "retail_unit":             1.00,
    "mixed_use":               0.90,
    "office_unit":             0.80,
    "warehouse":               0.45,
    "other":                   0.90,
}
_DEFAULT_PROPERTY_MULT = 0.90


# ═══════════════════════════════════════════════════════════════════════════════
# REASONED PRIOR 3 — The feasibility premium curve (the differentiator)
#
# A location's commercial DESIRABILITY for a given use adjusts its fair value: a
# tenant rationally pays more for a location where the business will thrive, and
# less where it will struggle. We translate the feasibility score into a premium /
# discount on the district market value.
#
#   feasibility_premium = clamp((score - PIVOT)/100 × SENSITIVITY, MIN, MAX)
#
#   PIVOT = 55           — an "average" feasibility for a location. At the pivot the
#                          premium is 0: the use is neither a draw nor a drag, so
#                          fair value equals the plain district market value.
#   SENSITIVITY = 1.30   — strength of the effect. Chosen so that an above-average
#                          location (score ~82) earns a meaningful premium (+0.35,
#                          capped) and a weak one (score ~41) a meaningful discount
#                          (~-0.18). This is what makes the SAME listed price read
#                          underpriced at a strong location and overpriced at a weak
#                          one — the engine's core intelligence.
#   MIN/MAX = -0.35/+0.40 — the premium cannot run away; even an exceptional
#                          location justifies at most +40% over the district rate,
#                          and even a poor one is not discounted below -35%.
# These are reasoned priors about how location desirability capitalises into rent,
# NOT coefficients fitted to data.
# ═══════════════════════════════════════════════════════════════════════════════
_FEASIBILITY_PIVOT = 55
_FEASIBILITY_SENSITIVITY = 1.30
_FEASIBILITY_PREMIUM_MIN = -0.35
_FEASIBILITY_PREMIUM_MAX = 0.40


def _feasibility_premium(feasibility_score: int) -> float:
    raw = (feasibility_score - _FEASIBILITY_PIVOT) / 100.0 * _FEASIBILITY_SENSITIVITY
    return max(_FEASIBILITY_PREMIUM_MIN, min(_FEASIBILITY_PREMIUM_MAX, raw))


# ═══════════════════════════════════════════════════════════════════════════════
# REASONED PRIOR 4 — Confidence model (coverage gate + the additional discount)
#
# Step 1: feasibility confidence as a function of coverage. Anchored to the brief's
#   example (91 coverage → ~82% feasibility confidence):
#     feas_conf = FEAS_CONF_FLOOR + FEAS_CONF_SLOPE × (coverage/100)
#   FEAS_CONF_FLOOR = 0.35 — a transparent prior-based engine is never more than
#                            ~35% confident on zero data (OSM coverage = 0).
#   FEAS_CONF_SLOPE = 0.52 — rises to 0.35 + 0.52 = 0.87 at full coverage; it never
#                            reaches certainty (these are priors, not calibration).
#                            At coverage 91 → 0.35 + 0.473 = 0.823 ≈ 82% (matches
#                            the brief's anchor).
#
# Step 2: the ADDITIONAL valuation discount.
#   valuation_conf = feas_conf × VALUATION_CONFIDENCE_DISCOUNT
#   VALUATION_CONFIDENCE_DISCOUNT = 0.84 — a reasoned ~16% confidence haircut for
#   the extra uncertainty layers valuation carries over feasibility:
#     (1) listed-price quality / staleness — asking prices are noisy and negotiable;
#     (2) comparable sparsity — Egyptian secondary-market rent/transaction comps are
#         thin, so the district benchmark is itself uncertain;
#     (3) no calibrated price model yet — Phase 1 reasons from priors, not a learned
#         market model.
#   0.84 maps the 82% feasibility-confidence anchor to ~69% valuation confidence
#   (0.823 × 0.84 = 0.691), inside the brief's 68–72% target. Phase 2 raises this
#   toward 1.0 as real comparables shrink layer (2) and a learned model removes (3).
#
# valuation_confidence < feasibility_confidence is therefore guaranteed by
# construction (multiply by a positive factor < 1), for every asset.
# ═══════════════════════════════════════════════════════════════════════════════
_FEAS_CONF_FLOOR = 0.35
_FEAS_CONF_SLOPE = 0.52
VALUATION_CONFIDENCE_DISCOUNT = 0.84

# When no concept is supplied, the feasibility signal is a generic proxy rather
# than a concept-specific assessment; discount valuation confidence a little more.
PROXY_CONFIDENCE_PENALTY = 0.92

# Below this coverage the asset is at the low-confidence floor: Mode A returns an
# indicative-only wide range; Mode B caps verdict at low_potential / insufficient.
COVERAGE_FLOOR = 30


def _feasibility_confidence(coverage_score: int) -> float:
    """Feasibility confidence in [floor, floor+slope], purely coverage-driven."""
    cov = max(0, min(100, coverage_score)) / 100.0
    return _FEAS_CONF_FLOOR + _FEAS_CONF_SLOPE * cov


def _valuation_confidence_numeric(coverage_score: int, *, concept_supplied: bool) -> float:
    conf = _feasibility_confidence(coverage_score) * VALUATION_CONFIDENCE_DISCOUNT
    if not concept_supplied:
        conf *= PROXY_CONFIDENCE_PENALTY
    return conf


def _confidence_band(numeric: float) -> str:
    """Map a numeric valuation confidence to the frontend band."""
    if numeric >= 0.70:
        return "high"
    if numeric >= 0.50:
        return "medium"
    return "low"


# ═══════════════════════════════════════════════════════════════════════════════
# REASONED PRIOR 5 — Fair-value range half-width (coverage-gated, % of centre)
#
# The displayed fair-value range is centre × (1 ± half_width_frac). The width is
# coverage-gated and set DELIBERATELY WIDER than the feasibility CI's relative
# width, to make valuation's extra uncertainty visible (criterion 4: a low-coverage
# asset must show a visibly wide range). The verdict is tied to this same band:
# the listed price is "underpriced/overpriced" exactly when it falls outside the
# displayed range, so verdict and range never contradict each other, and a
# low-coverage (wide-range) asset is humbler about calling a price mispriced.
# ═══════════════════════════════════════════════════════════════════════════════
def _range_half_width(coverage_score: int) -> float:
    if coverage_score >= 70:
        return 0.08
    if coverage_score >= 45:
        return 0.15
    if coverage_score >= COVERAGE_FLOOR:
        return 0.25
    return 0.35  # includes coverage = 0 (OSM)


def _round_money(x: float) -> int:
    """Round to the nearest 100 for rents, nearest 1000 for large sale prices."""
    if x >= 500_000:
        return int(round(x / 1000.0) * 1000)
    return int(round(x / 100.0) * 100)


# ═══════════════════════════════════════════════════════════════════════════════
# Internal: obtain the feasibility signal for this asset.
# ═══════════════════════════════════════════════════════════════════════════════
def _feasibility_signal(
    *,
    asset_id: str,
    area: str,
    size: float | None,
    monthly_rent: float | None,
    property_type: str,
    coverage_score: int,
    concept: str | None,
) -> dict:
    """Run the feasibility engine and return its output dict (the consumed signal)."""
    concept_key = concept if concept else _PROXY_CONCEPT
    return compute_feasibility(
        asset_id=asset_id,
        area=area,
        size=size,
        monthly_rent=monthly_rent,
        property_type=property_type,
        coverage_score=coverage_score,
        concept=concept_key,
    )


def _market_value(
    *,
    tier_label: str,
    property_type: str,
    size: float | None,
    basis: str,
) -> tuple[float | None, float]:
    """
    Return (market_value, benchmark_per_m2) for the chosen basis.
    market_value is None when size is unknown (cannot normalise) — the caller then
    falls back to a size-agnostic confidence-only treatment.
    """
    prop_mult = _PROPERTY_RENT_MULT.get(property_type, _DEFAULT_PROPERTY_MULT)
    if basis == "sale_price":
        bench = _DISTRICT_SALE_BENCHMARK.get(tier_label, _DEFAULT_SALE_BENCHMARK)
    else:
        bench = _DISTRICT_RENT_BENCHMARK.get(tier_label, _DEFAULT_RENT_BENCHMARK)
    per_m2 = bench * prop_mult
    if size is None or size <= 0:
        return None, per_m2
    return per_m2 * size, per_m2


def _drivers_from_feasibility(feasibility: dict) -> list[dict]:
    """Translate the feasibility contributing factors into valuation drivers."""
    drivers: list[dict] = []
    for f in feasibility["contributing_factors"]:
        drivers.append(
            {
                "label": f["label"],
                "detail": f["value"],
                "direction": f["direction"],
            }
        )
    return drivers


# ═══════════════════════════════════════════════════════════════════════════════
# Mode A — Listing Evaluation
# ═══════════════════════════════════════════════════════════════════════════════
def _evaluate_listing(
    *,
    feasibility: dict,
    listed_price: float,
    basis: str,             # "monthly_rent" | "sale_price"
    tier_label: str,
    tier_desc: str,
    property_type: str,
    size: float | None,
    currency: str,
    coverage_score: int,
    concept: str | None,
) -> dict:
    feas_score = feasibility["feasibility_score"]
    feas_band = feasibility["score_band"]
    premium = _feasibility_premium(feas_score)

    market_value, per_m2 = _market_value(
        tier_label=tier_label, property_type=property_type, size=size, basis=basis
    )

    half = _range_half_width(coverage_score)
    period = "/mo" if basis == "monthly_rent" else ""

    # If size is unknown we cannot anchor a market value; fall back to treating the
    # listed price itself as the centre, with a wide range and a clear caveat.
    if market_value is None:
        centre = listed_price * (1.0 + premium * 0.5)  # nudge by half the premium
        size_note = " Floor area is not listed, so the fair-value estimate is anchored loosely to the asking price; treat it as indicative."
    else:
        centre = market_value * (1.0 + premium)
        size_note = ""

    fair_low = centre * (1.0 - half)
    fair_high = centre * (1.0 + half)

    # Verdict tied to the displayed range (listed outside [low, high] ⇔ mispriced).
    gap_centre = (listed_price - centre) / centre  # signed; <0 = listed below fair
    if gap_centre < -half:
        verdict = "underpriced"
        price_delta = "below_market"
    elif gap_centre > half:
        verdict = "overpriced"
        price_delta = "above_market"
    else:
        verdict = "fairly_priced"
        price_delta = "at_market"

    # Expected value: the value the asset could command if its commercial potential
    # is realised — market value scaled by the full feasibility premium plus a small
    # demand uplift (0.06). Always >= centre by the uplift, matching the intuition
    # that a tenant's realised value sits at or above the fair asking value.
    base_for_expected = market_value if market_value is not None else listed_price
    expected_value = base_for_expected * (1.0 + premium + 0.06)

    fair_estimate_i = _round_money(centre)
    fair_low_i = _round_money(fair_low)
    fair_high_i = _round_money(fair_high)
    expected_i = _round_money(expected_value)

    # Gap relative to the range bounds, the form the brief asks for
    # ("+17.6% to +42.8% above estimated fair range").
    gap_vs_high = (listed_price - fair_high) / fair_high * 100.0
    gap_vs_low = (listed_price - fair_low) / fair_low * 100.0
    gap_centre_pct = round(gap_centre * 100.0, 1)

    conf_numeric = _valuation_confidence_numeric(
        coverage_score, concept_supplied=concept is not None
    )
    confidence = _confidence_band(conf_numeric)

    drivers = _drivers_from_feasibility(feasibility)
    # Prepend the two valuation-specific drivers (potential + price-vs-benchmark).
    potential_dir = "positive" if feas_band == "strong" else ("neutral" if feas_band == "moderate" else "negative")
    drivers.insert(
        0,
        {
            "label": "Commercial potential",
            "detail": f"Feasibility {feas_score}/100 ({feas_band}) — {'raises' if premium > 0.02 else ('lowers' if premium < -0.02 else 'neutral to')} fair value",
            "direction": potential_dir,
        },
    )
    if size is not None and size > 0:
        drivers.insert(
            1,
            {
                "label": "Price vs district benchmark",
                "detail": f"Listed {currency} {listed_price/size:,.0f}/m² vs benchmark {currency} {per_m2:,.0f}/m²",
                "direction": "positive" if price_delta == "below_market" else ("negative" if price_delta == "above_market" else "neutral"),
            },
        )

    narrative = _listing_narrative(
        verdict=verdict,
        listed_price=listed_price,
        fair_low_i=fair_low_i,
        fair_high_i=fair_high_i,
        gap_vs_low=gap_vs_low,
        gap_vs_high=gap_vs_high,
        currency=currency,
        period=period,
        tier_desc=tier_desc,
        feas_score=feas_score,
        feas_band=feas_band,
        premium=premium,
        coverage_score=coverage_score,
        confidence=confidence,
        concept=concept,
        size_note=size_note,
    )

    return {
        "mode": "listing_evaluation",
        "verdict": verdict,
        "fair_price_estimate": fair_estimate_i,
        "fair_value_range": {"low": fair_low_i, "high": fair_high_i},
        "expected_value": expected_i,
        "currency": currency,
        "confidence": confidence,
        "price_delta": price_delta,
        "valuation_gap_pct": gap_centre_pct,
        "drivers": drivers,
        "valuation_narrative": narrative,
        "generated_at": datetime.now(UTC).isoformat(),
    }


def _listing_narrative(
    *,
    verdict: str,
    listed_price: float,
    fair_low_i: int,
    fair_high_i: int,
    gap_vs_low: float,
    gap_vs_high: float,
    currency: str,
    period: str,
    tier_desc: str,
    feas_score: int,
    feas_band: str,
    premium: float,
    coverage_score: int,
    confidence: str,
    concept: str | None,
    size_note: str,
) -> str:
    listed_str = f"{currency} {listed_price:,.0f}{period}"
    range_str = f"{currency} {fair_low_i:,.0f}–{fair_high_i:,.0f}{period}"

    if verdict == "underpriced":
        # gaps are negative; report magnitude below the range
        lo = abs(gap_vs_low)
        hi = abs(gap_vs_high)
        lead = (
            f"The listed price of {listed_str} appears UNDERPRICED: it sits "
            f"{min(lo, hi):.1f}%–{max(lo, hi):.1f}% below the estimated fair range of {range_str}."
        )
    elif verdict == "overpriced":
        lo = gap_vs_low
        hi = gap_vs_high
        lead = (
            f"The listed price of {listed_str} appears OVERPRICED: it sits "
            f"{min(lo, hi):.1f}%–{max(lo, hi):.1f}% above the estimated fair range of {range_str}."
        )
    else:
        lead = (
            f"The listed price of {listed_str} appears FAIRLY PRICED: it falls "
            f"within the estimated fair range of {range_str}."
        )

    if premium > 0.02:
        why = (
            f" The {tier_desc} combined with strong commercial potential "
            f"(feasibility {feas_score}/100, {feas_band}) supports a fair value above the plain district rate."
        )
    elif premium < -0.02:
        why = (
            f" The {tier_desc} and weak commercial potential "
            f"(feasibility {feas_score}/100, {feas_band}) pull the fair value below the plain district rate."
        )
    else:
        why = (
            f" Commercial potential is average for this {tier_desc} "
            f"(feasibility {feas_score}/100), so fair value tracks the district rate."
        )

    if concept is None:
        proxy = (
            " This estimate uses a generic commercial profile because no specific business"
            " concept was provided; a concept-specific assessment would sharpen it."
        )
    else:
        proxy = ""

    if coverage_score < COVERAGE_FLOOR:
        cov = (
            f" Data coverage is very limited ({coverage_score}/100), so the range is wide and the"
            f" {confidence}-confidence verdict is indicative only — additional local price evidence"
            " would materially sharpen it before committing capital."
        )
    elif coverage_score < 70:
        cov = (
            f" Data coverage is partial ({coverage_score}/100); confidence is {confidence}."
            " This is a reasoned estimate from district priors, not a calibrated market valuation."
        )
    else:
        cov = (
            f" Data coverage is strong ({coverage_score}/100); confidence is {confidence}."
            " This remains a reasoned-prior estimate (Phase 1), not a transaction-calibrated valuation."
        )

    return lead + why + proxy + size_note + cov


# ═══════════════════════════════════════════════════════════════════════════════
# Mode B — Opportunity Discovery (no listed price)
# ═══════════════════════════════════════════════════════════════════════════════
def _discover_opportunity(
    *,
    feasibility: dict,
    tier_label: str,
    tier_desc: str,
    property_type: str,
    size: float | None,
    currency: str,
    coverage_score: int,
    concept: str | None,
) -> dict:
    feas_score = feasibility["feasibility_score"]
    feas_band = feasibility["score_band"]
    premium = _feasibility_premium(feas_score)

    conf_numeric = _valuation_confidence_numeric(
        coverage_score, concept_supplied=concept is not None
    )
    confidence = _confidence_band(conf_numeric)

    # Verdict from feasibility band, gated by coverage. Below the coverage floor we
    # cannot responsibly claim high/moderate potential — the claim is capped. We
    # only fall all the way to insufficient_data when we know almost nothing: near-
    # zero coverage AND an unrecognised district (limited_context). A near-zero-
    # coverage asset in a KNOWN district still carries a district-based signal worth
    # surfacing as low_potential, which keeps unlisted/OSM assets useful rather than
    # valueless.
    unknown_district = tier_label == "limited_context"
    if coverage_score < COVERAGE_FLOOR:
        if coverage_score < 15 and unknown_district:
            verdict = "insufficient_data"
        else:
            verdict = "low_potential"  # capped: signal present, coverage thin
    elif feas_band == "strong":
        verdict = "high_potential"
    elif feas_band == "moderate":
        verdict = "moderate_potential"
    else:
        verdict = "low_potential"

    # Indicative fair-rent range (what comparable space WOULD rent for) — NOT a
    # price evaluation, since there is no listed price. Clearly labelled as such.
    market_value, per_m2 = _market_value(
        tier_label=tier_label, property_type=property_type, size=size, basis="monthly_rent"
    )
    half = _range_half_width(coverage_score)
    if market_value is not None:
        centre = market_value * (1.0 + premium)
        fair_low_i = _round_money(centre * (1.0 - half))
        fair_high_i = _round_money(centre * (1.0 + half))
        fair_estimate_i = _round_money(centre)
        expected_i = _round_money(market_value * (1.0 + premium + 0.06))
    else:
        # No size → no indicative rent. Honest zeros; narrative explains.
        fair_low_i = fair_high_i = fair_estimate_i = expected_i = 0

    drivers = _drivers_from_feasibility(feasibility)
    potential_dir = "positive" if feas_band == "strong" else ("neutral" if feas_band == "moderate" else "negative")
    drivers.insert(
        0,
        {
            "label": "Commercial potential",
            "detail": f"Feasibility {feas_score}/100 ({feas_band}) for {tier_desc}",
            "direction": potential_dir,
        },
    )

    narrative = _opportunity_narrative(
        verdict=verdict,
        tier_desc=tier_desc,
        feas_score=feas_score,
        feas_band=feas_band,
        fair_low_i=fair_low_i,
        fair_high_i=fair_high_i,
        currency=currency,
        coverage_score=coverage_score,
        confidence=confidence,
        concept=concept,
        has_indicative=(market_value is not None),
    )

    return {
        "mode": "opportunity_discovery",
        "verdict": verdict,
        "fair_price_estimate": fair_estimate_i,
        "fair_value_range": {"low": fair_low_i, "high": fair_high_i},
        "expected_value": expected_i,
        "currency": currency,
        "confidence": confidence,
        "price_delta": None,          # no listed price to compare against
        "valuation_gap_pct": None,    # no gap without a listed price
        "drivers": drivers,
        "valuation_narrative": narrative,
        "generated_at": datetime.now(UTC).isoformat(),
    }


def _opportunity_narrative(
    *,
    verdict: str,
    tier_desc: str,
    feas_score: int,
    feas_band: str,
    fair_low_i: int,
    fair_high_i: int,
    currency: str,
    coverage_score: int,
    confidence: str,
    concept: str | None,
    has_indicative: bool,
) -> str:
    label = {
        "high_potential": "HIGH commercial potential",
        "moderate_potential": "MODERATE commercial potential",
        "low_potential": "LOW commercial potential",
        "insufficient_data": "INSUFFICIENT data to assess potential",
    }[verdict]

    lead = (
        f"No listing price is available for this asset, so this is a commercial-potential"
        f" assessment, not a price evaluation. The location shows {label}"
        f" (feasibility {feas_score}/100, {feas_band}) for a {tier_desc}."
    )

    if has_indicative and fair_high_i > 0:
        ind = (
            f" For reference only, comparable space here would indicatively rent for"
            f" {currency} {fair_low_i:,.0f}–{fair_high_i:,.0f}/mo — an estimate of market rent,"
            " not a judgement of any listed price."
        )
    else:
        ind = (
            " Floor area is not listed, so no indicative rent range can be estimated."
        )

    if concept is None:
        proxy = " A specific business concept would yield a sharper potential read."
    else:
        proxy = ""

    if coverage_score < COVERAGE_FLOOR:
        cov = (
            f" Data coverage is very limited ({coverage_score}/100); confidence is {confidence}."
            " This is an early, indicative signal — collect more local evidence before relying on it."
        )
    else:
        cov = (
            f" Data coverage is {coverage_score}/100; confidence is {confidence}."
            " This is a reasoned-prior estimate (Phase 1)."
        )

    return lead + ind + proxy + cov


# ═══════════════════════════════════════════════════════════════════════════════
# Main entry point
# ═══════════════════════════════════════════════════════════════════════════════
def compute_valuation(
    *,
    asset_id: str,
    area: str,
    size: float | None,
    monthly_rent: float | None,
    sale_price: float | None,
    property_type: str,
    coverage_score: int,
    currency: str,
    concept: str | None,
) -> dict:
    """
    Compute a valuation result for the given asset (and optional concept).

    Returns a dict whose keys are snake_case; the Pydantic response model at the
    router aliases them to the camelCase ValuationOutput the frontend expects.

    Mode is chosen by the presence of a listed price:
      · monthly_rent or sale_price present → Mode A (Listing Evaluation)
      · neither present                    → Mode B (Opportunity Discovery)
    The feasibility signal is consumed in both modes (the differentiator in A; the
    primary substance in B).
    """
    coverage_score = max(0, min(100, coverage_score))

    feasibility = _feasibility_signal(
        asset_id=asset_id,
        area=area,
        size=size,
        monthly_rent=monthly_rent,
        property_type=property_type,
        coverage_score=coverage_score,
        concept=concept,
    )

    _, tier_label, tier_desc = _district_lookup(area)

    has_price = (monthly_rent is not None and monthly_rent > 0) or (
        sale_price is not None and sale_price > 0
    )

    if has_price:
        if monthly_rent is not None and monthly_rent > 0:
            listed_price, basis = monthly_rent, "monthly_rent"
        else:
            listed_price, basis = sale_price, "sale_price"
        return _evaluate_listing(
            feasibility=feasibility,
            listed_price=listed_price,
            basis=basis,
            tier_label=tier_label,
            tier_desc=tier_desc,
            property_type=property_type,
            size=size,
            currency=currency,
            coverage_score=coverage_score,
            concept=concept,
        )

    return _discover_opportunity(
        feasibility=feasibility,
        tier_label=tier_label,
        tier_desc=tier_desc,
        property_type=property_type,
        size=size,
        currency=currency,
        coverage_score=coverage_score,
        concept=concept,
    )
