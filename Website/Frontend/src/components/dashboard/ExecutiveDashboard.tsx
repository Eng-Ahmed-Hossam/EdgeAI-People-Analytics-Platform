"use client";

import { useMemo } from "react";
import Link from "next/link";
import { useQuery } from "@tanstack/react-query";
import { useAssessmentStore } from "@/store/assessmentStore";
import { getAssets, getExecutiveDashboard, queryKeys } from "@/lib/api";
import { bandColors, bandLabel, cn, formatDate } from "@/lib/utils";
import {
  IntelligenceErrorState,
  IntelligenceLoadingState,
} from "@/components/primitives/IntelligenceStates";
import type {
  AssessmentPipelineItem,
  ExecutiveDashboardData,
  RecommendedDashboardAction,
} from "@/types/assessment";
import type { Asset } from "@/types/asset";
import type { ConceptAssessment } from "@/types/assessment-v2";

// ── Concept label lookup ──────────────────────────────────────────────────────

const CONCEPT_LABELS: Record<string, string> = {
  specialty_coffee: "Specialty Coffee",
  casual_dining: "Casual Dining",
  fast_casual_restaurant: "Fast Casual",
  pharmacy: "Pharmacy",
  premium_retail: "Premium Retail",
  fitness_studio: "Fitness Studio",
  beauty_wellness: "Beauty & Wellness",
};

function conceptLabel(concept: string): string {
  return CONCEPT_LABELS[concept] ?? concept.replace(/_/g, " ");
}

// ── Entry points (interface unchanged — page.tsx keeps working) ───────────────

export function ExecutiveDashboard() {
  return <ExecutiveDashboardClient />;
}

export function ExecutiveDashboardWithData({
  initialData,
}: {
  initialData: ExecutiveDashboardData;
}) {
  return <ExecutiveDashboardClient initialData={initialData} />;
}

// ── Recent assessments from the live store ────────────────────────────────────
// Reads allConceptAssessments (new asset-centric model). Each ConceptAssessment
// carries assetId but not asset metadata, so getAssets() is called alongside to
// enrich with address/area. No store modification required.

function useSessionAssessments(): {
  list: (ConceptAssessment & { asset?: Asset })[];
  total: number;
  isLoading: boolean;
} {
  const allConceptAssessments = useAssessmentStore((s) => s.allConceptAssessments);

  const { data: assets = [], isLoading } = useQuery({
    queryKey: queryKeys.assets,
    queryFn: getAssets,
    staleTime: 5 * 60 * 1000,
  });

  const assetMap = useMemo(
    () => new Map(assets.map((a) => [a.id, a])),
    [assets]
  );

  const list = useMemo(() => {
    return Object.values(allConceptAssessments)
      .flatMap((record) =>
        Object.values(record).filter((a): a is ConceptAssessment => !!a)
      )
      .map((a) => ({ ...a, asset: assetMap.get(a.assetId) }))
      .sort(
        (a, b) =>
          new Date(b.generatedAt).getTime() - new Date(a.generatedAt).getTime()
      )
      .slice(0, 8);
  }, [allConceptAssessments, assetMap]);

  const total = useMemo(() => {
    return Object.values(allConceptAssessments).reduce(
      (sum, record) => sum + Object.keys(record).length,
      0
    );
  }, [allConceptAssessments]);

  return { list, total, isLoading };
}

// ── Client component ──────────────────────────────────────────────────────────

function ExecutiveDashboardClient({
  initialData,
}: {
  initialData?: ExecutiveDashboardData;
}) {
  const dashboardQuery = useQuery({
    queryKey: queryKeys.dashboard,
    queryFn: getExecutiveDashboard,
    initialData,
  });

  const { list: sessionAssessments, total: sessionTotal, isLoading: assessmentsLoading } =
    useSessionAssessments();

  if (dashboardQuery.isLoading) {
    return (
      <main className="min-h-full bg-page px-5 py-8">
        <div className="mx-auto max-w-[960px]">
          <IntelligenceLoadingState
            kicker="Assembling dashboard"
            title="Loading intelligence overview"
            summary="Preparing active opportunities and recommended actions."
          />
        </div>
      </main>
    );
  }

  if (dashboardQuery.isError || !dashboardQuery.data) {
    return (
      <main className="min-h-full bg-page px-5 py-8">
        <IntelligenceErrorState
          className="mx-auto max-w-[960px]"
          kicker="Dashboard unavailable"
          title="Intelligence overview could not be loaded"
          summary="The dashboard request did not complete. Saved assessments are preserved — retry once the API seam is available."
        />
      </main>
    );
  }

  const data = dashboardQuery.data;
  const spotlight = data.opportunitySpotlight[0] ?? null;

  return (
    <main className="min-h-full bg-page text-ink-body">
      <div className="mx-auto max-w-[960px] px-5 py-8 lg:px-8">
        {/* 1. Executive header — metrics above the fold */}
        <ExecutiveHeader data={data} sessionTotal={sessionTotal} />

        {/* 2. Opportunity spotlight — single featured opportunity */}
        <section className="mt-10">
          <SectionLabel>Opportunity Spotlight</SectionLabel>
          {spotlight ? (
            <SpotlightCard item={spotlight} />
          ) : (
            <EmptyCard
              title="No active opportunities"
              detail="Run assessments in the workspace — high-confidence results will surface here."
              href="/workspace"
              cta="Open workspace"
            />
          )}
        </section>

        {/* 3. Recommended actions */}
        {data.recommendedActions.length > 0 && (
          <section className="mt-10">
            <SectionLabel>Recommended Actions</SectionLabel>
            <ActionsList actions={data.recommendedActions.slice(0, 3)} />
          </section>
        )}

        {/* 4. Recent assessments — reads from the live store */}
        <section className="mt-10 mb-8">
          <SectionLabel>Recent Assessments</SectionLabel>
          {assessmentsLoading && sessionAssessments.length === 0 ? (
            <div className="space-y-1.5">
              {[0, 1, 2].map((i) => (
                <div
                  key={i}
                  className="h-14 animate-pulse rounded-(--r-md) bg-surface"
                />
              ))}
            </div>
          ) : sessionAssessments.length > 0 ? (
            <RecentAssessmentsTable assessments={sessionAssessments} />
          ) : (
            <EmptyCard
              title="No assessments this session"
              detail="Select a location in the workspace, run a concept assessment, and it will appear here."
              href="/workspace"
              cta="Start exploring"
            />
          )}
        </section>
      </div>
    </main>
  );
}

// ── Executive header ──────────────────────────────────────────────────────────

function ExecutiveHeader({
  data,
  sessionTotal,
}: {
  data: ExecutiveDashboardData;
  sessionTotal: number;
}) {
  return (
    <header className="border-b border-hairline pb-8">
      <p className="font-body text-[0.65rem] font-medium uppercase tracking-widest text-ink-faint">
        EdgeAI Location Intelligence
      </p>
      <h1 className="mt-3 font-display text-[2rem] leading-tight text-ink-strong lg:text-[2.5rem]">
        Intelligence Overview
      </h1>

      {/* Four terse metrics — the three above the fold */}
      <div className="mt-6 flex flex-wrap items-end gap-x-8 gap-y-5">
        <InlineMetric value={data.summary.activeAssessments} label="Active" />
        <div className="hidden h-8 w-px bg-hairline sm:block" />
        <InlineMetric
          value={data.summary.highConfidenceOpportunities}
          label="High confidence"
        />
        <div className="hidden h-8 w-px bg-hairline sm:block" />
        <InlineMetric
          value={data.summary.needsReview}
          label="Needs attention"
          tone="warn"
        />
        <div className="hidden h-8 w-px bg-hairline sm:block" />
        <InlineMetric
          value={sessionTotal}
          label="This session"
          tone="accent"
          live
        />
      </div>

      {data.summary.intelligenceSummary && (
        <p className="mt-5 max-w-2xl font-body text-sm leading-6 text-ink-muted">
          {data.summary.intelligenceSummary}
        </p>
      )}
    </header>
  );
}

function InlineMetric({
  value,
  label,
  tone = "default",
  live = false,
}: {
  value: number;
  label: string;
  tone?: "default" | "warn" | "accent";
  live?: boolean;
}) {
  const numClass =
    tone === "warn"
      ? "text-verdict-warn"
      : tone === "accent"
        ? "text-accent-muted"
        : "text-ink-strong";
  return (
    <div>
      <div className="flex items-baseline gap-2">
        <p className={cn("font-mono text-3xl font-semibold leading-none", numClass)}>
          {value}
        </p>
        {live && (
          <span className="rounded-(--r-sm) border border-accent/40 bg-accent-soft px-1.5 py-0.5 font-body text-[9px] uppercase tracking-widest text-accent-muted">
            live
          </span>
        )}
      </div>
      <p className="mt-1.5 font-body text-xs text-ink-faint">{label}</p>
    </div>
  );
}

// ── Section label ─────────────────────────────────────────────────────────────

function SectionLabel({ children }: { children: React.ReactNode }) {
  return (
    <h2 className="mb-4 font-body text-[0.65rem] font-medium uppercase tracking-widest text-ink-faint">
      {children}
    </h2>
  );
}

// ── Opportunity spotlight card ─────────────────────────────────────────────────

function SpotlightCard({ item }: { item: AssessmentPipelineItem }) {
  const report = item.report;
  const colors = report ? bandColors(report.scoreBand) : null;
  const score = report?.feasibilityScore ?? item.site.previewScore;
  const ci = report?.confidenceInterval;

  const topFactors =
    report?.contributingFactors
      ?.filter((f) => f.direction === "positive")
      .slice(0, 2) ?? [];

  return (
    <article className="rounded-(--r-lg) border border-hairline bg-surface p-6 shadow-[var(--shadow-card)]">
      <div className="flex flex-col gap-5 sm:flex-row sm:items-start">
        {/* Score block — dominant on the left */}
        <div className="shrink-0 rounded-(--r-md) border border-hairline bg-sunken px-5 py-4 text-center sm:min-w-[100px]">
          <p
            className={cn(
              "font-mono text-[3rem] font-semibold leading-none",
              colors?.text ?? "text-accent-data"
            )}
          >
            {score ?? "--"}
          </p>
          <p className="mt-1.5 font-body text-[0.6rem] uppercase tracking-widest text-ink-faint">
            Feasibility
          </p>
          {ci && (
            <p className="mt-1 font-mono text-[0.65rem] text-ink-faint">
              {ci.low}–{ci.high}
            </p>
          )}
          {report && (
            <p
              className={cn(
                "mt-2 font-body text-[0.6rem] uppercase tracking-widest",
                colors?.text ?? "text-ink-faint"
              )}
            >
              {bandLabel(report.scoreBand)}
            </p>
          )}
        </div>

        {/* Location, concept, summary, factors, CTAs */}
        <div className="min-w-0 flex-1">
          <p className="font-body text-xs uppercase tracking-widest text-ink-faint">
            {item.businessType}
          </p>
          <h3 className="mt-1.5 font-display text-xl leading-tight text-ink-strong">
            {item.site.name}
          </h3>
          <p className="mt-2 font-body text-sm leading-6 text-ink-body">
            {item.summary}
          </p>

          {topFactors.length > 0 && (
            <div className="mt-3 flex flex-wrap gap-2">
              {topFactors.map((f) => (
                <span
                  key={f.label}
                  className="rounded-(--r-sm) border border-verdict-strong/40 bg-verdict-strong-wash px-2.5 py-1 font-body text-xs text-verdict-strong"
                >
                  {f.label}
                </span>
              ))}
            </div>
          )}

          <div className="mt-4 flex gap-2">
            <Link
              href="/workspace"
              className="inline-flex h-8 items-center rounded-(--r-md) border border-accent bg-accent px-3 font-body text-xs font-medium text-ink-strong transition-colors hover:bg-accent-hover"
            >
              Open workspace
            </Link>
            {report && (
              <Link
                href="/report"
                className="inline-flex h-8 items-center rounded-(--r-md) border border-hairline bg-transparent px-3 font-body text-xs font-medium text-ink-muted transition-colors hover:bg-sunken hover:text-ink-strong"
              >
                View report
              </Link>
            )}
          </div>
        </div>
      </div>
    </article>
  );
}

// ── Recommended actions list ──────────────────────────────────────────────────

function ActionsList({ actions }: { actions: RecommendedDashboardAction[] }) {
  return (
    <div className="divide-y divide-hairline rounded-(--r-lg) border border-hairline bg-surface overflow-hidden">
      {actions.map((action) => (
        <div
          key={action.id}
          className="flex items-start justify-between gap-4 px-5 py-4"
        >
          <div className="min-w-0">
            <div className="flex flex-wrap items-center gap-2">
              <p className="font-body text-sm font-medium text-ink-strong">
                {action.title}
              </p>
              {action.priority === "high" && (
                <span className="rounded-(--r-sm) border border-verdict-warn/50 bg-verdict-warn-wash px-2 py-0.5 font-body text-[9px] uppercase tracking-widest text-verdict-warn">
                  Urgent
                </span>
              )}
            </div>
            <p className="mt-0.5 font-body text-xs leading-5 text-ink-muted">
              {action.detail}
            </p>
          </div>
          <Link
            href={action.href}
            className="shrink-0 inline-flex h-7 items-center rounded-(--r-md) border border-hairline bg-transparent px-3 font-body text-xs text-ink-muted transition-colors hover:bg-sunken hover:text-ink-strong"
          >
            {action.actionLabel}
          </Link>
        </div>
      ))}
    </div>
  );
}

// ── Recent assessments table ──────────────────────────────────────────────────
// Reads from the live store (allConceptAssessments, new asset-centric model).
// Columns: location | concept | score + CI | band | date

function RecentAssessmentsTable({
  assessments,
}: {
  assessments: (ConceptAssessment & { asset?: Asset })[];
}) {
  return (
    <div className="overflow-hidden rounded-(--r-lg) border border-hairline bg-surface">
      {/* Header */}
      <div className="grid grid-cols-[minmax(0,1fr)_110px_64px_90px_100px] gap-3 bg-sunken px-5 py-2">
        {["Location", "Concept", "Score", "Band", "Date"].map((h) => (
          <p
            key={h}
            className={cn(
              "font-body text-[9px] uppercase tracking-widest text-ink-faint",
              h === "Score" || h === "Date" ? "text-right" : ""
            )}
          >
            {h}
          </p>
        ))}
      </div>

      {/* Rows */}
      <div className="divide-y divide-hairline">
        {assessments.map((a) => {
          const colors = bandColors(a.feasibility.scoreBand);
          const ci = a.feasibility.confidenceInterval;
          return (
            <div
              key={`${a.assetId}-${a.concept}`}
              className="grid grid-cols-[minmax(0,1fr)_110px_64px_90px_100px] gap-3 items-center px-5 py-3"
            >
              <div className="min-w-0">
                <p className="truncate font-body text-sm font-medium text-ink-strong">
                  {a.asset?.address ?? a.assetId}
                </p>
                {a.asset?.area && (
                  <p className="font-body text-xs text-ink-faint mt-0.5">
                    {a.asset.area}
                  </p>
                )}
              </div>
              <p className="truncate font-body text-xs text-ink-muted">
                {conceptLabel(a.concept)}
              </p>
              <div className="text-right">
                <p className={cn("font-mono text-sm font-semibold", colors.text)}>
                  {a.feasibility.feasibilityScore}
                </p>
                <p className="font-mono text-[9px] text-ink-faint">
                  {ci.low}–{ci.high}
                </p>
              </div>
              <span
                className={cn(
                  "self-start truncate rounded-(--r-sm) border px-2 py-0.5 font-body text-[10px]",
                  a.feasibility.scoreBand === "strong"
                    ? "border-verdict-strong/50 bg-verdict-strong-wash text-verdict-strong"
                    : a.feasibility.scoreBand === "moderate"
                      ? "border-verdict-warn/50 bg-verdict-warn-wash text-verdict-warn"
                      : "border-verdict-weak/50 bg-verdict-weak-wash text-verdict-weak"
                )}
              >
                {a.feasibility.scoreBand === "strong"
                  ? "Strong"
                  : a.feasibility.scoreBand === "moderate"
                    ? "Moderate"
                    : "Weak"}
              </span>
              <p className="font-mono text-[10px] text-ink-faint text-right">
                {formatDate(a.generatedAt)}
              </p>
            </div>
          );
        })}
      </div>
    </div>
  );
}

// ── Empty card ────────────────────────────────────────────────────────────────

function EmptyCard({
  title,
  detail,
  href,
  cta,
}: {
  title: string;
  detail: string;
  href: string;
  cta: string;
}) {
  return (
    <div className="rounded-(--r-lg) border border-dashed border-hairline bg-surface px-6 py-8 text-center">
      <p className="font-body text-sm font-medium text-ink-strong">{title}</p>
      <p className="mx-auto mt-2 max-w-sm font-body text-xs leading-5 text-ink-muted">
        {detail}
      </p>
      <Link
        href={href}
        className="mt-4 inline-flex h-8 items-center rounded-(--r-md) border border-hairline bg-transparent px-4 font-body text-xs font-medium text-ink-muted transition-colors hover:bg-sunken hover:text-ink-strong"
      >
        {cta}
      </Link>
    </div>
  );
}
