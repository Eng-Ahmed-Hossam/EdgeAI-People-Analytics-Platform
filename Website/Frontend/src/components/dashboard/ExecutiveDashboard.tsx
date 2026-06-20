"use client";

import Link from "next/link";
import { useMemo } from "react";
import { useQuery } from "@tanstack/react-query";
import {
  AttentionCard,
  BranchCard,
  EmptyState,
  ExecutiveSection,
  KPIBlock,
  LoadingState,
  RecommendationBanner,
} from "@/components/primitives/SahmPrimitives";
import { IntelligenceErrorState } from "@/components/primitives/IntelligenceStates";
import { getAssets, getExecutiveDashboard, queryKeys } from "@/lib/api";
import { cn, formatDate } from "@/lib/utils";
import { useAssessmentStore } from "@/store/assessmentStore";
import type {
  AssessmentPipelineItem,
  ExecutiveDashboardData,
} from "@/types/assessment";
import type { ConceptAssessment } from "@/types/assessment-v2";
import type { Asset } from "@/types/asset";

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
    () => new Map(assets.map((asset) => [asset.id, asset])),
    [assets]
  );

  const list = useMemo(() => {
    return Object.values(allConceptAssessments)
      .flatMap((record) =>
        Object.values(record).filter((assessment): assessment is ConceptAssessment => !!assessment)
      )
      .map((assessment) => ({ ...assessment, asset: assetMap.get(assessment.assetId) }))
      .sort((a, b) => new Date(b.generatedAt).getTime() - new Date(a.generatedAt).getTime())
      .slice(0, 6);
  }, [allConceptAssessments, assetMap]);

  const total = useMemo(
    () =>
      Object.values(allConceptAssessments).reduce(
        (sum, record) => sum + Object.keys(record).length,
        0
      ),
    [allConceptAssessments]
  );

  return { list, total, isLoading };
}

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

  const {
    list: sessionAssessments,
    total: sessionTotal,
    isLoading: assessmentsLoading,
  } = useSessionAssessments();

  if (dashboardQuery.isLoading) {
    return (
      <main className="min-h-full bg-page px-5 py-8">
        <div className="mx-auto max-w-[var(--content-max)]">
          <LoadingState
            title="Preparing Home"
            detail="Loading the current assessment activity and gated operating slots."
          />
        </div>
      </main>
    );
  }

  if (dashboardQuery.isError || !dashboardQuery.data) {
    return (
      <main className="min-h-full bg-page px-5 py-8">
        <IntelligenceErrorState
          className="mx-auto max-w-[var(--content-max)]"
          kicker="Home unavailable"
          title="Dashboard could not be loaded"
          summary="Existing assessments are preserved. Retry when the frontend API seam is available."
        />
      </main>
    );
  }

  const data = dashboardQuery.data;
  const needsReview = data.recentAssessments.filter(
    (item) => item.status === "needs_review"
  );
  const strongest = data.opportunitySpotlight[0] ?? null;

  return (
    <main className="min-h-full bg-page text-ink-body">
      <div className="mx-auto max-w-[var(--content-max)] px-4 py-6 lg:px-8 lg:py-8">
        <RecommendationBanner
          eyebrow="Today"
          title={
            needsReview.length > 0
              ? "Resolve the highest-risk decisions first"
              : "Keep operations gated; advance the strongest GROW opportunity"
          }
          reason={
            needsReview.length > 0
              ? `${needsReview.length} assessment${needsReview.length === 1 ? "" : "s"} need review before they are ready to share. RUN metrics remain gated until tenant-scoped operating data is connected.`
              : "Operational slots are ready for sales, stock, and cash data when contracts exist. Until then, Home shows honest setup states and recent assessment activity below the fold."
          }
          confidence={data.summary.highConfidenceOpportunities > 0 ? "GROW evidence ready" : "Building evidence"}
          impact="No fake operations"
          action={
            <Link
              href="/workspace"
              className="inline-flex h-9 items-center rounded-(--r-md) border border-accent bg-accent px-4 font-body text-sm font-semibold text-ink-strong hover:bg-accent-hover"
            >
              Open Intelligence Map
            </Link>
          }
        />

        <section className="mt-6 grid gap-4 lg:grid-cols-4">
          <KPIBlock
            label="Today"
            value="Setup required"
            status="Gated"
            locked
            detail="Sales, cash, and pace appear here after tenant-scoped operating APIs are connected."
          />
          <KPIBlock
            label="Assessment pipeline"
            value={data.summary.activeAssessments}
            detail="Active GROW decisions currently available to this frontend."
            status="Live"
          />
          <KPIBlock
            label="High confidence"
            value={data.summary.highConfidenceOpportunities}
            detail="Opportunities with enough evidence to consider next action."
            status="Evidence"
          />
          <KPIBlock
            label="Needs attention"
            value={data.summary.needsReview}
            detail="Items that should be reviewed before sharing a recommendation."
            status={data.summary.needsReview > 0 ? "Review" : "Clear"}
          />
        </section>

        <div className="mt-8 grid gap-6 xl:grid-cols-[minmax(0,1fr)_360px]">
          <div className="space-y-8">
            <ExecutiveSection label="Needs Attention" title="What to act on first">
              <div className="grid gap-3">
                {needsReview.length > 0 ? (
                  needsReview.slice(0, 3).map((item) => (
                    <AttentionCard
                      key={item.id}
                      title={item.site.name}
                      detail={item.attentionReason ?? item.nextStep}
                      actionLabel="Open workspace"
                      href="/workspace"
                      tone="warning"
                    />
                  ))
                ) : (
                  <AttentionCard
                    title="No urgent assessment review"
                    detail="Operational alerts are gated until RUN data exists. Continue with the strongest GROW candidate."
                    actionLabel="Review candidate"
                    href="/workspace"
                    tone="accent"
                  />
                )}
              </div>
            </ExecutiveSection>

            <ExecutiveSection label="Branch Health" title="Portfolio slots are ready, data is gated">
              <div className="grid gap-3 md:grid-cols-3">
                <BranchCard
                  name="All branches"
                  status="Gated"
                  detail="Branch health requires tenant, location, sales, and stock contracts."
                  locked
                />
                <BranchCard
                  name="Weakest first"
                  status="Pending"
                  detail="This sort order is prepared but not populated with fake branch data."
                  locked
                />
                <BranchCard
                  name="Location lens"
                  status="Pending"
                  detail="Real branch switching waits for membership and location scope."
                  locked
                />
              </div>
            </ExecutiveSection>

            <ExecutiveSection label="Recent Activity" title="Evidence already available">
              {assessmentsLoading && sessionAssessments.length === 0 ? (
                <LoadingState title="Loading activity" detail="Checking session assessments." />
              ) : sessionAssessments.length > 0 ? (
                <SessionActivity assessments={sessionAssessments} />
              ) : (
                <AssessmentActivity data={data.recentAssessments} />
              )}
            </ExecutiveSection>

            <ExecutiveSection label="Trends" title="Do not infer operations yet">
              <EmptyState
                eyebrow="Gated analytics"
                title="Operational trends require live business data"
                detail="Sales pace, cash movement, stock cover, and staff trends will appear after Phase 0 tenant wiring and RUN APIs. Current intelligence remains visible in GROW."
              />
            </ExecutiveSection>
          </div>

          <aside className="space-y-6">
            <ExecutiveSection label="Quick Actions" title="Next useful move">
              <div className="rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]">
                <div className="grid gap-2">
                  <QuickLink href="/workspace" label="Run a location assessment" />
                  <QuickLink href="/compare" label="Compare candidates" />
                  <QuickLink href="/report" label="Open executive report" />
                </div>
                <p className="mt-4 font-body text-xs leading-5 text-ink-faint">
                  Sales, inventory, customers, and expenses are visible in navigation as locked modules only.
                </p>
              </div>
            </ExecutiveSection>

            <ExecutiveSection label="GROW Spotlight" title="Best current candidate">
              {strongest ? (
                <Spotlight item={strongest} />
              ) : (
                <EmptyState
                  title="No candidate ready"
                  detail="Run a concept assessment in the Intelligence Map to populate this slot."
                  action={
                    <Link
                      href="/workspace"
                      className="rounded-(--r-md) border border-hairline px-3 py-2 font-body text-sm text-ink-muted hover:bg-raised hover:text-ink-body"
                    >
                      Open map
                    </Link>
                  }
                />
              )}
            </ExecutiveSection>

            <div className="rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]">
              <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
                Session assessments
              </p>
              <p className="mt-2 font-mono text-3xl font-semibold text-ink-strong">
                {sessionTotal}
              </p>
              <p className="mt-2 font-body text-xs leading-5 text-ink-muted">
                Session-only assessments are shown separately from persisted saved assessments.
              </p>
            </div>
          </aside>
        </div>
      </div>
    </main>
  );
}

function QuickLink({ href, label }: { href: string; label: string }) {
  return (
    <Link
      href={href}
      className="flex items-center justify-between rounded-(--r-md) border border-hairline bg-sunken px-3 py-2.5 font-body text-sm text-ink-muted hover:bg-raised hover:text-ink-body"
    >
      {label}
      <span className="font-mono text-xs text-accent-muted">Open</span>
    </Link>
  );
}

function Spotlight({ item }: { item: AssessmentPipelineItem }) {
  return (
    <article className="rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]">
      <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
        Recommended
      </p>
      <h3 className="mt-2 font-display text-lg leading-tight text-ink-strong">
        {item.site.name}
      </h3>
      <p className="mt-2 font-body text-sm leading-6 text-ink-muted">{item.summary}</p>
      <div className="mt-4 grid grid-cols-2 gap-3">
        <div className="rounded-(--r-md) border border-hairline bg-sunken p-3">
          <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
            Confidence
          </p>
          <p className="mt-1 font-mono text-sm font-semibold text-ink-strong">
            {item.confidenceLevel}
          </p>
        </div>
        <div className="rounded-(--r-md) border border-hairline bg-sunken p-3">
          <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
            Action
          </p>
          <p className="mt-1 font-body text-sm font-semibold text-ink-strong">
            {item.nextStep}
          </p>
        </div>
      </div>
    </article>
  );
}

function AssessmentActivity({ data }: { data: AssessmentPipelineItem[] }) {
  if (data.length === 0) {
    return (
      <EmptyState
        title="No assessment activity yet"
        detail="Select an asset and run a concept assessment to build the activity feed."
      />
    );
  }

  return (
    <div className="overflow-hidden rounded-(--r-lg) border border-hairline bg-surface shadow-[var(--shadow-card)]">
      {data.slice(0, 5).map((item) => (
        <div key={item.id} className="grid gap-3 border-b border-hairline px-4 py-3 last:border-b-0 md:grid-cols-[1fr_150px_120px]">
          <div className="min-w-0">
            <p className="truncate font-body text-sm font-semibold text-ink-strong">
              {item.site.name}
            </p>
            <p className="mt-1 font-body text-xs leading-5 text-ink-muted">
              {item.summary}
            </p>
          </div>
          <p className="font-body text-xs capitalize text-ink-muted">{item.status.replace(/_/g, " ")}</p>
          <p className="font-mono text-xs text-ink-faint md:text-right">{formatDate(item.updatedAt)}</p>
        </div>
      ))}
    </div>
  );
}

function SessionActivity({
  assessments,
}: {
  assessments: (ConceptAssessment & { asset?: Asset })[];
}) {
  return (
    <div className="overflow-hidden rounded-(--r-lg) border border-hairline bg-surface shadow-[var(--shadow-card)]">
      {assessments.map((assessment) => {
        const band = assessment.feasibility.scoreBand;
        return (
          <div key={`${assessment.assetId}-${assessment.concept}`} className="grid gap-3 border-b border-hairline px-4 py-3 last:border-b-0 md:grid-cols-[1fr_130px_120px_96px]">
            <div className="min-w-0">
              <p className="truncate font-body text-sm font-semibold text-ink-strong">
                {assessment.asset?.address ?? assessment.assetId}
              </p>
              <p className="mt-1 font-body text-xs text-ink-muted">
                {conceptLabel(assessment.concept)}
              </p>
            </div>
            <p className="font-body text-xs capitalize text-ink-muted">{band}</p>
            <p className="font-mono text-xs text-ink-muted">
              CI {assessment.feasibility.confidenceInterval.low}-{assessment.feasibility.confidenceInterval.high}
            </p>
            <p
              className={cn(
                "font-mono text-sm font-semibold md:text-right",
                band === "strong"
                  ? "text-verdict-strong"
                  : band === "moderate"
                    ? "text-verdict-warn"
                    : "text-verdict-weak"
              )}
            >
              {assessment.feasibility.feasibilityScore}
            </p>
          </div>
        );
      })}
    </div>
  );
}
