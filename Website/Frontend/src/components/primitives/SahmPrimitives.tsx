import Link from "next/link";
import type { ReactNode } from "react";
import { cn } from "@/lib/utils";

type Tone = "default" | "positive" | "warning" | "danger" | "accent" | "locked";

const toneClasses: Record<Tone, string> = {
  default: "border-hairline bg-sunken text-ink-muted",
  positive: "border-verdict-strong/45 bg-verdict-strong-wash text-verdict-strong",
  warning: "border-verdict-warn/45 bg-verdict-warn-wash text-verdict-warn",
  danger: "border-verdict-weak/45 bg-verdict-weak-wash text-verdict-weak",
  accent: "border-accent/45 bg-accent-soft text-accent-muted",
  locked: "border-hairline bg-raised/45 text-ink-faint",
};

export function ConfidenceBadge({
  label,
  detail,
  tone = "default",
  className,
}: {
  label: string;
  detail?: string;
  tone?: Tone;
  className?: string;
}) {
  return (
    <span
      className={cn(
        "inline-flex items-center gap-1.5 rounded-(--r-sm) border px-2.5 py-1",
        "font-body text-[11px] font-medium leading-none",
        toneClasses[tone],
        className
      )}
    >
      <span className="h-1.5 w-1.5 rounded-full bg-current" aria-hidden="true" />
      <span>{label}</span>
      {detail && <span className="font-mono text-[10px] opacity-75">{detail}</span>}
    </span>
  );
}

export function RecommendationBanner({
  eyebrow = "Recommended",
  title,
  reason,
  confidence,
  impact,
  action,
  tone = "accent",
  className,
}: {
  eyebrow?: string;
  title: string;
  reason: string;
  confidence?: string;
  impact?: string;
  action?: ReactNode;
  tone?: Tone;
  className?: string;
}) {
  return (
    <section
      className={cn(
        "rounded-(--r-lg) border px-5 py-5 shadow-[var(--shadow-card)]",
        toneClasses[tone],
        className
      )}
    >
      <div className="flex flex-col gap-4 lg:flex-row lg:items-start lg:justify-between">
        <div className="min-w-0">
          <p className="font-body text-[10px] font-semibold uppercase tracking-widest opacity-75">
            {eyebrow}
          </p>
          <h1 className="mt-2 font-display text-2xl leading-tight text-ink-strong lg:text-3xl">
            {title}
          </h1>
          <p className="mt-3 max-w-3xl font-body text-sm leading-6 text-ink-body">
            {reason}
          </p>
        </div>
        {(confidence || impact) && (
          <div className="grid min-w-58 grid-cols-2 gap-3 rounded-(--r-md) border border-hairline bg-page/35 p-3">
            {confidence && (
              <div>
                <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                  Confidence
                </p>
                <p className="mt-1 font-mono text-sm font-semibold text-ink-strong">
                  {confidence}
                </p>
              </div>
            )}
            {impact && (
              <div>
                <p className="font-body text-[10px] uppercase tracking-widest text-ink-faint">
                  Impact
                </p>
                <p className="mt-1 font-body text-sm font-semibold text-ink-strong">
                  {impact}
                </p>
              </div>
            )}
          </div>
        )}
      </div>
      {action && <div className="mt-5 flex flex-wrap gap-2">{action}</div>}
    </section>
  );
}

export function DecisionCard({
  eyebrow,
  title,
  reason,
  confidence,
  impact,
  action,
  children,
  className,
}: {
  eyebrow: string;
  title: string;
  reason: string;
  confidence?: string;
  impact?: string;
  action?: ReactNode;
  children?: ReactNode;
  className?: string;
}) {
  return (
    <article className={cn("rounded-(--r-lg) border border-hairline bg-surface p-5 shadow-[var(--shadow-card)]", className)}>
      <div className="flex flex-col gap-4 lg:flex-row lg:items-start lg:justify-between">
        <div className="min-w-0">
          <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
            {eyebrow}
          </p>
          <h2 className="mt-2 font-display text-xl leading-tight text-ink-strong">
            {title}
          </h2>
          <p className="mt-2 font-body text-sm leading-6 text-ink-muted">{reason}</p>
        </div>
        <div className="flex shrink-0 flex-wrap gap-2">
          {confidence && <ConfidenceBadge label={confidence} tone="accent" />}
          {impact && <ConfidenceBadge label={impact} tone="default" />}
        </div>
      </div>
      {children && <div className="mt-4">{children}</div>}
      {action && <div className="mt-5">{action}</div>}
    </article>
  );
}

export function ExecutiveSection({
  label,
  title,
  children,
  action,
  className,
}: {
  label: string;
  title?: string;
  children: ReactNode;
  action?: ReactNode;
  className?: string;
}) {
  return (
    <section className={cn("space-y-4", className)}>
      <div className="flex items-end justify-between gap-4">
        <div>
          <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
            {label}
          </p>
          {title && <h2 className="mt-1 font-display text-xl text-ink-strong">{title}</h2>}
        </div>
        {action}
      </div>
      {children}
    </section>
  );
}

export function KPIBlock({
  label,
  value,
  detail,
  status,
  locked = false,
  className,
}: {
  label: string;
  value: string | number;
  detail: string;
  status?: string;
  locked?: boolean;
  className?: string;
}) {
  return (
    <div className={cn("rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]", className)}>
      <div className="flex items-start justify-between gap-3">
        <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
          {label}
        </p>
        {status && <ConfidenceBadge label={status} tone={locked ? "locked" : "accent"} />}
      </div>
      <p className={cn("mt-3 font-mono text-2xl font-semibold tabular-nums", locked ? "text-ink-muted" : "text-ink-strong")}>
        {value}
      </p>
      <p className="mt-2 font-body text-xs leading-5 text-ink-muted">{detail}</p>
    </div>
  );
}

export function AttentionCard({
  title,
  detail,
  actionLabel,
  href,
  tone = "warning",
}: {
  title: string;
  detail: string;
  actionLabel?: string;
  href?: string;
  tone?: Tone;
}) {
  const content = (
    <>
      <div className="min-w-0">
        <p className="font-body text-sm font-semibold text-ink-strong">{title}</p>
        <p className="mt-1 font-body text-xs leading-5 text-ink-muted">{detail}</p>
      </div>
      {actionLabel && (
        <span className="shrink-0 font-body text-xs font-semibold text-accent-muted">
          {actionLabel}
        </span>
      )}
    </>
  );

  const className = cn(
    "flex items-start justify-between gap-4 rounded-(--r-md) border px-4 py-3 transition-colors",
    toneClasses[tone],
    href && "hover:border-accent/60 hover:bg-accent-soft"
  );

  return href ? (
    <Link href={href} className={className}>
      {content}
    </Link>
  ) : (
    <div className={className}>{content}</div>
  );
}

export function BranchCard({
  name,
  status,
  detail,
  locked = false,
}: {
  name: string;
  status: string;
  detail: string;
  locked?: boolean;
}) {
  return (
    <article className="rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]">
      <div className="flex items-start justify-between gap-3">
        <div>
          <p className="font-body text-sm font-semibold text-ink-strong">{name}</p>
          <p className="mt-1 font-body text-xs text-ink-faint">{detail}</p>
        </div>
        <ConfidenceBadge label={status} tone={locked ? "locked" : "positive"} />
      </div>
    </article>
  );
}

export function AssetBriefCard({
  title,
  subtitle,
  coverage,
  children,
}: {
  title: string;
  subtitle: string;
  coverage: number;
  children?: ReactNode;
}) {
  const tone = coverage >= 70 ? "positive" : coverage >= 45 ? "warning" : "danger";
  return (
    <article className="rounded-(--r-lg) border border-hairline bg-surface p-4 shadow-[var(--shadow-card)]">
      <div className="flex items-start justify-between gap-4">
        <div className="min-w-0">
          <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
            Asset brief
          </p>
          <h2 className="mt-1 truncate font-display text-lg text-ink-strong" title={title}>
            {title}
          </h2>
          <p className="mt-1 font-body text-xs text-ink-muted">{subtitle}</p>
        </div>
        <ConfidenceBadge label={`${coverage}/100`} detail="coverage" tone={tone} />
      </div>
      {children && <div className="mt-4">{children}</div>}
    </article>
  );
}

export function EmptyState({
  eyebrow = "Not available yet",
  title,
  detail,
  action,
  className,
}: {
  eyebrow?: string;
  title: string;
  detail: string;
  action?: ReactNode;
  className?: string;
}) {
  return (
    <section className={cn("rounded-(--r-lg) border border-dashed border-hairline bg-surface px-6 py-8 text-center", className)}>
      <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
        {eyebrow}
      </p>
      <h2 className="mt-2 font-display text-xl text-ink-strong">{title}</h2>
      <p className="mx-auto mt-3 max-w-lg font-body text-sm leading-6 text-ink-muted">
        {detail}
      </p>
      {action && <div className="mt-5 flex justify-center">{action}</div>}
    </section>
  );
}

export function LoadingState({
  title = "Loading",
  detail = "Preparing the workspace.",
}: {
  title?: string;
  detail?: string;
}) {
  return (
    <section className="rounded-(--r-lg) border border-hairline bg-surface p-6 shadow-[var(--shadow-card)]">
      <p className="font-body text-[10px] font-semibold uppercase tracking-widest text-ink-faint">
        {title}
      </p>
      <p className="mt-2 font-body text-sm text-ink-muted">{detail}</p>
      <div className="mt-6 space-y-3">
        <div className="h-3 w-3/4 animate-pulse rounded-full bg-raised" />
        <div className="h-3 w-1/2 animate-pulse rounded-full bg-raised" />
        <div className="grid gap-3 sm:grid-cols-3">
          {[0, 1, 2].map((item) => (
            <div key={item} className="h-24 animate-pulse rounded-(--r-md) border border-hairline bg-sunken" />
          ))}
        </div>
      </div>
    </section>
  );
}
